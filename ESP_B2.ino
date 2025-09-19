// === ESP B: BTS7960 driver (6 motors) ===
// RX/TX link from ESP A on UART2, parse: "CMD,<p1,...,p6>" with p in [-255..255]
// Safety: 200 ms watchdog -> stop all
// Pins from user:
//  rpwm = {4, 18, 21, 25, 26, 27}
//  lpwm = {5, 19, 22, 32, 33, 23}

#include <Arduino.h>

// ---------- Serial Links ----------
#define USB_BAUD   115200
#define LINK_BAUD  115200
#define UART_RX    16   // from ESP A TX
#define UART_TX    17   // to ESP A RX
HardwareSerial Link(2);

// ---------- Motor / PWM ----------
static const int N_MOTOR = 6;
const int RPWM[N_MOTOR] = {4};//, 18, 21, 25, 32, 23};  // CW
const int LPWM[N_MOTOR] = {5};//, 19, 22, 26, 33, 27};  // CCW

// LEDC settings
static const int PWM_FREQ   = 20000;  // 20 kHz (เงียบหู)
static const int PWM_RES    = 8;      // 8-bit -> duty 0..255
// จอง 12 ช่อง (2 ต่อมอเตอร์)
int chR[N_MOTOR];
int chL[N_MOTOR];

// ---------- Watchdog ----------
static const uint32_t WATCHDOG_MS = 200;
uint32_t last_cmd_ms = 0;
bool estopped = false;

// ---------- Helpers ----------
static inline int clamp255(int x) { return (x < -255) ? -255 : (x > 255) ? 255 : x; }

void pwmWritePair(int idx, int val) {
  // val > 0 -> CW on RPWM; val < 0 -> CCW on LPWM; 0 -> both 0 (coast)
  int v = clamp255(val);
  if (estopped) v = 0;

  if (v > 0) {
    ledcWrite(chR[idx], v);
    ledcWrite(chL[idx], 0);
  } else if (v < 0) {
    ledcWrite(chR[idx], 0);
    ledcWrite(chL[idx], -v);
  } else {
    ledcWrite(chR[idx], 0);
    ledcWrite(chL[idx], 0);
  }
}

void stopAll() {
  for (int i = 0; i < N_MOTOR; ++i) {
    ledcWrite(chR[i], 0);
    ledcWrite(chL[i], 0);
  }
}

void applyPWM6(const int p[6]) {
  for (int i = 0; i < N_MOTOR; ++i) pwmWritePair(i, p[i]);
}

// พิมพ์สถานะย่อกลับไปให้ดูใน Serial Monitor
void printApplied(const int p[6], const char* tag) {
  Serial.printf("%s [%d,%d,%d,%d,%d,%d]\n", tag, p[0], p[1], p[2], p[3], p[4], p[5]);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(USB_BAUD);
  delay(100);
  Link.begin(LINK_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  delay(100);

  // ตั้งค่า PWM/LEDC
  int ch = 0;
  for (int i = 0; i < N_MOTOR; ++i) {
    pinMode(RPWM[i], OUTPUT);
    pinMode(LPWM[i], OUTPUT);

    chR[i] = ch++;
    chL[i] = ch++;
    ledcSetup(chR[i], PWM_FREQ, PWM_RES);
    ledcSetup(chL[i], PWM_FREQ, PWM_RES);
    ledcAttachPin(RPWM[i], chR[i]);
    ledcAttachPin(LPWM[i], chL[i]);
    ledcWrite(chR[i], 0);
    ledcWrite(chL[i], 0);
  }

  last_cmd_ms = millis();
  Serial.println(F("ESP-B ready. Waiting for CMD,<p1..p6> or ESTOP/STOP"));
}

// ---------- Parser ----------
bool parseCmdLine(const String& s, int outP[6]) {
  // Expect: CMD,p1,p2,p3,p4,p5,p6
  // Return true if parsed successfully
  if (!s.startsWith("CMD")) return false;

  int start = s.indexOf(',');
  if (start < 0) return false;

  int idx = 0;
  int pos = start + 1;
  while (idx < 6) {
    int comma = s.indexOf(',', pos);
    String tok;
    if (comma < 0) { // last one
      tok = s.substring(pos);
    } else {
      tok = s.substring(pos, comma);
      pos = comma + 1;
    }
    tok.trim();
    if (tok.length() == 0) return false;
    outP[idx++] = clamp255(tok.toInt());
    if (comma < 0) break;
  }
  return (idx == 6);
}

// ---------- Loop ----------
void loop() {
  // รับข้อมูลจาก ESP A
  if (Link.available()) {
    String line = Link.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (line.equalsIgnoreCase("ESTOP") || line.equalsIgnoreCase("STOP")) {
        estopped = true;
        stopAll();
        Serial.println(F("[ESTOP] Motors disabled"));
        Link.println(F("[ESP-B] ESTOP ACK"));
      } else if (line.equalsIgnoreCase("CLEAR") || line.equalsIgnoreCase("RESET")) {
        estopped = false;
        Serial.println(F("[ESTOP] Cleared"));
        Link.println(F("[ESP-B] ESTOP CLEARED"));
      } else if (line.equalsIgnoreCase("PING")) {
        Link.println(F("PONG"));
      } else {
        int p[6];
        if (parseCmdLine(line, p)) {
          if (!estopped) {
            applyPWM6(p);
          } else {
            stopAll();
          }
          last_cmd_ms = millis();
          printApplied(p, "APPLIED");
          // (optional) echo กลับไปยัง ESP A
          // Link.printf("ACK,%d,%d,%d,%d,%d,%d\n", p[0],p[1],p[2],p[3],p[4],p[5]);
        } else {
          Serial.print(F("[WARN] Bad frame: "));
          Serial.println(line);
        }
      }
    }
  }

  // Watchdog
  uint32_t now = millis();
  if (!estopped && (now - last_cmd_ms) > WATCHDOG_MS) {
    stopAll();
    Serial.println(F("[WD] Timeout -> STOP ALL"));
    // ไม่ spam log บ่อย: รีเซ็ต last_cmd_ms เพื่อไม่พิมพ์รัว ๆ
    last_cmd_ms = now;
  }

  // Debug คำสั่งผ่าน USB (ช่วยทดสอบโดยไม่ต้องมี ESP A)
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) {
      if (s.equalsIgnoreCase("ESTOP") || s.equalsIgnoreCase("STOP")) {
        estopped = true; stopAll(); Serial.println(F("[USB] ESTOP"));
      } else if (s.equalsIgnoreCase("CLEAR") || s.equalsIgnoreCase("RESET")) {
        estopped = false; Serial.println(F("[USB] ESTOP CLEARED"));
      } else if (s.startsWith("CMD")) {
        int p[6];
        if (parseCmdLine(s, p)) { if (!estopped) applyPWM6(p); printApplied(p, "[USB]"); }
      } else {
        Serial.println(F("[USB] Use: CMD,p1..p6 | ESTOP | CLEAR"));
      }
    }
  }
}
