/*  ======================= ESP-A  (6WD Skid-Steer Controller)  =======================
    จุดประสงค์: รับคำสั่งความเร็วจากผู้ใช้/ROS2  (vd, rd) แล้วแปลงเป็นความเร็วล้อ 6 ล้อ
    - สมการการเคลื่อนที่ (Kinematics) แบบ skid-steer: vL = vd - (b/2)*rd, vR = vd + (b/2)*rd
    - กระจายลง 6 ล้อ (ซ้าย 3, ขวา 3) แล้วแปลงเป็น PWM (เชิงเส้น, ไม่มี deadband)
    - คง loop 50ms, คง encoder วัดความเร็วล้อจริง (เอาไว้ log/ตรวจการทำงาน)
    - ส่งคำสั่งไป ESP-B: "CMD,<p1..p6>\r\n"

    รูปแบบคำสั่งทาง USB:
      - SETVR,<vd>,<rd>   // vd [m/s], rd [rad/s]
      - SET,<v>           // วิ่งตรง (เทียบเท่า SETVR,<v>,0)
      - ESTOP / STOP      // หยุดฉุกเฉิน

    อิงแนวคิดคิเนเมติก 6WD จาก RobuROC6 (sliding-mode paper) นำมาปรับใช้ระดับใช้งานจริงบน MCU
    ================================================================================== */

#include <Arduino.h>

/*================= [CONFIG: ฮาร์ดแวร์เอนโค้ดเดอร์/ล้อ] =================*/
const int N_WHEEL = 6;
const int encA[N_WHEEL] = {4, 18, 21, 25, 32, 34};
const int encB[N_WHEEL] = {5, 19, 22, 26, 33, 35};

// สเปก encoder/gear/wheel (ใช้คำนวณ v_meas จากการนับพัลส์)
const float PPR   = 16.0f;     // pulses per rev @ motor shaft
const float GEAR  = 99.5f;     // motor:wheel
const int   QUAD  = 1;         // x1 (ISR: RISING on A)
const float R     = 0.075f;    // wheel radius (m)
const float TPR   = PPR * GEAR * QUAD;  // ticks per wheel-rev (≈ 1592)

/*================= [CONFIG: ไทม์มิ่ง & ขีดจำกัด] =================*/
const unsigned long SAMPLE_MS = 50; // loop period ~50ms
const float V_MIN  = -1.0f;         // m/s  ลิมิตความเร็วล้อ
const float V_MAX  =  1.0f;         // m/s
const float A_MAX  =  0.10f;        // m/s^2 ลิมิตความเร่ง (slew vd)

const float TRACK_B = 0.46f;        // m  ระยะศูนย์ล้อซ้าย-ขวา (ตั้งตามรถจริง)
const float R_MIN   = -1.50f;       // rad/s ลิมิตอัตราเลี้ยว
const float R_MAX   =  1.50f;       // rad/s

/*================= [CONFIG: UART ไป ESP-B] =================*/
#define UART_TX 17
#define UART_RX 16
HardwareSerial Link(2);

/*================= [STATE & TELEMETRY] =================*/
// นับพัลส์ encoder (interrupt-safe)
volatile long encCount[N_WHEEL] = {0};
long lastCount[N_WHEEL]         = {0};

// ค่าที่คำนวณต่อเฟรม (การวัดจริงของล้อ)
float wheelRPM[N_WHEEL]   = {0};  // rpm
float wheelOmega[N_WHEEL] = {0};  // rad/s
float wheelV[N_WHEEL]     = {0};  // m/s
float v_meas              = 0.0f;  // m/s เฉลี่ย 6 ล้อ

// คำสั่งเป้าหมาย (จากผู้ใช้/ROS2)
float vd_target = 0.0f;           // m/s  ความเร็วตามยาวที่อยากได้
float rd_target = 0.0f;           // rad/s อัตราเลี้ยวที่อยากได้

// คำสั่งหลัง slew/limit (ตัวที่ “ใช้จริง” ในเฟรมนี้)
float vd_cmd = 0.0f;              // m/s
float rd_cmd = 0.0f;              // rad/s

unsigned long lastCompute = 0;

/*================= [ISR: นับพัลส์จาก encoder A, ใช้ทิศจาก B] =================*/
// ทั้งก้อนนี้คือ "อินพุตการวัดความเร็วล้อแบบเรียลไทม์"
static inline void handleEncoderEdge(uint8_t i) {
  int dir = digitalRead(encB[i]) ? +1 : -1;  // HIGH=+1, LOW=-1 (ปรับตามการต่อสาย)
  encCount[i] += dir;
}
void IRAM_ATTR isr0() { handleEncoderEdge(0); }
void IRAM_ATTR isr1() { handleEncoderEdge(1); }
void IRAM_ATTR isr2() { handleEncoderEdge(2); }
void IRAM_ATTR isr3() { handleEncoderEdge(3); }
void IRAM_ATTR isr4() { handleEncoderEdge(4); }
void IRAM_ATTR isr5() { handleEncoderEdge(5); }
void (*isrTable[N_WHEEL])() = {isr0, isr1, isr2, isr3, isr4, isr5};

/*================= [UTIL: ค่าช่วยทั่วไป] =================*/
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

/*================= [PARSER: รับคำสั่งจาก USB] =================*/
// ทั้งก้อนนี้คือ "อินเทอร์เฟซควบคุมจากภายนอก"
bool parseSET(const String &s, float &v_out) {
  int c = s.indexOf(',');
  if (c < 0) return false;
  String cmd = s.substring(0, c), sn = s.substring(c+1);
  cmd.trim(); sn.trim();
  if (!cmd.equalsIgnoreCase("SET")) return false;
  v_out = sn.toFloat();
  return true;
}
bool parseSETVR(const String &s, float &vd_out, float &rd_out) {
  if (!s.startsWith("SETVR")) return false;
  int c1 = s.indexOf(','), c2 = s.indexOf(',', c1+1);
  if (c1 < 0 || c2 < 0) return false;
  String svd = s.substring(c1+1, c2), srd = s.substring(c2+1);
  svd.trim(); srd.trim();
  vd_out = svd.toFloat();
  rd_out = srd.toFloat();
  return true;
}

/*================= [MATH: แมปความเร็ว → PWM แบบเชิงเส้น (ไม่มี deadband)] =================*/
// ทั้งก้อนนี้คือ "การแปลงคำสั่งความเร็วล้อไปเป็นสัญญาณ PWM ต่อ-ล้อ"
int pwm_from_v_linear(float v) {
  // สเกลเชิงเส้น: v=V_MAX -> +255, v=-V_MAX -> -255
  float p = 255.0f * (v / V_MAX);
  p = clampf(p, -255.0f, 255.0f);
  return (int)roundf(p);
}

/*================= [COMMS: ส่ง PWM 6 ล้อไป ESP-B] =================*/
// ทั้งก้อนนี้คือ "เอาต์พุตคำสั่งมอเตอร์รายล้อ"
void sendPWM6(const int p[6]) {
  char buf[96];
  snprintf(buf, sizeof(buf), "CMD,%d,%d,%d,%d,%d,%d\r\n", p[0],p[1],p[2],p[3],p[4],p[5]);
  Link.print(buf);
}

/*================= [SENSING: คำนวณความเร็วล้อจริงจาก encoder] =================*/
// ทั้งก้อนนี้คือ "ซับซิสเต็มวัดผล/เทเลเมทรี" ใช้ยืนยันผลลัพธ์และดีบัก
void computeWheelKinematics(float dt) {
  long snap[N_WHEEL];
  noInterrupts();
  for (int i = 0; i < N_WHEEL; ++i) snap[i] = encCount[i];
  interrupts();

  float sum_v = 0.0f;
  for (int i = 0; i < N_WHEEL; ++i) {
    long dcnt = snap[i] - lastCount[i];
    lastCount[i] = snap[i];

    wheelRPM[i]   = (dcnt / TPR) * (60.0f / dt);
    wheelOmega[i] = wheelRPM[i] * (2*PI / 60.0f);
    wheelV[i]     = wheelOmega[i] * R;
    sum_v += wheelV[i];
  }
  v_meas = sum_v / (float)N_WHEEL;
}

/*================= [COMMAND SHAPING: จำกัด/ไล่ vd & rd ด้วย slew/limits] =================*/
// ทั้งก้อนนี้คือ "คอนดิชันนิ่งคำสั่ง" เพื่อความนุ่มนวลและปลอดภัย
void shapeCommands(float dt) {
  // ไล่ vd_cmd → vd_target ด้วยเพดานความเร่ง A_MAX
  {
    float dv = vd_target - vd_cmd;
    float step = clampf(dv, -A_MAX*dt, +A_MAX*dt);
    vd_cmd = clampf(vd_cmd + step, V_MIN, V_MAX);
  }
  // ไล่ rd_cmd → rd_target ด้วยเพดานเทียบเคียง (แปลงจาก A_MAX กับแขนโมเมนต์ประมาณ)
  {
    float r_slew = (A_MAX / max(TRACK_B*0.5f, 0.01f)); // rad/s^2 (คร่าว ๆ)
    float dr = rd_target - rd_cmd;
    float step = clampf(dr, -r_slew*dt, +r_slew*dt);
    rd_cmd = clampf(rd_cmd + step, R_MIN, R_MAX);
  }
}

/*================= [MOTION EQNS: สมการการเคลื่อนที่ของหุ่น (Skid-Steer Kinematics)] =================*/
// ทั้งก้อนนี้คือ "หัวใจคณิตศาสตร์" ที่แปลง (vd,rd) → (vL,vR) แล้วแจกลง 6 ล้อ
void computeWheelSetpoints(float v_set[6]) {
  // คิเนเมติกพื้นฐาน 6WD skid-steer (อิงหลักจากวรรณกรรม RobuROC6)
  float vL = vd_cmd - 0.5f * TRACK_B * rd_cmd;   // m/s
  float vR = vd_cmd + 0.5f * TRACK_B * rd_cmd;   // m/s

  // ลิมิตป้องกันผิดพลาด
  vL = clampf(vL, V_MIN, V_MAX);
  vR = clampf(vR, V_MIN, V_MAX);

  // กระจาย 3 ล้อต่อข้าง (เริ่มด้วยน้ำหนักเท่ากันก่อน)
  v_set[0] = vL;  v_set[1] = vL;  v_set[2] = vL;   // ซ้าย: หน้า-กลาง-หลัง
  v_set[3] = vR;  v_set[4] = vR;  v_set[5] = vR;   // ขวา: หน้า-กลาง-หลัง
}

/*================= [CYCLE: เฟรมหลัก 50ms] =================*/
// ทั้งก้อนนี้คือ "เมนลูปควบคุมระดับล้อ" เรียงลอจิก: วัด → ปั้นคำสั่ง → คิเนเมติก → PWM → ส่ง
void controlCycle() {
  unsigned long now = millis();
  if (now - lastCompute < SAMPLE_MS) return;

  float dt = (now - lastCompute) / 1000.0f;
  lastCompute = now;
  if (dt <= 0) return;

  computeWheelKinematics(dt);    // วัดของจริงจาก encoder (ไว้เทียบ/ล็อก)
  shapeCommands(dt);             // จำกัด/ไล่คำสั่ง vd, rd ให้เนียนและอยู่ในลิมิต

  float v_set[6];
  computeWheelSetpoints(v_set);  // สมการการเคลื่อนที่ของหุ่น → v_set รายล้อ

  int p[6];
  for (int i = 0; i < 6; ++i) p[i] = pwm_from_v_linear(v_set[i]);  // ไม่มี deadband

  sendPWM6(p);                   // ส่งไป ESP-B

  // ----- LOG -----
  Serial.print("vd="); Serial.print(vd_target,3);
  Serial.print(" rd="); Serial.print(rd_target,3);
  Serial.print(" | cmd vd="); Serial.print(vd_cmd,3);
  Serial.print(" rd="); Serial.print(rd_cmd,3);
  Serial.print(" | vL/R=(");
  Serial.print(v_set[0],3); Serial.print(","); Serial.print(v_set[3],3); Serial.print(")");
  Serial.print(" | v_meas="); Serial.print(v_meas,3);
  Serial.print(" | PWM L(");
  Serial.print(p[0]); Serial.print(","); Serial.print(p[1]); Serial.print(","); Serial.print(p[2]);
  Serial.print(") R(");
  Serial.print(p[3]); Serial.print(","); Serial.print(p[4]); Serial.print(","); Serial.print(p[5]);
  Serial.println(")");
}

/*================= [SETUP] =================*/
// ทั้งก้อนนี้คือ "บูตระบบ I/O, Interrupt, Serial"
void setup() {
  Serial.begin(115200);
  delay(100);

  for (int i = 0; i < N_WHEEL; ++i) {
    // หมายเหตุ: GPIO34/35 ไม่มี internal pull-up → INPUT ธรรมดา (ควรมี pull-up ภายนอก)
    if (encA[i] == 34 || encA[i] == 35) pinMode(encA[i], INPUT);
    else                                pinMode(encA[i], INPUT_PULLUP);
    if (encB[i] == 34 || encB[i] == 35) pinMode(encB[i], INPUT);
    else                                pinMode(encB[i], INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encA[i]), isrTable[i], RISING); // x1
  }

  Link.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(50);

  lastCompute = millis();
  Serial.println("ESP-A 6WD (vd,rd → v_set[6]) ready. Use: SETVR,<vd>,<rd> | SET,<v> | ESTOP");
}

/*================= [LOOP] =================*/
// ทั้งก้อนนี้คือ "รับคำสั่ง, E-STOP, และเรียก controlCycle() ต่อเนื่อง"
void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) {
      if (s.equalsIgnoreCase("ESTOP") || s.equalsIgnoreCase("STOP")) {
        vd_target = 0.0f; rd_target = 0.0f;
        vd_cmd = 0.0f;    rd_cmd = 0.0f;
        int pz[6] = {0,0,0,0,0,0};
        sendPWM6(pz);
        Serial.println("[CMD] E-STOP -> PWM=0 for all wheels");
      } else {
        float vd_in, rd_in;
        if (parseSETVR(s, vd_in, rd_in)) {
          vd_target = clampf(vd_in, V_MIN, V_MAX);
          rd_target = clampf(rd_in, R_MIN, R_MAX);
          Serial.print("[CMD] SETVR vd="); Serial.print(vd_target,3);
          Serial.print(" rd=");             Serial.println(rd_target,3);
        } else {
          float v_in;
          if (parseSET(s, v_in)) {          // backward compatible
            vd_target = clampf(v_in, V_MIN, V_MAX);
            rd_target = 0.0f;
            Serial.print("[CMD] SET vd="); Serial.println(vd_target,3);
          } else {
            Serial.println("[WARN] Unknown. Use: SETVR,<vd>,<rd> | SET,<v> | ESTOP");
          }
        }
      }
    }
  }

  controlCycle();
}
