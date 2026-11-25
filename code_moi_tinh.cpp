/*
 * ============================================================================
 * ROBOT DO LINE THI DAU - DUONG TOI TAN CUONG THAI NGUYEN LAN THU 2 (ICTU)
 * Phien ban v5: AP DUNG KIEN TRUC CHUAN OPRobots (DragonBot2 4m/s)
 * ============================================================================
 * Tai sao v4 mat line (va v3 bi lac cua), va v5 sua triet de bang DragonBot2:
 *   1) QUAY LAI 100% SLOW-DECAY BRAKE PWM (y het DragonBot2/motors.c):
 *      - v4 doi sang Fast-Decay (Coast) lam duong cong toc do N20 phi tuyen
 *        (50-80% Coast motor van chay ~90% toc do -> mat han luc be lai!).
 *      - Slow-Decay tuyen tinh 100% theo PWM, o 1023 VAN LA 100% DIEN AP PIN!
 *   2) TANG TAN SO DIEU KHIEN LEN 1000 Hz (1 ms) + BO LOC TRE PHA PHAN MEM:
 *      - Theo DragonBot2 ROADMAP: moi bo loc thong thap phan mem deu gay tre pha,
 *        an mon du tru pha va gay LAC CUA (mach ban da co san tu loc RC C5..C8!).
 *   3) TRONG SO HINH HOC THUC TUYEN TINH {-1000, -562, +562, +1000}:
 *      - Khop chinh xac toa do that (-20mm, -11.25mm, +11.25mm, +20mm), xoa buoc
 *        nhay 3x gia tao gay giat lac khi chuyen tu mat trong S2/S3 ra S1/S4!
 *   4) PD THUAN TUYEN TINH TY LE KD/KP = 6:1 (chuan DragonBot2/kinematics.c):
 *      - Ki = 0, khong meo bac 2, KD/KP = 6:1 o 1000 Hz triet tieu lac cua!
 *      - Giu SPEED_BASE = 1023 (100% FULL GA CA THANG LAN CUA), mixer thuan
 *        L = base + turn, R = base - turn -> banh ngoai LUON 1023, banh trong
 *        chay nhanh +350..+750 trong cua tron & chi phanh/dao chieu o cua 180!
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// 1. KHU VUC TINH CHINH NHANH (SUA TRUC TIEP O DAY!)
// ============================================================================

// --- A. Khoi dong & Che do thi dau ---
constexpr uint32_t START_DELAY_MS        = 5000;   // Cho 5 giay sau khi gat cong tac pin (di thi giam con 300-800ms)
constexpr uint32_t SOFT_START_MS         = 350;    // Tang ga muot 350ms luc xuat phat -> KHONG SUT AP (nhung sau do van 100% ga!)
constexpr bool     ENABLE_LOOP2_SHORTCUT = false;  // false = An toan | true = DI TAT re PHAI 90 do tai Vong tron 2
constexpr bool     DEBUG_SERIAL          = false;  // false = Chay thi toc do toi da | true = In debug ra Serial (115200)

// --- B. Toc do 100% FULL-POWER + Chong sut ap kieu Dien tu Cong suat ---
// Nguyen ly giu 100% TOC DO CAO ma KHONG SUT AP:
//   - Trong STATE_FOLLOW: banh trong chi phanh toi 0 (Slow-Decay 1/1 ton 0mA pin -> khong sut ap & bo cua nhanh hon!),
//     KHONG bao gio dao chieu am dot ngot khi dang chay 1023!
//   - Chi cho phep dao chieu nhe (-340) trong STATE_LOST_RECOVERY sau khi phanh 25ms triet tieu suc dien dong nguoc!
constexpr int16_t  SPEED_BASE            = 1023;   // 100% FULL GA - KHONG HA 1 DON VI NAO!
constexpr int16_t  SPEED_MAX_OUT         = 1023;   // Kich kim banh ngoai
constexpr int16_t  FOLLOW_MIN_INNER      = 0;      // Phanh cung = 0 (0 mA dong pin -> KHONG SUT AP trong cua!)
constexpr int16_t  SPEED_MAX_REVERSE     = 340;    // Chi dung trong cuu line khan cap (vua du vuot deadband 306)

// Bo gioi han dinh dong (chong sut ap pin AA nhung phan xa phanh vao cua tuc thi!):
constexpr int16_t  SLEW_ACCEL_LOW_MS     = 15;     // Tang ga muot khi speed < 600 hoac dao chieu (cat dinh dong ket truc)
constexpr int16_t  SLEW_ACCEL_HIGH_MS    = 40;     // Tang ga nhanh khi speed >= 600 (da co da, khong sut ap)
constexpr int16_t  SLEW_BRAKE_PER_MS     = 90;     // Phanh ve 0 cuc nhanh (ton 0mA pin -> vao cua sieu nhay!)
constexpr uint32_t REVERSE_DEADTIME_MS   = 25;     // Giữ phanh 0 trong 25ms truoc khi dao chieu -> giam 50% soc dong!

// Bu lech co khi giua 2 dong co N20 (neu di thang hoi keo sang phai -> giam LEFT con 0.96)
constexpr float    LEFT_MOTOR_TRIM       = 1.00f;
constexpr float    RIGHT_MOTOR_TRIM      = 1.00f;

// --- C. Tham so PD chuan DragonBot2 v5.2 (1000 Hz, bo cua NHANH HON & HET KHUNG!) ---
//   - KP = 1.05 giup banh trong giu toc do cao (+430..+710, vuot xa deadband 306) -> cua NHANH & MUOT!
//   - KD = 6.30 (ty le 6:1) ket hop dao ham cua so 2ms -> loc sach xung nhieu gay khung banh trong!
//   - Muon cua NHANH HON NUA (neu xe chua vang cua): giam nhẹ KP (1.05 -> 0.95) va KD (6.3 -> 5.7)
//   - Neu thay pin xin 6.2V ma cua 180 hoi troi mep ngoai: tang KP (1.05 -> 1.18) va KD (6.3 -> 7.1)
constexpr float    PID_KP                = 1.05f;
constexpr float    PID_KD                = 6.30f;

// Trong so HINH HOC THUC theo khoang cach vat ly (-20.0, -11.25, +11.25, +20.0 mm)
// quy doi ve thang [-1000 .. +1000] chuan DragonBot2:
constexpr float    SENSOR_WEIGHTS[4]     = { -1000.0f, -562.0f, +562.0f, +1000.0f };

// --- D. Nguong cam bien IR (da do thuc te: Trang=2832mV, Den=15-240mV) ---
constexpr float    WHITE_MV              = 2720.0f;
constexpr float    BLACK_MV              = 250.0f;
constexpr float    BLACK_NOISE_FLOOR     = 0.10f;
constexpr float    LINE_PRESENT_SUM_MIN  = 0.25f;

// --- E. Xu ly mat line / Khe ho 22mm / Go giam toc (kieu DragonBot2/sensors.c) ---
// Khi mat line luc dang thang (|linePos| < GAP_HOLD_POS_TH): giu nguyen huong cu trong GAP_BRIDGE_MAX_MS!
// Khi mat line luc dang cua   (|linePos| >= GAP_HOLD_POS_TH): xoay gat cuu line ngay lap tuc!
constexpr float    GAP_HOLD_POS_TH       = 420.0f; // Chi giu thang khi thuc su gan tam (<420/1000)
constexpr uint32_t GAP_BRIDGE_MAX_MS     = 50;     // 50ms full ga (~3cm). Khi co go nhua 0.5cm tang len 75-90ms
constexpr int16_t  RECOVERY_OUTER_SPEED  = 1023;   // Banh ngoai 100% ga
constexpr int16_t  RECOVERY_INNER_SPEED  = -340;   // Dao chieu em sau 25ms phanh -> KHONG SUT AP!
constexpr float    RECOVERY_INNER_TH     = 0.35f;  // Thoat recovery ngay khi mat trong S3/S2 bat lai vach den
constexpr uint32_t RECOVERY_FALLBACK_MS  = 250;

// --- F. Xu ly Nga tu cheo X (Vong tron 1: 100% ga di thang | Vong tron 2: di tat tuy chon) ---
constexpr float    CROSS_OUTER_TH        = 0.65f;
constexpr float    CROSS_SUM_MIN         = 2.65f;
constexpr float    CROSS_MAX_ENTRY_POS   = 500.0f; // Chi nhan nga tu khi truoc do dang tuong doi thang
constexpr uint32_t CROSS_CONFIRM_MS      = 6;
constexpr uint32_t CROSS_REFRACTORY_MS   = 1100;
constexpr uint32_t CROSS_STRAIGHT_MS     = 85;
constexpr int16_t  SPEED_CROSS_STRAIGHT  = 1023;

// Thong so re PHAI 90 do di tat tai Vong tron 2 (chi chay khi ENABLE_LOOP2_SHORTCUT = true)
constexpr uint32_t SHORTCUT_ADVANCE_MS   = 45;
constexpr uint32_t SHORTCUT_MIN_TURN_MS  = 110;
constexpr uint32_t SHORTCUT_MAX_TURN_MS  = 450;
constexpr int16_t  SHORTCUT_TURN_SPEED   = 820;

// ============================================================================
// 2. DINH NGHIA PHAN CUNG (DA CHOT 100% TU TEST THUC TE)
// ============================================================================
constexpr uint8_t  IR_PINS[4]     = { 0, 1, 3, 4 };  // [0]=S4, [1]=S3, [2]=S2, [3]=S1
constexpr uint8_t  PIN_IR_ON      = 21;
constexpr uint8_t  PIN_LED_ON     = 20;

constexpr uint8_t  PIN_ML_A       = 2;   // Motor 1 (Banh TRAI)
constexpr uint8_t  PIN_ML_B       = 10;
constexpr uint8_t  PIN_MR_A       = 5;   // Motor 2 (Banh PHAI)
constexpr uint8_t  PIN_MR_B       = 6;

constexpr uint32_t PWM_FREQ       = 6000;   // 6 kHz: giam gon dong ripple tren VBAT -> it sut ap hon!
constexpr uint8_t  PWM_RES        = 10;
constexpr int16_t  PWM_MAX        = (1 << PWM_RES) - 1;

constexpr uint8_t  CH_ML_A        = 0;
constexpr uint8_t  CH_ML_B        = 1;
constexpr uint8_t  CH_MR_A        = 2;
constexpr uint8_t  CH_MR_B        = 3;

// Vong lap 1000 Hz (1 ms) chuan DragonBot2!
constexpr uint32_t LOOP_PERIOD_US = 1000;

// ============================================================================
// 3. BIEN TRANG THAI HE THONG
// ============================================================================
enum RobotState : uint8_t {
  STATE_FOLLOW = 0,
  STATE_GAP_BRIDGE,
  STATE_LOST_RECOVERY,
  STATE_CROSS_STRAIGHT,
  STATE_SHORTCUT_ADVANCE,
  STATE_SHORTCUT_TURN
};

RobotState state             = STATE_FOLLOW;
uint32_t   stateStartMs      = 0;
uint32_t   runStartMs        = 0;
uint32_t   lastLoopUs        = 0;
uint32_t   lastDebugMs       = 0;

float      linePos           = 0.0f;   // Vi tri line hien tai [-1000 .. +1000]
float      prevPos1          = 0.0f;   // Vi tri line 1ms truoc
float      prevPos2          = 0.0f;   // Vi tri line 2ms truoc (cho dao ham 2ms cuc em, het khung banh!)
int8_t     recoveryDir       = 1;      // -1 = xoay TRAI, +1 = xoay PHAI

uint8_t    crossingCount     = 0;
uint32_t   crossConfirmStart = 0;
uint32_t   lastCrossingMs    = 0;

int16_t    lastLeftCmd       = 0;
int16_t    lastRightCmd      = 0;
int16_t    appliedLeftCmd    = 0;      // Gia tri PWM thuc te sau khi qua bo chong sut ap (Slew-Rate Limiter)
int16_t    appliedRightCmd   = 0;
uint32_t   leftZeroHoldUntil = 0;      // Dead-time phanh 0 truoc khi dao chieu chong soc dong 2x I_stall
uint32_t   rightZeroHoldUntil= 0;

// ============================================================================
// 4. HAM DIEU KHIEN PHAN CUNG (100% SLOW-DECAY BRAKE PWM CHUAN DRAGONBOT2)
// ============================================================================
void pwmInitPin(uint8_t pin, uint8_t ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  (void)ch;
  ledcAttach(pin, PWM_FREQ, PWM_RES);
  ledcWrite(pin, 0);
#else
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
#endif
}

inline void pwmWritePin(uint8_t pin, uint8_t ch, uint16_t duty) {
  if (duty > (uint16_t)PWM_MAX) duty = PWM_MAX;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

// 100% Slow-Decay Brake PWM (y het DragonBot2/motors.c):
//   - QUAN TRONG: Kep doi xung [-PWM_MAX .. +PWM_MAX] (-1023..+1023) vi banh PHAI
//     quay TIEN bang gia tri AM (-1023)! Gioi han dao chieu SPEED_MAX_REVERSE
//     da duoc xu ly truoc do trong applySlewLimit()!
void driveBridgeSlowDecay(uint8_t pinA, uint8_t chA, uint8_t pinB, uint8_t chB, int16_t speed) {
  if (speed > PWM_MAX)  speed = PWM_MAX;
  if (speed < -PWM_MAX) speed = -PWM_MAX;

  uint16_t mag = (uint16_t)abs(speed);
  uint16_t inv = (uint16_t)PWM_MAX - mag;

  if (speed >= 0) {
    pwmWritePin(pinA, chA, PWM_MAX);
    pwmWritePin(pinB, chB, inv);
  } else {
    pwmWritePin(pinA, chA, inv);
    pwmWritePin(pinB, chB, PWM_MAX);
  }
}

// Gioi han dong thong minh + Dead-time 25ms khi dao chieu (chong sut ap nhung GIU 100% TOC DO CAO!):
int16_t applySlewLimit(int16_t currentVal, int16_t targetVal, uint32_t &zeroHoldUntilMs, uint32_t nowMs) {
  if (targetVal > SPEED_MAX_OUT)      targetVal = SPEED_MAX_OUT;
  if (targetVal < -SPEED_MAX_REVERSE) targetVal = -SPEED_MAX_REVERSE;

  // Neu dang trong cua so dead-time phanh 0 truoc khi dao chieu -> giu 0 de triet back-EMF
  if (currentVal == 0 && targetVal < 0 && nowMs < zeroHoldUntilMs) {
    return 0;
  }

  int16_t diff = targetVal - currentVal;
  if (diff == 0) return currentVal;

  bool movingTowardZero = (currentVal > 0 && targetVal < currentVal) ||
                          (currentVal < 0 && targetVal > currentVal);
  if (movingTowardZero) {
    int16_t nextVal = (diff > 0) ? (currentVal + SLEW_BRAKE_PER_MS) : (currentVal - SLEW_BRAKE_PER_MS);
    if ((currentVal > 0 && nextVal <= 0) || (currentVal < 0 && nextVal >= 0)) {
      if (targetVal < 0 && currentVal > 0) {
        zeroHoldUntilMs = nowMs + REVERSE_DEADTIME_MS; // Phanh 25ms truoc khi dao chieu!
      }
      return 0;
    }
    if ((diff > 0 && nextVal > targetVal) || (diff < 0 && nextVal < targetVal)) {
      return targetVal;
    }
    return nextVal;
  }

  // Tang ga / tang dao chieu: vung >=600 nhanh (40/ms), vung thap <600 muot (15/ms) de khong sut ap!
  int16_t maxStep = (currentVal >= 600 && targetVal > 0) ? SLEW_ACCEL_HIGH_MS : SLEW_ACCEL_LOW_MS;
  if (diff > maxStep)       return currentVal + maxStep;
  else if (diff < -maxStep) return currentVal - maxStep;
  return targetVal;
}

void setMotors(int16_t leftSpeed, int16_t rightSpeed) {
  uint32_t nowMs   = millis();
  uint32_t elapsed = nowMs - runStartMs;
  if (elapsed < SOFT_START_MS) {
    float scale = 0.25f + 0.75f * ((float)elapsed / (float)SOFT_START_MS);
    leftSpeed  = (int16_t)(leftSpeed  * scale);
    rightSpeed = (int16_t)(rightSpeed * scale);
  }

  int16_t targetL = (int16_t)(leftSpeed  * LEFT_MOTOR_TRIM);
  int16_t targetR = (int16_t)(rightSpeed * RIGHT_MOTOR_TRIM);

  appliedLeftCmd  = applySlewLimit(appliedLeftCmd,  targetL, leftZeroHoldUntil,  nowMs);
  appliedRightCmd = applySlewLimit(appliedRightCmd, targetR, rightZeroHoldUntil, nowMs);

  lastLeftCmd  = appliedLeftCmd;
  lastRightCmd = appliedRightCmd;

  // Banh TRAI (M1): duong = TIEN | Banh PHAI (M2): am = TIEN
  driveBridgeSlowDecay(PIN_ML_A, CH_ML_A, PIN_ML_B, CH_ML_B,  appliedLeftCmd);
  driveBridgeSlowDecay(PIN_MR_A, CH_MR_A, PIN_MR_B, CH_MR_B, -appliedRightCmd);
}

void motorsCoastOff() {
  pwmWritePin(PIN_ML_A, CH_ML_A, 0);
  pwmWritePin(PIN_ML_B, CH_ML_B, 0);
  pwmWritePin(PIN_MR_A, CH_MR_A, 0);
  pwmWritePin(PIN_MR_B, CH_MR_B, 0);
  appliedLeftCmd  = 0;
  appliedRightCmd = 0;
  lastLeftCmd     = 0;
  lastRightCmd    = 0;
}

// Doc nhanh 2 mau/kenh (~240us tong cho 4 kenh -> dam bao vong lap 1000 Hz muot!)
void readSensors(uint16_t mvOut[4], float bOut[4], float &sumB) {
  sumB = 0.0f;
  const float invSpan = 1.0f / (WHITE_MV - BLACK_MV);

  for (uint8_t i = 0; i < 4; i++) {
    uint32_t acc = analogReadMilliVolts(IR_PINS[i]);
    acc += analogReadMilliVolts(IR_PINS[i]);
    uint16_t mv = (uint16_t)(acc >> 1);
    mvOut[i] = mv;

    float b = (WHITE_MV - (float)mv) * invSpan;
    if (b < BLACK_NOISE_FLOOR) b = 0.0f;
    else if (b > 1.0f)         b = 1.0f;

    bOut[i] = b;
    sumB += b;
  }
}

// ============================================================================
// 5. SETUP (AN TOAN + CHO 5 GIAY TRUOC KHI XUAT PHAT)
// ============================================================================
void setup() {
  pinMode(PIN_ML_A, OUTPUT); digitalWrite(PIN_ML_A, LOW);
  pinMode(PIN_ML_B, OUTPUT); digitalWrite(PIN_ML_B, LOW);
  pinMode(PIN_MR_A, OUTPUT); digitalWrite(PIN_MR_A, LOW);
  pinMode(PIN_MR_B, OUTPUT); digitalWrite(PIN_MR_B, LOW);

  pwmInitPin(PIN_ML_A, CH_ML_A);
  pwmInitPin(PIN_ML_B, CH_ML_B);
  pwmInitPin(PIN_MR_A, CH_MR_A);
  pwmInitPin(PIN_MR_B, CH_MR_B);
  motorsCoastOff();

  pinMode(PIN_LED_ON, OUTPUT);
  digitalWrite(PIN_LED_ON, LOW);

  pinMode(PIN_IR_ON, OUTPUT);
  digitalWrite(PIN_IR_ON, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(IR_PINS[i], INPUT);
    analogSetPinAttenuation(IR_PINS[i], ADC_11db);
  }

  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    delay(400);
    Serial.println(F("\n=== ROBOT ICTU v5 - DRAGONBOT2 1kHz PD RACING ==="));
  }

  uint32_t waitStart = millis();
  while (millis() - waitStart < START_DELAY_MS) {
    delay(5);
  }

  runStartMs     = millis();
  lastCrossingMs = runStartMs;
  lastLoopUs     = micros();
  state          = STATE_FOLLOW;
}

// ============================================================================
// 6. VONG LAP DIEU KHIEN CHINH (1000 Hz CO DINH - KHONG TRE PHA)
// ============================================================================
void loop() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_PERIOD_US) {
    return;
  }
  const float dtMs = (float)(nowUs - lastLoopUs) * 0.001f; // ~1.0 ms
  lastLoopUs = nowUs;
  uint32_t nowMs = millis();

  // 1. Doc 4 cam bien [S4, S3, S2, S1]
  uint16_t mv[4];
  float b[4];
  float sumB = 0.0f;
  readSensors(mv, b, sumB);

  bool hasLine = (sumB >= LINE_PRESENT_SUM_MIN);

  float newPos = linePos;
  if (hasLine) {
    float weighted = b[0] * SENSOR_WEIGHTS[0]
                   + b[1] * SENSOR_WEIGHTS[1]
                   + b[2] * SENSOR_WEIGHTS[2]
                   + b[3] * SENSOR_WEIGHTS[3];
    newPos = weighted / sumB;
  }

  // 2. Phat hien Nga tu cheo X
  if (state == STATE_FOLLOW &&
      (nowMs - lastCrossingMs >= CROSS_REFRACTORY_MS) &&
      (fabsf(linePos) <= CROSS_MAX_ENTRY_POS)) {
    bool crossPattern = (b[0] >= CROSS_OUTER_TH && b[3] >= CROSS_OUTER_TH && sumB >= CROSS_SUM_MIN);
    if (crossPattern) {
      if (crossConfirmStart == 0) {
        crossConfirmStart = nowMs;
      } else if (nowMs - crossConfirmStart >= CROSS_CONFIRM_MS) {
        crossingCount++;
        lastCrossingMs    = nowMs;
        crossConfirmStart = 0;
        stateStartMs      = nowMs;

        if (ENABLE_LOOP2_SHORTCUT && crossingCount == 3) {
          state = STATE_SHORTCUT_ADVANCE;
        } else {
          state = STATE_CROSS_STRAIGHT;
        }
      }
    } else {
      crossConfirmStart = 0;
    }
  } else {
    crossConfirmStart = 0;
  }

  // 3. May trang thai dieu khien
  switch (state) {

    // ------------------------------------------------------------------------
    case STATE_CROSS_STRAIGHT: {
      setMotors(SPEED_CROSS_STRAIGHT, SPEED_CROSS_STRAIGHT);
      if (nowMs - stateStartMs >= CROSS_STRAIGHT_MS) {
        state    = STATE_FOLLOW;
        linePos  = 0.0f;
        prevPos1 = 0.0f;
        prevPos2 = 0.0f;
      }
      break;
    }

    // ------------------------------------------------------------------------
    case STATE_SHORTCUT_ADVANCE: {
      setMotors(SPEED_CROSS_STRAIGHT, SPEED_CROSS_STRAIGHT);
      if (nowMs - stateStartMs >= SHORTCUT_ADVANCE_MS) {
        state        = STATE_SHORTCUT_TURN;
        stateStartMs = nowMs;
      }
      break;
    }

    // ------------------------------------------------------------------------
    case STATE_SHORTCUT_TURN: {
      setMotors(+SHORTCUT_TURN_SPEED, -SHORTCUT_TURN_SPEED);
      uint32_t turnElapsed = nowMs - stateStartMs;
      bool caughtNewBranch = (turnElapsed >= SHORTCUT_MIN_TURN_MS) &&
                             (b[2] >= RECOVERY_INNER_TH || b[1] >= RECOVERY_INNER_TH);
      if (caughtNewBranch || turnElapsed >= SHORTCUT_MAX_TURN_MS) {
        state    = STATE_FOLLOW;
        linePos  = 0.0f;
        prevPos1 = 0.0f;
        prevPos2 = 0.0f;
      }
      break;
    }

    // ------------------------------------------------------------------------
    case STATE_GAP_BRIDGE: {
      if (hasLine) {
        state = STATE_FOLLOW;
        // Fall-through tinh PD ngay lap tuc
      } else if (nowMs - stateStartMs < GAP_BRIDGE_MAX_MS) {
        setMotors(SPEED_BASE, SPEED_BASE);
        break;
      } else {
        recoveryDir  = (linePos >= 0.0f) ? +1 : -1;
        state        = STATE_LOST_RECOVERY;
        stateStartMs = nowMs;
        break;
      }
    }
    // fallthrough when hasLine == true

    // ------------------------------------------------------------------------
    case STATE_FOLLOW: {
      if (!hasLine) {
        stateStartMs = nowMs;
        if (fabsf(linePos) < GAP_HOLD_POS_TH) {
          // Dang gan tam ma mat vach ngan -> giu thang full ga vuot khe/go!
          state = STATE_GAP_BRIDGE;
          setMotors(SPEED_BASE, SPEED_BASE);
        } else {
          // Mat line o cua -> ghim bien & xoay cuu line ngay lap tuc!
          recoveryDir = (linePos > 0.0f) ? +1 : -1;
          linePos     = (recoveryDir > 0) ? +1000.0f : -1000.0f;
          prevPos1    = linePos;
          prevPos2    = linePos;
          state       = STATE_LOST_RECOVERY;
          if (recoveryDir > 0) setMotors(RECOVERY_OUTER_SPEED, RECOVERY_INNER_SPEED);
          else                 setMotors(RECOVERY_INNER_SPEED, RECOVERY_OUTER_SPEED);
        }
        break;
      }

      // Cap nhat lich su 2ms & tinh PD cua so 2ms (giam doi nhieu ADC, het khung banh trong cua!)
      prevPos2 = prevPos1;
      prevPos1 = linePos;
      linePos  = newPos;

      float dPos = (linePos - prevPos2) / (2.0f * ((dtMs > 0.3f) ? dtMs : 1.0f));
      float turn = (PID_KP * linePos) + (PID_KD * dPos);

      // Mixer chuan DragonBot2: L = base + turn, R = base - turn
      // KHONG cho dao chieu am trong STATE_FOLLOW (giu >= FOLLOW_MIN_INNER = 0):
      //   - Phanh tai 0 trong Slow-Decay = luc phanh cuc dai + 0 mA dong pin -> KHONG SUT AP!
      //   - Banh ngoai LUON KICH KIM 1023 -> TOC DO CUC CAO!
      int16_t leftSpeed  = SPEED_BASE + (int16_t)turn;
      int16_t rightSpeed = SPEED_BASE - (int16_t)turn;
      if (leftSpeed  < FOLLOW_MIN_INNER) leftSpeed  = FOLLOW_MIN_INNER;
      if (rightSpeed < FOLLOW_MIN_INNER) rightSpeed = FOLLOW_MIN_INNER;

      setMotors(leftSpeed, rightSpeed);
      break;
    }

    // ------------------------------------------------------------------------
    case STATE_LOST_RECOVERY: {
      bool caughtInner = (b[1] >= RECOVERY_INNER_TH || b[2] >= RECOVERY_INNER_TH);
      bool fallbackAny = ((nowMs - stateStartMs >= RECOVERY_FALLBACK_MS) && hasLine);

      if (caughtInner || fallbackAny) {
        state    = STATE_FOLLOW;
        linePos  = newPos;
        prevPos1 = newPos;
        prevPos2 = newPos; // Reset dPos = 0 khi vua bat lai line de tranh giat!
      } else {
        if (recoveryDir > 0) setMotors(RECOVERY_OUTER_SPEED, RECOVERY_INNER_SPEED);
        else                 setMotors(RECOVERY_INNER_SPEED, RECOVERY_OUTER_SPEED);
      }
      break;
    }
  }

  if (DEBUG_SERIAL && (nowMs - lastDebugMs >= 100)) {
    lastDebugMs = nowMs;
    Serial.printf("ST:%u X:%u | mV:%4u/%4u/%4u/%4u | pos:%+6.1f | L:%+4d R:%+4d\n",
                  (uint8_t)state, crossingCount,
                  mv[0], mv[1], mv[2], mv[3],
                  linePos, lastLeftCmd, lastRightCmd);
  }
}
