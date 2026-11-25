/*
 * ============================================================================
 * ROBOT DO LINE THI DAU - DUONG TOI TAN CUONG THAI NGUYEN LAN THU 2 (ICTU)
 * Phien ban v5.1-BB: DRAGONBOT2 1kHz + BLACKBOX TELEMETRY (RAM -> LittleFS)
 * ============================================================================
 * Cau hinh test:
 *   - PID_KP = 0.85f (giam tu 1.05 -> giu banh trong nhanh hon trong cua)
 *   - PID_KD = 6.80f (tang nhe tu 6.30 -> tang giam chan chong lac)
 *   - BlackBox ghi chi tiet 22 truong: thoi gian, dtUs, 4x mV, 4x b[],
 *     linePos, dPos, pTerm, dTerm, turn, targetL/R, appliedL/R, state, flags!
 *
 * Huong dan lay du lieu BlackBox:
 *   1) Cho xe chay tren san. Khi chay xong, NHAC XE LEN khoi san > 1.5 giay.
 *      -> Motor tu tat, LED nhay 3 lan bao da luu xong vao Flash.
 *   2) Tat pin, cam cap USB vao may tinh, mo Serial Monitor (115200 baud).
 *   3) Trong 5 giay cho khoi dong, nhap 'd' + Enter -> huy chay & in TOAN BO!
 *      Cac lenh khac:
 *        d = In tom tat + TOAN BO du lieu CSV BlackBox
 *        s = Chi in bang tom tat phan tich nhanh (0.1 giay)
 *        r = Ep in truc tiep du lieu dang nam trong RAM
 *        c = Xoa sach BlackBox trong Flash & RAM
 *        g = Cho xe chay lai (sau 3 giay)
 * ============================================================================
 */

#include <Arduino.h>
#include <LittleFS.h>

// ============================================================================
// 1. KHU VUC TINH CHINH NHANH
// ============================================================================

// --- A. Khoi dong & Che do thi dau ---
constexpr uint32_t START_DELAY_MS         = 5000;   // Cho 5s (trong luc nay nhap 'd' de xem BlackBox)
constexpr uint32_t SOFT_START_MS          = 350;    // Tang ga muot 350ms luc xuat phat
constexpr bool     ENABLE_LOOP2_SHORTCUT  = false;  // false = An toan | true = Di tat vong 2
constexpr bool     DEBUG_SERIAL           = false;  // Giu false khi dua de dat 1000 Hz chuan!

// --- B. Cau hinh BlackBox ---
constexpr uint32_t BB_MAX_SAMPLES         = 4500;   // Toi da 4500 mau (~171 KB RAM, tu ha neu thieu RAM)
constexpr uint16_t BB_SAMPLE_DIVIDER      = 5;      // 5 = ghi moi 5ms (200Hz -> 22.5 giay)
                                                    // Doi thanh 1 = 1000Hz (4.5s) | 10 = 100Hz (45s)
constexpr uint32_t BB_AUTO_SAVE_NOLINE_MS = 1500;   // Nhac xe/mat line lien tuc 1.5s -> tat motor & luu Flash
constexpr uint32_t BB_MIN_VALID_LINE_MS   = 500;    // Phai bam line >= 0.5s moi ghi de Flash (tranh loi tren ban)

// --- C. Toc do 100% FULL-POWER + Chong sut ap ---
constexpr int16_t  SPEED_BASE             = 1023;
constexpr int16_t  SPEED_MAX_OUT          = 1023;
constexpr int16_t  FOLLOW_MIN_INNER       = 0;
constexpr int16_t  SPEED_MAX_REVERSE      = 340;

constexpr int16_t  SLEW_ACCEL_LOW_MS      = 15;
constexpr int16_t  SLEW_ACCEL_HIGH_MS     = 40;
constexpr int16_t  SLEW_BRAKE_PER_MS      = 90;
constexpr uint32_t REVERSE_DEADTIME_MS    = 25;

constexpr float    LEFT_MOTOR_TRIM        = 1.00f;
constexpr float    RIGHT_MOTOR_TRIM       = 1.00f;

// --- D. Tham so PD (Da giam KP -> 0.85, tang nhe KD -> 6.80) ---
constexpr float    PID_KP                 = 0.85f;
constexpr float    PID_KD                 = 6.80f;

constexpr float    SENSOR_WEIGHTS[4]      = { -1000.0f, -562.0f, +562.0f, +1000.0f };

// --- E. Nguong cam bien IR ---
constexpr float    WHITE_MV               = 2720.0f;
constexpr float    BLACK_MV               = 250.0f;
constexpr float    BLACK_NOISE_FLOOR      = 0.10f;
constexpr float    LINE_PRESENT_SUM_MIN   = 0.25f;

// --- F. Xu ly mat line / Khe ho / Go giam toc ---
constexpr float    GAP_HOLD_POS_TH        = 420.0f;
constexpr uint32_t GAP_BRIDGE_MAX_MS      = 50;
constexpr int16_t  RECOVERY_OUTER_SPEED   = 1023;
constexpr int16_t  RECOVERY_INNER_SPEED   = -340;
constexpr float    RECOVERY_INNER_TH      = 0.35f;
constexpr uint32_t RECOVERY_FALLBACK_MS   = 250;

// --- G. Xu ly Nga tu cheo X ---
constexpr float    CROSS_OUTER_TH         = 0.65f;
constexpr float    CROSS_SUM_MIN          = 2.65f;
constexpr float    CROSS_MAX_ENTRY_POS    = 500.0f;
constexpr uint32_t CROSS_CONFIRM_MS       = 6;
constexpr uint32_t CROSS_REFRACTORY_MS    = 1100;
constexpr uint32_t CROSS_STRAIGHT_MS      = 85;
constexpr int16_t  SPEED_CROSS_STRAIGHT   = 1023;

constexpr uint32_t SHORTCUT_ADVANCE_MS    = 45;
constexpr uint32_t SHORTCUT_MIN_TURN_MS   = 110;
constexpr uint32_t SHORTCUT_MAX_TURN_MS   = 450;
constexpr int16_t  SHORTCUT_TURN_SPEED    = 820;

// ============================================================================
// 2. DINH NGHIA PHAN CUNG
// ============================================================================
constexpr uint8_t  IR_PINS[4]     = { 0, 1, 3, 4 };  // [0]=S4, [1]=S3, [2]=S2, [3]=S1
constexpr uint8_t  PIN_IR_ON      = 21;
constexpr uint8_t  PIN_LED_ON     = 20;

constexpr uint8_t  PIN_ML_A       = 2;   // Motor 1 (Banh TRAI)
constexpr uint8_t  PIN_ML_B       = 10;
constexpr uint8_t  PIN_MR_A       = 5;   // Motor 2 (Banh PHAI)
constexpr uint8_t  PIN_MR_B       = 6;

constexpr uint32_t PWM_FREQ       = 6000;
constexpr uint8_t  PWM_RES        = 10;
constexpr int16_t  PWM_MAX        = (1 << PWM_RES) - 1;

constexpr uint8_t  CH_ML_A        = 0;
constexpr uint8_t  CH_ML_B        = 1;
constexpr uint8_t  CH_MR_A        = 2;
constexpr uint8_t  CH_MR_B        = 3;

constexpr uint32_t LOOP_PERIOD_US = 1000;

// ============================================================================
// 3. CAU TRUC DU LIEU BLACKBOX & ENUM (DAT TRUOC TAT CA CAC HAM CHO ARDUINO IDE)
// ============================================================================
constexpr uint32_t    BB_MAGIC     = 0x42425635UL; // "BBV5"
constexpr uint16_t    BB_VERSION   = 1;
constexpr const char* BB_FILE_PATH = "/blackbox.bin";

enum RobotState : uint8_t {
  STATE_FOLLOW = 0,
  STATE_GAP_BRIDGE,
  STATE_LOST_RECOVERY,
  STATE_CROSS_STRAIGHT,
  STATE_SHORTCUT_ADVANCE,
  STATE_SHORTCUT_TURN,
  STATE_BENCH_IDLE          // Trang thai cho Serial / da luu xong BlackBox (motor OFF)
};

struct __attribute__((packed)) BlackBoxHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t sampleSize;
  uint32_t sampleCount;
  uint32_t totalLoopTicks;
  uint32_t sawLineMs;
  uint16_t samplePeriodMs;
  uint16_t maxDtUs;
  float    kp;
  float    kd;
  int16_t  speedBase;
  int16_t  slewAccelLow;
  int16_t  slewAccelHigh;
  int16_t  slewBrake;
  uint16_t lostCount;
  uint16_t gapCount;
  uint8_t  crossCount;
  uint8_t  stopReason;      // 0=Dang chay, 1=Nhac xe/mat line >1.5s, 2=Day RAM, 3=Lenh Serial
  uint8_t  reserved[2];
};

struct __attribute__((packed)) BlackBoxSample {
  uint32_t tMs;             // Thoi gian tu luc xuat phat (ms)
  uint16_t dtUs;            // Chu ky vong loop thuc te (us)
  uint16_t mv[4];           // Dien ap tho 4 cam bien [S4, S3, S2, S1] (mV)
  uint8_t  b[4];            // Do den chuan hoa 0..250 (chia 250.0 -> 0.000..1.000)
  int16_t  linePos;         // Vi tri line [-1000 .. +1000]
  int16_t  dPos10;          // Dao ham dPos * 10 (pos/ms * 10)
  int16_t  pTerm;           // Thanh phan P = KP * linePos
  int16_t  dTerm;           // Thanh phan D = KD * dPos
  int16_t  turn;            // Tong luc be lai = P + D
  int16_t  targetL;         // Lenh banh TRAI TRUOC Slew-Rate (PD muon)
  int16_t  targetR;         // Lenh banh PHAI TRUOC Slew-Rate (PD muon)
  int16_t  appliedL;        // PWM THUC TE ra banh TRAI (SAU Slew-Rate)
  int16_t  appliedR;        // PWM THUC TE ra banh PHAI (SAU Slew-Rate)
  uint8_t  state;           // Trang thai FSM (0..6)
  uint8_t  flags;           // bit0:hasLine | bit1:crossPat | bit2:slewL | bit3:slewR | bit4..7:crossCnt
};

struct BlackBoxStats {
  uint32_t count;
  double   sumAppL, sumAppR, sumTgtL, sumTgtR;
  double   sumAbsPos, sumAbsP, sumAbsD;
  uint32_t slewLimitedSamples;
  uint32_t innerUnderDeadband; // So mau banh trong < 306 trong luc STATE_FOLLOW
  uint32_t outerDropped;       // So mau banh ngoai < 900 trong luc STATE_FOLLOW
  uint32_t stateCounts[7];
  int16_t  maxAbsD;
  int16_t  maxAbsTurn;
  uint16_t maxDtUs;
  uint32_t durationMs;
};

// ============================================================================
// 4. BIEN TRANG THAI HE THONG & BLACKBOX
// ============================================================================
RobotState state             = STATE_FOLLOW;
uint32_t   stateStartMs      = 0;
uint32_t   runStartMs        = 0;
uint32_t   lastLoopUs        = 0;
uint32_t   lastDebugMs       = 0;
uint32_t   lastSeenLineMs    = 0;

float      linePos           = 0.0f;
float      prevPos1          = 0.0f;
float      prevPos2          = 0.0f;
int8_t     recoveryDir       = 1;

uint8_t    crossingCount     = 0;
uint32_t   crossConfirmStart = 0;
uint32_t   lastCrossingMs    = 0;

int16_t    rawTargetL        = 0;
int16_t    rawTargetR        = 0;
int16_t    lastLeftCmd       = 0;
int16_t    lastRightCmd      = 0;
int16_t    appliedLeftCmd    = 0;
int16_t    appliedRightCmd   = 0;
uint32_t   leftZeroHoldUntil = 0;
uint32_t   rightZeroHoldUntil= 0;

// Bien phuc vu BlackBox
bool            littleFsReady = false;
BlackBoxSample* bbBuf         = nullptr;
uint32_t        bbCapacity    = 0;
uint32_t        bbCount       = 0;
uint32_t        bbLoopTicks   = 0;
uint32_t        bbSawLineMs   = 0;
uint16_t        bbMaxDtUs     = 0;
uint16_t        bbLostCount   = 0;
uint16_t        bbGapCount    = 0;
uint8_t         bbStopReason  = 0;
bool            bbSavedFlash  = false;

float           lastDPos      = 0.0f;
float           lastPTerm     = 0.0f;
float           lastDTerm     = 0.0f;
float           lastTurn      = 0.0f;

// ============================================================================
// 5. HAM DIEU KHIEN PHAN CUNG (100% SLOW-DECAY BRAKE PWM)
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

int16_t applySlewLimit(int16_t currentVal, int16_t targetVal, uint32_t &zeroHoldUntilMs, uint32_t nowMs) {
  if (targetVal > SPEED_MAX_OUT)      targetVal = SPEED_MAX_OUT;
  if (targetVal < -SPEED_MAX_REVERSE) targetVal = -SPEED_MAX_REVERSE;

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
        zeroHoldUntilMs = nowMs + REVERSE_DEADTIME_MS;
      }
      return 0;
    }
    if ((diff > 0 && nextVal > targetVal) || (diff < 0 && nextVal < targetVal)) {
      return targetVal;
    }
    return nextVal;
  }

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

  if (targetL > SPEED_MAX_OUT)      targetL = SPEED_MAX_OUT;
  if (targetL < -SPEED_MAX_REVERSE) targetL = -SPEED_MAX_REVERSE;
  if (targetR > SPEED_MAX_OUT)      targetR = SPEED_MAX_OUT;
  if (targetR < -SPEED_MAX_REVERSE) targetR = -SPEED_MAX_REVERSE;

  rawTargetL = targetL;
  rawTargetR = targetR;

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
  rawTargetL      = 0;
  rawTargetR      = 0;
  appliedLeftCmd  = 0;
  appliedRightCmd = 0;
  lastLeftCmd     = 0;
  lastRightCmd    = 0;
}

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
// 6. HE THONG BLACKBOX (GHI RAM -> LUU FLASH -> XUAT SERIAL)
// ============================================================================
inline int16_t clampInt16(float val) {
  if (val > 32000.0f)  return 32000;
  if (val < -32000.0f) return -32000;
  return (int16_t)(val >= 0.0f ? (val + 0.5f) : (val - 0.5f));
}

const char* stateName(uint8_t st) {
  switch (st) {
    case STATE_FOLLOW:           return "FOLLOW";
    case STATE_GAP_BRIDGE:       return "GAP";
    case STATE_LOST_RECOVERY:    return "LOST";
    case STATE_CROSS_STRAIGHT:   return "CROSS";
    case STATE_SHORTCUT_ADVANCE: return "SC_ADV";
    case STATE_SHORTCUT_TURN:    return "SC_TRN";
    case STATE_BENCH_IDLE:       return "IDLE";
    default:                     return "UNK";
  }
}

void bbInit() {
  littleFsReady = LittleFS.begin(true);

  uint32_t maxAlloc     = ESP.getMaxAllocHeap();
  uint32_t reserveBytes = 52 * 1024; // Giu lai >= 52 KB cho LittleFS, Serial va Stack
  uint32_t usableBytes  = (maxAlloc > reserveBytes) ? (maxAlloc - reserveBytes) : 0;
  uint32_t maxFit       = usableBytes / sizeof(BlackBoxSample);
  bbCapacity            = (maxFit < BB_MAX_SAMPLES) ? maxFit : BB_MAX_SAMPLES;

  if (bbCapacity > 0) {
    bbBuf = (BlackBoxSample*)malloc(bbCapacity * sizeof(BlackBoxSample));
    if (bbBuf == nullptr) {
      bbCapacity = 0;
    }
  }
}

void bbResetRun() {
  bbCount      = 0;
  bbLoopTicks  = 0;
  bbSawLineMs  = 0;
  bbMaxDtUs    = 0;
  bbLostCount  = 0;
  bbGapCount   = 0;
  bbStopReason = 0;
  bbSavedFlash = false;
  lastDPos     = 0.0f;
  lastPTerm    = 0.0f;
  lastDTerm    = 0.0f;
  lastTurn     = 0.0f;
}

// Ghi 1 mau vao RAM (~1 us, 100% non-blocking trong vong lap 1000 Hz!)
inline void bbRecordSample(uint32_t tMs, uint16_t dtUs,
                           const uint16_t mv[4], const float b[4],
                           bool hasLine, bool crossPat) {
  if (bbBuf == nullptr || bbCount >= bbCapacity) return;

  BlackBoxSample &s = bbBuf[bbCount++];
  s.tMs      = tMs;
  s.dtUs     = dtUs;
  for (uint8_t i = 0; i < 4; i++) {
    s.mv[i] = mv[i];
    float scaled = b[i] * 250.0f;
    if (scaled < 0.0f)   scaled = 0.0f;
    if (scaled > 250.0f) scaled = 250.0f;
    s.b[i] = (uint8_t)(scaled + 0.5f);
  }
  s.linePos  = clampInt16(linePos);
  s.dPos10   = clampInt16(lastDPos * 10.0f);
  s.pTerm    = clampInt16(lastPTerm);
  s.dTerm    = clampInt16(lastDTerm);
  s.turn     = clampInt16(lastTurn);
  s.targetL  = rawTargetL;
  s.targetR  = rawTargetR;
  s.appliedL = appliedLeftCmd;
  s.appliedR = appliedRightCmd;
  s.state    = (uint8_t)state;

  uint8_t fl = 0;
  if (hasLine)                       fl |= (1 << 0);
  if (crossPat)                      fl |= (1 << 1);
  if (appliedLeftCmd != rawTargetL)  fl |= (1 << 2);
  if (appliedRightCmd != rawTargetR) fl |= (1 << 3);
  uint8_t xc = (crossingCount > 15) ? 15 : crossingCount;
  fl |= (uint8_t)(xc << 4);
  s.flags = fl;
}

BlackBoxHeader bbBuildCurrentHeader(uint8_t reason) {
  BlackBoxHeader hdr = {};
  hdr.magic          = BB_MAGIC;
  hdr.version        = BB_VERSION;
  hdr.sampleSize     = sizeof(BlackBoxSample);
  hdr.sampleCount    = bbCount;
  hdr.totalLoopTicks = bbLoopTicks;
  hdr.sawLineMs      = bbSawLineMs;
  hdr.samplePeriodMs = BB_SAMPLE_DIVIDER;
  hdr.maxDtUs        = bbMaxDtUs;
  hdr.kp             = PID_KP;
  hdr.kd             = PID_KD;
  hdr.speedBase      = SPEED_BASE;
  hdr.slewAccelLow   = SLEW_ACCEL_LOW_MS;
  hdr.slewAccelHigh  = SLEW_ACCEL_HIGH_MS;
  hdr.slewBrake      = SLEW_BRAKE_PER_MS;
  hdr.lostCount      = bbLostCount;
  hdr.gapCount       = bbGapCount;
  hdr.crossCount     = crossingCount;
  hdr.stopReason     = reason;
  return hdr;
}

bool bbSaveToFlash(uint8_t reason) {
  if (!littleFsReady || bbBuf == nullptr || bbCount == 0) return false;

  BlackBoxHeader hdr = bbBuildCurrentHeader(reason);

  File f = LittleFS.open(BB_FILE_PATH, "w");
  if (!f) return false;

  // Bat LED bao dang luu du lieu vao Flash
  digitalWrite(PIN_LED_ON, HIGH);

  size_t wHdr = f.write((const uint8_t*)&hdr, sizeof(hdr));
  const uint8_t* ptr = (const uint8_t*)bbBuf;
  size_t totalBytes  = (size_t)bbCount * sizeof(BlackBoxSample);
  size_t written     = 0;
  while (written < totalBytes) {
    size_t chunk = totalBytes - written;
    if (chunk > 4096) chunk = 4096;
    size_t w = f.write(ptr + written, chunk);
    if (w == 0) break;
    written += w;
    yield();
  }
  f.close();

  // Nhay LED 3 lan bao luu thanh cong
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_ON, LOW);  delay(80);
    digitalWrite(PIN_LED_ON, HIGH); delay(80);
  }
  digitalWrite(PIN_LED_ON, LOW);

  bbSavedFlash = (wHdr == sizeof(hdr) && written == totalBytes);
  return bbSavedFlash;
}

void bbAccumulateStats(BlackBoxStats &st, const BlackBoxSample &s) {
  st.count++;
  st.sumAppL   += s.appliedL;
  st.sumAppR   += s.appliedR;
  st.sumTgtL   += s.targetL;
  st.sumTgtR   += s.targetR;
  st.sumAbsPos += abs((int)s.linePos);
  st.sumAbsP   += abs((int)s.pTerm);
  st.sumAbsD   += abs((int)s.dTerm);

  if (s.flags & ((1 << 2) | (1 << 3))) st.slewLimitedSamples++;
  if (s.state < 7) st.stateCounts[s.state]++;

  if (s.state == STATE_FOLLOW) {
    int16_t inner = (s.appliedL < s.appliedR) ? s.appliedL : s.appliedR;
    int16_t outer = (s.appliedL > s.appliedR) ? s.appliedL : s.appliedR;
    if (inner < 306) st.innerUnderDeadband++;
    if (outer < 900) st.outerDropped++;
  }

  int16_t ad = (int16_t)abs((int)s.dTerm);
  int16_t at = (int16_t)abs((int)s.turn);
  if (ad > st.maxAbsD)     st.maxAbsD    = ad;
  if (at > st.maxAbsTurn)  st.maxAbsTurn = at;
  if (s.dtUs > st.maxDtUs) st.maxDtUs    = s.dtUs;
  st.durationMs = s.tMs;
}

void bbPrintSummary(const char* srcLabel, const BlackBoxHeader &hdr, const BlackBoxStats &st) {
  Serial.println(F("\n======================================================================"));
  Serial.printf("=== BLACKBOX SUMMARY [%s] ===\n", srcLabel);
  Serial.println(F("======================================================================"));
  Serial.printf("Cau hinh : KP=%.2f | KD=%.2f | SPEED_BASE=%d | Slew(Lo=%d, Hi=%d, Brk=%d)\n",
                hdr.kp, hdr.kd, (int)hdr.speedBase,
                (int)hdr.slewAccelLow, (int)hdr.slewAccelHigh, (int)hdr.slewBrake);
  Serial.printf("Thoi gian: %lu ms (bam line: %lu ms) | So mau: %lu (chu ky %u ms) | Max dt: %u us\n",
                (unsigned long)st.durationMs, (unsigned long)hdr.sawLineMs,
                (unsigned long)hdr.sampleCount, (unsigned int)hdr.samplePeriodMs,
                (unsigned int)hdr.maxDtUs);
  Serial.printf("Su kien  : Nga tu X=%u | GAP_BRIDGE=%u | LOST_RECOVERY=%u | Ly do dung=%u\n",
                (unsigned int)hdr.crossCount, (unsigned int)hdr.gapCount,
                (unsigned int)hdr.lostCount, (unsigned int)hdr.stopReason);

  if (st.count > 0) {
    double avgTgt    = 0.5 * (st.sumTgtL + st.sumTgtR) / st.count;
    double avgApp    = 0.5 * (st.sumAppL + st.sumAppR) / st.count;
    double pctSlew   = 100.0 * st.slewLimitedSamples / st.count;
    double followCnt = (st.stateCounts[STATE_FOLLOW] > 0) ? (double)st.stateCounts[STATE_FOLLOW] : 1.0;

    Serial.printf("PWM TB   : Target=(%.1f / %.1f, TB=%.1f) | Applied=(%.1f / %.1f, TB=%.1f)\n",
                  st.sumTgtL / st.count, st.sumTgtR / st.count, avgTgt,
                  st.sumAppL / st.count, st.sumAppR / st.count, avgApp);
    Serial.printf("Slew/Cua : Bi Slew-Rate kim=%.1f%% | Banh trong <306=%.1f%% | Banh ngoai <900=%.1f%%\n",
                  pctSlew,
                  100.0 * st.innerUnderDeadband / followCnt,
                  100.0 * st.outerDropped / followCnt);
    Serial.printf("PD Stats : TB|pos|=%.1f | TB|P|=%.1f | TB|D|=%.1f | Max|D|=%d | Max|turn|=%d\n",
                  st.sumAbsPos / st.count, st.sumAbsP / st.count, st.sumAbsD / st.count,
                  (int)st.maxAbsD, (int)st.maxAbsTurn);
    Serial.printf("States   : FOLLOW=%.1f%% | GAP=%.1f%% | LOST=%.1f%% | CROSS=%.1f%%\n",
                  100.0 * st.stateCounts[STATE_FOLLOW] / st.count,
                  100.0 * st.stateCounts[STATE_GAP_BRIDGE] / st.count,
                  100.0 * st.stateCounts[STATE_LOST_RECOVERY] / st.count,
                  100.0 * st.stateCounts[STATE_CROSS_STRAIGHT] / st.count);
  }
  Serial.println(F("======================================================================"));
}

void bbPrintCsvHeader() {
  Serial.println(F("=== BLACKBOX CSV BEGIN ==="));
  Serial.println(F("t_ms,dt_us,mv_S4,mv_S3,mv_S2,mv_S1,b_S4,b_S3,b_S2,b_S1,pos,dPos,pTerm,dTerm,turn,tgtL,tgtR,appL,appR,state,crossCnt,flags"));
}

void bbPrintCsvRow(const BlackBoxSample &s) {
  uint8_t crossCnt = (s.flags >> 4) & 0x0F;
  uint8_t lowFlags = s.flags & 0x0F; // bit0=line, bit1=crossPat, bit2=slewL, bit3=slewR
  Serial.printf("%lu,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%d,%.1f,%d,%d,%d,%d,%d,%d,%d,%s,%u,%u\n",
                (unsigned long)s.tMs,
                (unsigned int)s.dtUs,
                (unsigned int)s.mv[0], (unsigned int)s.mv[1], (unsigned int)s.mv[2], (unsigned int)s.mv[3],
                s.b[0] / 250.0f, s.b[1] / 250.0f, s.b[2] / 250.0f, s.b[3] / 250.0f,
                (int)s.linePos,
                s.dPos10 / 10.0f,
                (int)s.pTerm, (int)s.dTerm, (int)s.turn,
                (int)s.targetL, (int)s.targetR,
                (int)s.appliedL, (int)s.appliedR,
                stateName(s.state),
                (unsigned int)crossCnt,
                (unsigned int)lowFlags);
}

void bbDump(bool includeCsv, bool forceRam) {
  bool useRam = (forceRam && bbCount > 0) ||
                (!forceRam && bbCount > 0 && bbSawLineMs >= BB_MIN_VALID_LINE_MS);

  if (useRam) {
    BlackBoxHeader hdr = bbBuildCurrentHeader(bbStopReason);
    BlackBoxStats st   = {};
    for (uint32_t i = 0; i < bbCount; i++) {
      bbAccumulateStats(st, bbBuf[i]);
    }
    bbPrintSummary("RAM - LAN CHAY HIEN TAI", hdr, st);

    if (includeCsv) {
      bbPrintCsvHeader();
      for (uint32_t i = 0; i < bbCount; i++) {
        bbPrintCsvRow(bbBuf[i]);
        if ((i & 63) == 0) yield();
      }
      Serial.println(F("=== BLACKBOX CSV END ==="));
    }
    return;
  }

  // Doc tu Flash (/blackbox.bin) khi vua cam USB / reboot
  if (!littleFsReady || !LittleFS.exists(BB_FILE_PATH)) {
    Serial.println(F("[BB] Chua co du lieu BlackBox nao trong Flash!"));
    if (bbCount > 0) {
      Serial.println(F("[BB] Goi y: Dang co du lieu tam trong RAM, go 'r' de xem thu."));
    }
    return;
  }

  File f = LittleFS.open(BB_FILE_PATH, "r");
  if (!f) {
    Serial.println(F("[BB] Loi khong mo duoc file /blackbox.bin!"));
    return;
  }

  BlackBoxHeader hdr = {};
  if (f.read((uint8_t*)&hdr, sizeof(hdr)) != sizeof(hdr) ||
      hdr.magic != BB_MAGIC ||
      hdr.sampleSize != sizeof(BlackBoxSample)) {
    f.close();
    Serial.println(F("[BB] File BlackBox trong Flash bi hong hoac khac phien ban! Go 'c' de xoa."));
    return;
  }

  // Luot 1: Quet tinh bang tom tat
  BlackBoxStats st = {};
  BlackBoxSample chunkBuf[32];
  uint32_t readTotal = 0;
  while (readTotal < hdr.sampleCount) {
    uint32_t toRead = hdr.sampleCount - readTotal;
    if (toRead > 32) toRead = 32;
    size_t got = f.read((uint8_t*)chunkBuf, toRead * sizeof(BlackBoxSample));
    uint32_t n = got / sizeof(BlackBoxSample);
    if (n == 0) break;
    for (uint32_t i = 0; i < n; i++) {
      bbAccumulateStats(st, chunkBuf[i]);
    }
    readTotal += n;
    yield();
  }
  bbPrintSummary("FLASH /blackbox.bin", hdr, st);

  // Luot 2: In toan bo CSV chi tiet
  if (includeCsv) {
    f.seek(sizeof(BlackBoxHeader));
    bbPrintCsvHeader();
    uint32_t dumped = 0;
    while (dumped < hdr.sampleCount) {
      uint32_t toRead = hdr.sampleCount - dumped;
      if (toRead > 32) toRead = 32;
      size_t got = f.read((uint8_t*)chunkBuf, toRead * sizeof(BlackBoxSample));
      uint32_t n = got / sizeof(BlackBoxSample);
      if (n == 0) break;
      for (uint32_t i = 0; i < n; i++) {
        bbPrintCsvRow(chunkBuf[i]);
      }
      dumped += n;
      yield();
    }
    Serial.println(F("=== BLACKBOX CSV END ==="));
  }

  f.close();
}

void bbEraseAll() {
  bbResetRun();
  if (littleFsReady && LittleFS.exists(BB_FILE_PATH)) {
    LittleFS.remove(BB_FILE_PATH);
  }
  Serial.println(F("[BB] Da xoa sach BlackBox trong RAM va Flash!"));
}

void startNewRun(uint32_t delayMs) {
  Serial.printf("[BB] Chuan bi xuat phat sau %lu ms...\n", (unsigned long)delayMs);
  uint32_t t0 = millis();
  while (millis() - t0 < delayMs) {
    if (Serial.available()) {
      int c = Serial.peek();
      if (c == 'd' || c == 'D' || c == 's' || c == 'S' || c == 'c' || c == 'C' || c == 'r' || c == 'R') {
        return; // Huy xuat phat de xu ly lenh Serial
      }
      Serial.read();
    }
    delay(5);
  }

  bbResetRun();
  crossingCount     = 0;
  crossConfirmStart = 0;
  linePos           = 0.0f;
  prevPos1          = 0.0f;
  prevPos2          = 0.0f;
  runStartMs        = millis();
  lastCrossingMs    = runStartMs;
  lastSeenLineMs    = runStartMs;
  lastLoopUs        = micros();
  state             = STATE_FOLLOW;
}

bool handleSerialCommands() {
  bool enteredBench = false;
  while (Serial.available()) {
    char cmd = (char)Serial.read();
    if (cmd == '\r' || cmd == '\n' || cmd == ' ') continue;

    switch (cmd) {
      case 'd':
      case 'D':
        motorsCoastOff();
        state = STATE_BENCH_IDLE;
        enteredBench = true;
        bbDump(true, false);
        break;

      case 's':
      case 'S':
        motorsCoastOff();
        state = STATE_BENCH_IDLE;
        enteredBench = true;
        bbDump(false, false);
        break;

      case 'r':
      case 'R':
        motorsCoastOff();
        state = STATE_BENCH_IDLE;
        enteredBench = true;
        bbDump(true, true);
        break;

      case 'c':
      case 'C':
        motorsCoastOff();
        state = STATE_BENCH_IDLE;
        enteredBench = true;
        bbEraseAll();
        break;

      case 'g':
      case 'G':
        motorsCoastOff();
        startNewRun(3000);
        break;

      case 'h':
      case 'H':
      case '?':
        motorsCoastOff();
        state = STATE_BENCH_IDLE;
        enteredBench = true;
        Serial.println(F("\n[BB] Lenh Serial: d=In toan bo CSV | s=Tom tat | r=In RAM | c=Xoa | g=Chay lai"));
        break;

      default:
        break;
    }
  }
  return enteredBench;
}

// ============================================================================
// 7. SETUP
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

  // Luon bat Serial de cam cap nhap 'd' xem BlackBox bat cu luc nao
  Serial.begin(115200);
  delay(250);

  bbInit();

  Serial.println(F("\n================================================================"));
  Serial.printf("=== ROBOT ICTU v5.1 + BLACKBOX (KP=%.2f, KD=%.2f) ===\n", PID_KP, PID_KD);
  Serial.printf("[BB] RAM: %lu mau (%.1f giay @ %u Hz) | Flash LittleFS: %s\n",
                (unsigned long)bbCapacity,
                (bbCapacity * (float)BB_SAMPLE_DIVIDER) / 1000.0f,
                (unsigned int)(1000 / BB_SAMPLE_DIVIDER),
                littleFsReady ? "OK" : "LOI");
  if (littleFsReady && LittleFS.exists(BB_FILE_PATH)) {
    Serial.println(F("[BB] Phat hien du lieu BlackBox da luu trong Flash!"));
  }
  Serial.printf("[BB] Cho %lu ms truoc khi chay -> Nhap 'd' ngay de XEM BLACKBOX!\n",
                (unsigned long)START_DELAY_MS);
  Serial.println(F("     (d=In full CSV | s=Tom tat | c=Xoa | g=Chay lai | h=Tro giup)"));
  Serial.println(F("================================================================"));

  uint32_t waitStart = millis();
  while (millis() - waitStart < START_DELAY_MS) {
    if (handleSerialCommands()) {
      return; // Da vao STATE_BENCH_IDLE -> dung yen tren ban, khong quay banh!
    }
    delay(5);
  }

  bbResetRun();
  runStartMs     = millis();
  lastCrossingMs = runStartMs;
  lastSeenLineMs = runStartMs;
  lastLoopUs     = micros();
  state          = STATE_FOLLOW;
}

// ============================================================================
// 8. VONG LAP DIEU KHIEN CHINH (1000 Hz CO DINH + GHI BLACKBOX)
// ============================================================================
void loop() {
  // Neu dang o che do doc BlackBox tren ban -> motor tat, chi lang nghe Serial
  if (state == STATE_BENCH_IDLE) {
    handleSerialCommands();
    delay(5);
    return;
  }

  // Kiem tra nhanh lenh Serial (~1 us khi khong co du lieu -> khong tre vong lap)
  if (Serial.available()) {
    if (handleSerialCommands()) {
      return;
    }
  }

  uint32_t nowUs     = micros();
  uint32_t elapsedUs = (uint32_t)(nowUs - lastLoopUs);
  if (elapsedUs < LOOP_PERIOD_US) {
    return;
  }
  const float dtMs = (float)elapsedUs * 0.001f;
  lastLoopUs = nowUs;
  uint32_t nowMs = millis();

  uint16_t dtUsClipped = (elapsedUs > 65535UL) ? 65535 : (uint16_t)elapsedUs;
  if (dtUsClipped > bbMaxDtUs) bbMaxDtUs = dtUsClipped;

  // 1. Doc 4 cam bien [S4, S3, S2, S1]
  uint16_t mv[4];
  float b[4];
  float sumB = 0.0f;
  readSensors(mv, b, sumB);

  bool hasLine = (sumB >= LINE_PRESENT_SUM_MIN);
  if (hasLine) {
    lastSeenLineMs = nowMs;
    bbSawLineMs++;
  }

  float newPos = linePos;
  if (hasLine) {
    float weighted = b[0] * SENSOR_WEIGHTS[0]
                   + b[1] * SENSOR_WEIGHTS[1]
                   + b[2] * SENSOR_WEIGHTS[2]
                   + b[3] * SENSOR_WEIGHTS[3];
    newPos = weighted / sumB;
  }

  // 2. Phat hien Nga tu cheo X
  bool crossPattern = (b[0] >= CROSS_OUTER_TH && b[3] >= CROSS_OUTER_TH && sumB >= CROSS_SUM_MIN);
  if (state == STATE_FOLLOW &&
      (nowMs - lastCrossingMs >= CROSS_REFRACTORY_MS) &&
      (fabsf(linePos) <= CROSS_MAX_ENTRY_POS)) {
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

  // Mac dinh reset thong so PD ve 0 cho cac trang thai khong dung PD
  lastDPos  = 0.0f;
  lastPTerm = 0.0f;
  lastDTerm = 0.0f;
  lastTurn  = 0.0f;

  // 3. May trang thai dieu khien
  switch (state) {

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

    case STATE_SHORTCUT_ADVANCE: {
      setMotors(SPEED_CROSS_STRAIGHT, SPEED_CROSS_STRAIGHT);
      if (nowMs - stateStartMs >= SHORTCUT_ADVANCE_MS) {
        state        = STATE_SHORTCUT_TURN;
        stateStartMs = nowMs;
      }
      break;
    }

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

    case STATE_GAP_BRIDGE: {
      if (hasLine) {
        state = STATE_FOLLOW;
        // fall-through tinh PD ngay
      } else if (nowMs - stateStartMs < GAP_BRIDGE_MAX_MS) {
        setMotors(SPEED_BASE, SPEED_BASE);
        break;
      } else {
        recoveryDir  = (linePos >= 0.0f) ? +1 : -1;
        state        = STATE_LOST_RECOVERY;
        stateStartMs = nowMs;
        bbLostCount++;
        break;
      }
    }
    // fallthrough when hasLine == true

    case STATE_FOLLOW: {
      if (!hasLine) {
        stateStartMs = nowMs;
        if (fabsf(linePos) < GAP_HOLD_POS_TH) {
          state = STATE_GAP_BRIDGE;
          bbGapCount++;
          setMotors(SPEED_BASE, SPEED_BASE);
        } else {
          recoveryDir = (linePos > 0.0f) ? +1 : -1;
          linePos     = (recoveryDir > 0) ? +1000.0f : -1000.0f;
          prevPos1    = linePos;
          prevPos2    = linePos;
          state       = STATE_LOST_RECOVERY;
          bbLostCount++;
          if (recoveryDir > 0) setMotors(RECOVERY_OUTER_SPEED, RECOVERY_INNER_SPEED);
          else                 setMotors(RECOVERY_INNER_SPEED, RECOVERY_OUTER_SPEED);
        }
        break;
      }

      prevPos2 = prevPos1;
      prevPos1 = linePos;
      linePos  = newPos;

      float dPos  = (linePos - prevPos2) / (2.0f * ((dtMs > 0.3f) ? dtMs : 1.0f));
      float pTerm = PID_KP * linePos;
      float dTerm = PID_KD * dPos;
      float turn  = pTerm + dTerm;

      lastDPos  = dPos;
      lastPTerm = pTerm;
      lastDTerm = dTerm;
      lastTurn  = turn;

      int16_t leftSpeed  = SPEED_BASE + (int16_t)turn;
      int16_t rightSpeed = SPEED_BASE - (int16_t)turn;
      if (leftSpeed  < FOLLOW_MIN_INNER) leftSpeed  = FOLLOW_MIN_INNER;
      if (rightSpeed < FOLLOW_MIN_INNER) rightSpeed = FOLLOW_MIN_INNER;

      setMotors(leftSpeed, rightSpeed);
      break;
    }

    case STATE_LOST_RECOVERY: {
      bool caughtInner = (b[1] >= RECOVERY_INNER_TH || b[2] >= RECOVERY_INNER_TH);
      bool fallbackAny = ((nowMs - stateStartMs >= RECOVERY_FALLBACK_MS) && hasLine);

      if (caughtInner || fallbackAny) {
        state    = STATE_FOLLOW;
        linePos  = newPos;
        prevPos1 = newPos;
        prevPos2 = newPos;
      } else {
        if (recoveryDir > 0) setMotors(RECOVERY_OUTER_SPEED, RECOVERY_INNER_SPEED);
        else                 setMotors(RECOVERY_INNER_SPEED, RECOVERY_OUTER_SPEED);
      }
      break;
    }

    default:
      break;
  }

  // 4. Ghi mau BlackBox vao RAM (moi BB_SAMPLE_DIVIDER ms)
  if ((bbLoopTicks % BB_SAMPLE_DIVIDER) == 0) {
    bbRecordSample(nowMs - runStartMs, dtUsClipped, mv, b, hasLine, crossPattern);
  }
  bbLoopTicks++;

  // 5. Tu dong dung motor & luu Flash khi nhac xe khoi san > 1.5 giay
  if ((nowMs - runStartMs >= 2000) && (nowMs - lastSeenLineMs >= BB_AUTO_SAVE_NOLINE_MS)) {
    motorsCoastOff();
    bbStopReason = (bbCount >= bbCapacity) ? 2 : 1;
    state = STATE_BENCH_IDLE;

    if (bbSawLineMs >= BB_MIN_VALID_LINE_MS) {
      bool ok = bbSaveToFlash(bbStopReason);
      Serial.printf("\n[BB] Da dung xe & luu %lu mau vao Flash: %s! Nhap 'd' de in.\n",
                    (unsigned long)bbCount, ok ? "THANH CONG" : "THAT BAI");
    } else {
      Serial.println(F("\n[BB] Khong thay line du 0.5s -> Khong ghi de Flash. Nhap 'd' de xem lan chay truoc."));
    }
    return;
  }

  if (DEBUG_SERIAL && (nowMs - lastDebugMs >= 100)) {
    lastDebugMs = nowMs;
    Serial.printf("ST:%u X:%u | mV:%4u/%4u/%4u/%4u | pos:%+6.1f | L:%+4d R:%+4d\n",
                  (uint8_t)state, crossingCount,
                  mv[0], mv[1], mv[2], mv[3],
                  linePos, lastLeftCmd, lastRightCmd);
  }
}
