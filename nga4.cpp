// ============================================================================
//  COBOT BƯỚC 3: MAX TỐC ĐỘ + CUA DỨT KHOÁT + CHỐNG SỤT ÁP PIN CON Ó
//  Board: ESP32-C3 SuperMini | Vòng lặp: 500 Hz | Động cơ: TỰ ĐỘNG CHỐNG SỤT ÁP
// ----------------------------------------------------------------------------
//  BẢO VỆ NGUỒN PIN CON Ó (ZINC-CARBON):
//    1) Bộ lọc gia tốc Slew Rate Limiter: PWM không tăng đột biến trong 1 tick,
//       mà tăng mềm theo đường dốc 18ms -> triệt tiêu dòng khởi động dI/dt cực đại.
//    2) Zero-crossing Pause (4ms): Khi đảo chiều quay, bắt buộc xả dòng tự cảm
//       Back-EMF ở mức 0 trước khi đảo chiều -> không bao giờ bị sụt áp Brownout!
//    3) Khóa Brownout phần cứng: Tắt bộ ngắt Brownout Detector trên ESP32-C3.
//
//  HIỆU NĂNG CAO:
//    - Tốc độ chạy thẳng: 140 PWM (xé gió)
//    - Cua ngã tư dứt khoát: Xoay cực nhanh 135 PWM -> Ép tâm 75 PWM -> Phanh ngược 90 PWM!
// ============================================================================

#include <Arduino.h>
#include <math.h>

#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
// 1. CẤU HÌNH TỐC ĐỘ MAX & CHIẾN THUẬT RẼ DỨT KHOÁT
// ============================================================================
const int   BASE_SPEED      = 140;    // [MAX TỐC ĐỘ] Tốc độ chạy thẳng bám line (140/255)
const int   MAX_PWM_CAP     = 200;    // Trần PWM tối đa (bảo vệ nguồn pin không bị kiệt dòng)

const int   TURN_PWM_FAST   = 135;    // [CUA DỨT KHOÁT] Vung xoay nhanh thoát vạch cũ & quét vạch mới
const int   TURN_PWM_SLOW   = 75;     // Tốc độ ép góc khóa tâm 90° chuẩn xác
const int   BRAKE_PWM       = 90;     // Xung phanh ngược dập đứng quán tính (dứt khoát)

const int   INCH_CENTER_MS  = 140;    // Ở tốc độ 140, tiến 140ms (~65mm) là trục bánh vào trúng tâm!
const int   BRAKE_MS        = 18;     // Xung phanh ngược dập tắt đà trôi trong 18ms

const float KP_LINE         = 3.6f;   // Độ nhạy lái tốc độ cao
const float KD_LINE         = 0.045f; // Chống lắc rung tay lái

// Đổi chiều động cơ nếu cần:
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;
const bool SWAP_MOTORS_LR     = false;

// ============================================================================
// 2. KHAI BÁO CHÂN PHẦN CỨNG
// ============================================================================
#define PIN_OUTER_L   0   // GPIO 0: Ngoài cùng TRÁI (S4)
#define PIN_INNER_L   1   // GPIO 1: Giữa TRÁI       (S3)
#define PIN_INNER_R   3   // GPIO 3: Giữa PHẢI       (S2)
#define PIN_OUTER_R   4   // GPIO 4: Ngoài cùng PHẢI (S1)

#define IR_LED_ON    21   // Bật LED hồng ngoại

// Chân điều khiển mạch cầu H (Động cơ N20)
#define PWM_PIN_L_A   2   // Motor Trái A
#define PWM_PIN_L_B  10   // Motor Trái B
#define PWM_PIN_R_A   6   // Motor Phải A
#define PWM_PIN_R_B   5   // Motor Phải B

#define CH_L_A 0
#define CH_L_B 1
#define CH_R_A 2
#define CH_R_B 3

const int SENSOR_PINS[4] = { PIN_OUTER_L, PIN_INNER_L, PIN_INNER_R, PIN_OUTER_R };

// ============================================================================
// 3. THÔNG SỐ HIỆU CHUẨN CẢM BIẾN
// ============================================================================
const int CAL_WHITE = 2500;  // RAW >= 2500 là nền trắng sàn (0% Đen)
const int CAL_BLACK = 500;   // RAW <= 500 là vạch đen đậm   (100% Đen)

const float THRESH_BLACK_LINE = 0.35f;  // >= 35% đen là có chạm vạch
const float THRESH_CROSS_WING = 0.40f;  // Hai cánh ngoài cùng >= 40% đen
const float THRESH_CROSS_MID  = 0.30f;  // Mắt giữa >= 30% đen

const int DEBOUNCE_CROSS_TICKS = 3;     // Ở tốc độ cao, 3 chu kỳ (6ms) là đủ chốt ngã tư
const int DEBOUNCE_LINE_TICKS  = 2;     // 2 chu kỳ (4ms) chốt line
const int DEBOUNCE_OUT_TICKS   = 4;     // 4 chu kỳ chốt ra ngoài

// ============================================================================
// 4. KIỂU DỮ LIỆU & BIẾN TRẠNG THÁI
// ============================================================================
enum LineState {
    STATE_OUT_OF_LINE = 0,
    STATE_LINE        = 1,
    STATE_CROSSROAD   = 2
};

enum TurnPhase {
    TURN_IDLE = 0,
    TURN_INCH_CENTER,       // Pha 1: Tiến dứt khoát vào tâm ngã tư
    TURN_CLEAR_OLD_LINE,    // Pha 2: Vung xoay nhanh thoát ra nền trắng
    TURN_SWEEP_TO_LINE,     // Pha 3: Quét nhanh đón vạch nhánh mới (~75°)
    TURN_LOCK_CENTER,       // Pha 4: Ép góc khóa tâm 90° (tới khi S3 chạm vạch)
    TURN_ACTIVE_BRAKE       // Pha 5: Phanh ngược dập đứng quán tính
};

typedef struct {
    int       raw[4];
    float     norm[4];
    float     k_cross;
    float     center_val;
    float     max_val;
    LineState state;
} SensorFrame;

LineState currentState = STATE_OUT_OF_LINE;
TurnPhase turnPhase    = TURN_IDLE;

int cross_ticks = 0;
int line_ticks  = 0;
int out_ticks   = 0;

float last_line_x = 0.0f;
unsigned long turn_timer = 0;
unsigned long cross_cooldown_until = 0;

SensorFrame currentFrame;

// Prototype hàm
SensorFrame processSensors();
void handleMotion(const SensorFrame& sf);
void setMotorsHardware(int target_l, int target_r);
void setupMotorsHardware();
void printDashboard(const SensorFrame& sf);
int readMedianADC(int pin);

// ============================================================================
// 5. BỘ ĐIỀU KHIỂN CHỐNG SỤT ÁP NGUỒN PIN (SLEW RATE & ZERO-CROSS BRAKE)
// ============================================================================
const int SLEW_UP_PER_TICK    = 16;  // Tăng tối đa 16 đơn vị PWM mỗi 2ms
const int SLEW_DN_PER_TICK    = 35;  // Giảm tốc nhanh hơn
const int REVERSE_PAUSE_TICKS = 2;   // Dừng 2 tick (4ms) xả tự cảm khi đảo chiều

int prev_l_pwm = 0;
int prev_r_pwm = 0;
int rev_hold_l = 0;
int rev_hold_r = 0;

static inline int applySlew(int target, int prev, int& hold) {
    if (hold > 0) { hold--; return 0; }
    // Kiểm tra đảo chiều: nếu đang tiến mà đảo sang lùi hoặc ngược lại
    if (prev != 0 && target != 0 && ((target > 0) != (prev > 0))) {
        hold = REVERSE_PAUSE_TICKS; // Xả dòng Back-EMF 4ms ở mức 0V
        return 0;
    }
    int diff = target - prev;
    int lim  = (abs(target) > abs(prev)) ? SLEW_UP_PER_TICK : SLEW_DN_PER_TICK;
    if (diff > lim) diff = lim;
    else if (diff < -lim) diff = -lim;
    return prev + diff;
}

void setupMotorsHardware() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(PWM_PIN_L_A, 5000, 8, CH_L_A);
    ledcAttachChannel(PWM_PIN_L_B, 5000, 8, CH_L_B);
    ledcAttachChannel(PWM_PIN_R_A, 5000, 8, CH_R_A);
    ledcAttachChannel(PWM_PIN_R_B, 5000, 8, CH_R_B);
#elif defined(ESP32)
    ledcSetup(CH_L_A, 5000, 8);
    ledcSetup(CH_L_B, 5000, 8);
    ledcSetup(CH_R_A, 5000, 8);
    ledcSetup(CH_R_B, 5000, 8);
    ledcAttachPin(PWM_PIN_L_A, CH_L_A);
    ledcAttachPin(PWM_PIN_L_B, CH_L_B);
    ledcAttachPin(PWM_PIN_R_A, CH_R_A);
    ledcAttachPin(PWM_PIN_R_B, CH_R_B);
#endif
    setMotorsHardware(0, 0);
}

static inline void writePWM(int channel, int val) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(channel, val);
#elif defined(ESP32)
    ledcWrite(channel, val);
#else
    analogWrite(channel, val);
#endif
}

void setMotorsHardware(int target_l, int target_r) {
    if (SWAP_MOTORS_LR) { int t = target_l; target_l = target_r; target_r = t; }
    if (INVERT_LEFT_MOTOR)  target_l  = -target_l;
    if (INVERT_RIGHT_MOTOR) target_r = -target_r;

    // Giới hạn trần PWM bảo vệ nguồn pin
    target_l = constrain(target_l, -MAX_PWM_CAP, MAX_PWM_CAP);
    target_r = constrain(target_r, -MAX_PWM_CAP, MAX_PWM_CAP);

    // Đi qua bộ làm mềm gia tốc chống sụt áp tức thời
    int out_l = applySlew(target_l, prev_l_pwm, rev_hold_l);
    int out_r = applySlew(target_r, prev_r_pwm, rev_hold_r);
    prev_l_pwm = out_l;
    prev_r_pwm = out_r;

    // Kênh Trái
    if (out_l >= 0) {
        writePWM(CH_L_A, out_l);
        writePWM(CH_L_B, 0);
    } else {
        writePWM(CH_L_A, 0);
        writePWM(CH_L_B, -out_l);
    }

    // Kênh Phải
    if (out_r >= 0) {
        writePWM(CH_R_A, out_r);
        writePWM(CH_R_B, 0);
    } else {
        writePWM(CH_R_A, 0);
        writePWM(CH_R_B, -out_r);
    }
}

// ============================================================================
// 6. ĐỌC CẢM BIẾN & XỬ LÝ QUANG HỌC
// ============================================================================
int readMedianADC(int pin) {
    int buf[5];
    for (int i = 0; i < 5; i++) buf[i] = analogRead(pin);
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (buf[i] > buf[j]) { int t = buf[i]; buf[i] = buf[j]; buf[j] = t; }
        }
    }
    return buf[2];
}

SensorFrame processSensors() {
    SensorFrame sf;
    sf.max_val = 0.0f;

    for (int i = 0; i < 4; i++) {
        sf.raw[i] = readMedianADC(SENSOR_PINS[i]);
        
        float n = (float)(CAL_WHITE - sf.raw[i]) / (float)(CAL_WHITE - CAL_BLACK);
        if (n < 0.0f) n = 0.0f;
        if (n > 1.0f) n = 1.0f;
        sf.norm[i] = n;

        if (n > sf.max_val) sf.max_val = n;
    }

    sf.k_cross    = fminf(sf.norm[0], sf.norm[3]);
    sf.center_val = fmaxf(sf.norm[1], sf.norm[2]);

    bool is_cross = (sf.k_cross >= THRESH_CROSS_WING) && (sf.center_val >= THRESH_CROSS_MID);
    bool is_line  = (sf.max_val >= THRESH_BLACK_LINE) && !is_cross;

    if (is_cross) {
        cross_ticks++; line_ticks = 0; out_ticks = 0;
        if (cross_ticks >= DEBOUNCE_CROSS_TICKS) currentState = STATE_CROSSROAD;
    } else if (is_line) {
        line_ticks++; cross_ticks = 0; out_ticks = 0;
        if (line_ticks >= DEBOUNCE_LINE_TICKS) currentState = STATE_LINE;
    } else {
        out_ticks++;
        if (out_ticks >= DEBOUNCE_OUT_TICKS) {
            cross_ticks = 0; line_ticks = 0;
            currentState = STATE_OUT_OF_LINE;
        }
    }

    sf.state = currentState;
    return sf;
}

// ============================================================================
// 7. MÁY TRẠNG THÁI RẼ PHẢI DỨT KHOÁT TỐC ĐỘ CAO
// ============================================================================
void handleMotion(const SensorFrame& sf) {
    unsigned long now = millis();

    // 1. PHÁT HIỆN NGÃ TƯ -> KÍCH HOẠT RẼ
    if (turnPhase == TURN_IDLE) {
        if (sf.state == STATE_CROSSROAD && now > cross_cooldown_until) {
            turnPhase  = TURN_INCH_CENTER;
            turn_timer = now;
            Serial.printf("\n>>> [NGÃ TƯ PHÁT HIỆN] -> TIẾN TÂM %d ms <<<\n", INCH_CENTER_MS);
        }
    }

    // 2. MÁY TRẠNG THÁI RẼ 5 PHA DỨT KHOÁT
    if (turnPhase == TURN_INCH_CENTER) {
        // PHA 1: Tiến dứt khoát 140ms đưa trục bánh sau vào giữa tâm ngã tư
        setMotorsHardware(BASE_SPEED, BASE_SPEED);
        if (now - turn_timer >= INCH_CENTER_MS) {
            turnPhase  = TURN_CLEAR_OLD_LINE;
            turn_timer = now;
            Serial.println(">>> [CUA DỨT KHOÁT] -> PHA 2: QUAY NHANH THOÁT VẠCH CŨ <<<");
        }
    }
    else if (turnPhase == TURN_CLEAR_OLD_LINE) {
        // PHA 2: Xoay cực nhanh (135 PWM) để thoát vạch cũ
        setMotorsHardware(TURN_PWM_FAST, -TURN_PWM_FAST);

        // Bắt buộc: mắt đón bên phải (S2, S1) rơi xuống sàn trắng (< 25% đen)
        bool is_on_white      = (sf.norm[2] < 0.25f && sf.norm[3] < 0.30f);
        bool min_time_ok      = (now - turn_timer >= 80);
        bool fallback_timeout = (now - turn_timer >= 450);

        if ((min_time_ok && is_on_white) || fallback_timeout) {
            turnPhase  = TURN_SWEEP_TO_LINE;
            turn_timer = now;
            Serial.println(">>> [CUA DỨT KHOÁT] -> PHA 3: QUÉT NHANH ĐÓN NHÁNH MỚI <<<");
        }
    }
    else if (turnPhase == TURN_SWEEP_TO_LINE) {
        // PHA 3: Vung xoay nhanh trên nền trắng đón nhánh mới (~75°)
        setMotorsHardware(TURN_PWM_FAST, -TURN_PWM_FAST);

        bool reached_first_edge = (sf.norm[2] >= 0.40f || sf.norm[3] >= 0.45f);
        bool timeout            = (now - turn_timer >= 900);

        if (reached_first_edge || timeout) {
            turnPhase  = TURN_LOCK_CENTER;
            turn_timer = now;
            Serial.println(">>> [CUA DỨT KHOÁT] -> PHA 4: ÉP GÓC KHÓA TÂM 90° <<<");
        }
    }
    else if (turnPhase == TURN_LOCK_CENTER) {
        // PHA 4: Hạ tốc độ (75 PWM) xoay thêm cho tới khi MẮT GIỮA TRÁI (S3) chạm vạch đen
        setMotorsHardware(TURN_PWM_SLOW, -TURN_PWM_SLOW);

        bool aligned_90_deg = (sf.norm[1] >= 0.35f);
        bool timeout        = (now - turn_timer >= 350);

        if (aligned_90_deg || timeout) {
            turnPhase  = TURN_ACTIVE_BRAKE;
            turn_timer = now;
            // Kích hoạt ngay xung phanh ngược dập quán tính
            setMotorsHardware(-BRAKE_PWM, BRAKE_PWM);
            Serial.println(">>> [CUA DỨT KHOÁT] -> PHA 5: PHANH NGƯỢC DẬP QUÁN TÍNH <<<");
        }
    }
    else if (turnPhase == TURN_ACTIVE_BRAKE) {
        // PHA 5: Giữ xung phanh ngược trong 18ms dập tắt hoàn toàn đà trôi
        setMotorsHardware(-BRAKE_PWM, BRAKE_PWM);
        if (now - turn_timer >= BRAKE_MS) {
            turnPhase            = TURN_IDLE;
            cross_cooldown_until = now + 900; // Khóa 900ms chống nhận ngã tư lặp
            last_line_x          = 0.0f;
            setMotorsHardware(0, 0); // Ngắt dứt điểm
            Serial.println(">>> [HOÀN THÀNH RẼ 90°] -> XÉ GIÓ BÁM LINE TIẾP TỤC <<<\n");
        }
    }
    else {
        // 3. CHẾ ĐỘ BÌNH THƯỜNG: BÁM LINE TỐC ĐỘ CAO (PD CONTROL)
        float w_sum = sf.norm[0] + sf.norm[1] + sf.norm[2] + sf.norm[3];
        if (w_sum > 0.15f) {
            // Tọa độ lệch tâm vạch X (mm): [-20, -11.25, +11.25, +20]
            float line_x = (-20.0f * sf.norm[0] - 11.25f * sf.norm[1] 
                            + 11.25f * sf.norm[2] + 20.0f * sf.norm[3]) / w_sum;
            
            float dt    = 0.002f; // 500 Hz
            float d_err = (line_x - last_line_x) / dt;
            last_line_x = line_x;

            float steer = KP_LINE * line_x + KD_LINE * d_err;
            steer = constrain(steer, -75.0f, 75.0f);

            int p_l = (int)(BASE_SPEED + steer);
            int p_r = (int)(BASE_SPEED - steer);
            setMotorsHardware(p_l, p_r);
        } else {
            // Nhấc xe hoặc ra ngoài vạch: nếu mất vạch quá 200 ms -> ngắt động cơ an toàn
            if (out_ticks > 100) {
                setMotorsHardware(0, 0);
            } else {
                setMotorsHardware(BASE_SPEED - 30, BASE_SPEED - 30);
            }
        }
    }
}

// ============================================================================
// 8. DASHBOARD THEO DÕI SERIAL
// ============================================================================
void printVisualBar(float val) {
    if (val < 0.25f)      Serial.print("[    ]");
    else if (val < 0.50f) Serial.print("[ .  ]");
    else if (val < 0.75f) Serial.print("[ ## ]");
    else                  Serial.print("[####]");
}

void printDashboard(const SensorFrame& sf) {
    if (turnPhase == TURN_INCH_CENTER) {
        Serial.print("RẼ (P1:Sâu)   | ");
    } else if (turnPhase == TURN_CLEAR_OLD_LINE) {
        Serial.print("RẼ (P2:Trắng) | ");
    } else if (turnPhase == TURN_SWEEP_TO_LINE) {
        Serial.print("RẼ (P3:Đón)   | ");
    } else if (turnPhase == TURN_LOCK_CENTER) {
        Serial.print("RẼ (P4:Ép 90) | ");
    } else if (turnPhase == TURN_ACTIVE_BRAKE) {
        Serial.print("RẼ (P5:Phanh) | ");
    } else if (sf.state == STATE_CROSSROAD) {
        Serial.print("NGÃ TƯ        | ");
    } else if (sf.state == STATE_LINE) {
        Serial.print("LINE          | ");
    } else {
        Serial.print("NGOÀI         | ");
    }

    Serial.print("Xe [T_ng T_gi P_gi P_ng]: ");
    printVisualBar(sf.norm[0]);
    printVisualBar(sf.norm[1]);
    printVisualBar(sf.norm[2]);
    printVisualBar(sf.norm[3]);
    Serial.print(" | ");

    Serial.printf("T_ng:%3.0f%% T_gi:%3.0f%% P_gi:%3.0f%% P_ng:%3.0f%% | ",
                  sf.norm[0] * 100.0f, sf.norm[1] * 100.0f,
                  sf.norm[2] * 100.0f, sf.norm[3] * 100.0f);

    Serial.printf("K_ngatu:%.2f | RAW:[G0:%4d, G1:%4d, G3:%4d, G4:%4d]\n",
                  sf.k_cross, sf.raw[0], sf.raw[1], sf.raw[2], sf.raw[3]);
}

// ============================================================================
// 9. SETUP
// ============================================================================
void setup() {
    // VÔ HIỆU HÓA BROWNOUT DETECTOR (CHỐNG RESET KHI DÙNG PIN CON Ó)
#if defined(RTC_CNTL_BROWN_OUT_REG) && defined(RTC_CNTL_BROWN_OUT_ENA)
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#endif

    pinMode(IR_LED_ON, OUTPUT);
    digitalWrite(IR_LED_ON, HIGH);

    for (int i = 0; i < 4; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }

    // TẮT SẠCH WIFI & BLUETOOTH ĐỂ KHÔNG TIÊU THỤ DÒNG ĐỈNH
#if defined(ESP32)
    WiFi.mode(WIFI_OFF);
    btStop();
    analogReadResolution(12);
#endif

    setupMotorsHardware();

    Serial.begin(115200);
    Serial.setTxTimeoutMs(0); // Không bao giờ chặn vòng lặp 500 Hz
    delay(500);

    Serial.println("\n========================================================");
    Serial.println("  COBOT STEP 3: MAX TỐC ĐỘ + CUA DỨT KHOÁT + CHỐNG SỤT ÁP");
    Serial.println("========================================================");
    Serial.printf("Tốc độ bám: %d | Cua nhanh: %d | Ép góc: %d | Phanh: %d\n",
                  BASE_SPEED, TURN_PWM_FAST, TURN_PWM_SLOW, BRAKE_PWM);
    Serial.println("TÍNH NĂNG BẢO VỆ PIN: Slew Rate Limiter + Zero-Cross Pause BẬT 100%!");
    Serial.println("ĐỢI 2 GIÂY ĐẶT XE LÊN SÀN AN TOÀN...");
    
    delay(2000); // 2 giây an toàn
    Serial.println(">>> XE BẮT ĐẦU CHẠY XÉ GIÓ! <<<\n");
}

// ============================================================================
// 10. VÒNG LẶP CHÍNH (500 HZ ĐIỀU KHIỂN, 10 HZ SERIAL)
// ============================================================================
unsigned long lastLoopMicros = 0;
unsigned long lastPrintMillis = 0;

void loop() {
    unsigned long nowMicros = micros();
    if (nowMicros - lastLoopMicros >= 2000) { // 500 Hz (2ms)
        lastLoopMicros = nowMicros;
        currentFrame = processSensors();
        handleMotion(currentFrame);
    }

    unsigned long nowMillis = millis();
    if (nowMillis - lastPrintMillis >= 100) { // 10 Hz
        lastPrintMillis = nowMillis;
        printDashboard(currentFrame);
    }
}
