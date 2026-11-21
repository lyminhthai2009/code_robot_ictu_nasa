// ============================================================================
// COBOT LINE FOLLOWER VIP v7.3 - DUAL-ZONE GRIP + MAX SPEED + ZERO-SAG
// Target  : ESP32-C3 (KM-11-102 + KMM-11-100), USB CDC On Boot: Enabled
// Contest : Duong toi Tan Cuong Thai Nguyen 2026 (Vong Bang & Vong Chung Ket)
// ============================================================================
#include <Arduino.h>
#include <math.h>
#if defined(ESP32)
#include "driver/gpio.h"
#endif

// ==================== CAU HINH CHE DO ====================
#define ENABLE_SHORTCUT          1   // 1 = re tat phai 90 do tai Vong tron 2
#define ENABLE_FLOOR_TRACK       1   // 1 = tu thich ung nen mau / anh sang san dau
#define ENABLE_BOOT_CHIRP        1   // 1 = keu "tit" nhe bao boot sach
#define ENABLE_BLACKBOX          1   // Dat 0 ngay thi dau chinh thuc
#define ENABLE_SERIAL_TELEMETRY  0

// ==================== PHAN CUNG (CHUAN XE BAN) ====================
#define PWM_PIN_L_A   2
#define PWM_PIN_L_B   10
#define PWM_PIN_R_A   6
#define PWM_PIN_R_B   5
#define CH_L_A        0
#define CH_L_B        1
#define CH_R_A        2
#define CH_R_B        3

#define SENSOR_1_PIN  0
#define SENSOR_2_PIN  1
#define SENSOR_3_PIN  3
#define SENSOR_4_PIN  4
#define W_LED_ON      20
#define IR_LED_ON     21

static const int   PIN_IR[4]          = { SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN };
static const int   MIN_N20_PWM        = 34;     // Bu vung chet GA12-N20
static const float RIGHT_MOTOR_TRIM   = 1.00f;  // Chinh 0.96..1.04 neu lech co khi
static const bool  INVERT_LEFT_MOTOR  = false;
static const bool  INVERT_RIGHT_MOTOR = false;
static const bool  SWAP_MOTORS_LR     = false;

#ifndef IR_WHITE_IS_HIGH
#define IR_WHITE_IS_HIGH 1
#endif
#if IR_WHITE_IS_HIGH
static const int IR_POL        = -1;
static const int ADC_WHITE_NOM = 2500;
static const int ADC_BLACK_NOM = 500;
#else
static const int IR_POL        = +1;
static const int ADC_WHITE_NOM = 300;
static const int ADC_BLACK_NOM = 2400;
#endif

// ==================== HINH HOC THUC TE CUA XE BAN ====================
static const float TRACK_WIDTH_MM     = 78.5f;  // 85mm - 6.5mm
static const float SENSOR_X_MM[4]     = { -20.0f, -11.25f, 11.25f, 20.0f };
static const float SENSOR_Y_MM[4]     = {  60.0f,  64.84f, 64.84f, 60.0f };
static const float SENSOR_VORONOI[4]  = {  0.72f,   1.25f,  1.25f,  0.72f };
static const float DEFAULT_LINE_W_MM  = 24.0f;
static const float SPEED_MMPS_AT_255  = 535.0f;

// ==================== TOC DO & BAM CUA 2 VUNG (CUA THUONG FULL / CUA GAP BAM CUNG) ====================
static const float V_MAX              = 255.0f; // Duong thang & cua nhe FULL 255!
static const float V_CORNER_MIN       = 250.0f; // Cua thuong van ghim sat 250-255!
static const float V_BRAKE_MIN        = 192.0f; // Cua rat gap tu ha banh ngoai ve 192 de khong bay line!
static const float V_START            = 135.0f;
static const float SOFT_START_S       = 0.11f;
static const float ACC_UP             = 3200.0f;
static const float ACC_DN_SOFT        = 1500.0f;
static const float ACC_DN             = 3800.0f;
static const float K_SPD_E            = 0.40f;
static const float E_SPD_DEAD_MM      = 12.5f;
static const float K_SPD_R            = 60.0f;

// Banh trong: cua thuong giu >=50 (nhanh + muot), cua gap ha thang ve 0 (ban kinh R=39mm, 0 plugging!)
static const float F_IN_MIN           = 50.0f;
static const float F_IN_EMERG         = 0.0f;   // Khong bao gio dao chieu khi con tren line -> 0 sut ap!
static const float F_IN_LOST          = -26.0f; // Chi lui nhe khi mat han line de keo ve tuc thi

// ==================== DIEU KHIEN LAI 2 VUNG (EM O TAM - BAM CUNG O MEP CUA) ====================
static const float KP                 = 3.15f;  // Em o duong thang & cua thuong
static const float KP_NL_MM           = 19.5f;  // Tang vot luc keo khi |x| > 11mm (cua gap!)
static const float KP_NL_QUAD         = 0.32f;  // Thanh phan bac 2 bat chat mep cua gap
static const float KD                 = 0.28f;
static const float KD_NL              = 0.24f;
static const float KD_NL_VX           = 300.0f;

// Smart Curvature Feedforward (don cua muot + xa sieu nhanh luc thoat cua S)
static const float KI_CURVE           = 18.0f;
static const float I_MAX              = 58.0f;
static const float I_LEAK_TAU         = 0.42f;
static const float I_LEAK_DYN         = 2.5f;
static const float I_VX_REF           = 230.0f;
static const float I_LEAK_WRONG       = 12.0f;
static const float I_DEAD_MM          = 2.5f;
static const float I_ACTIVE_MM        = 20.0f;

// Loc Alpha-Beta + do vi tri 2D
static const float AB_ALPHA           = 0.36f;
static const float AB_BETA            = 0.042f;
static const float JUMP_MM            = 26.0f;
static const float W_FLOOR            = 0.045f;
static const float W_PEAK_REL         = 0.10f;
static const float WSUM_MIN           = 0.035f;
static const float K_EXT_MM           = 10.2f;
static const float X_LIM_MM           = 30.0f;

// Du bao nguy co NHAY GAP DOI khi vao cua gap (bat som 3mm + phan xa 8ms!)
static const float T_PRED             = 0.036f;
static const float RISK_X0            = 10.5f;  // Bat dau mo quyen cua gap ngay tu 10.5mm!
static const float RISK_X1            = 20.5f;
static const float T_PRED_V           = 0.038f;
static const float RISK_V_X0          = 13.0f;
static const float RISK_V_X1          = 22.0f;
static const float RISK_TAU_UP        = 0.008f; // Phan xa cuc nhanh 8ms -> khong kip bay khoi cua!
static const float RISK_TAU_DN        = 0.060f;
static const float U_SLEW_CALM        = 7200.0f;  // Em o cua thuong, khong bi tre pha!
static const float U_SLEW_FAST        = 24000.0f; // Phan xa gan nhu tuc thi khi cua gap!

// ==================== MAT VACH / GO GIAM TOC / DOC CAU ====================
static const float GAP_X_MM           = 11.5f;
static const float GAP_HOLD_MM        = 80.0f;
static const float GAP_REACQ_MAX      = 0.14f;
static const float GAP_REACQ_MIN      = 0.34f;
static const float V_GAP_MAX          = 250.0f;
static const float V_LOST             = 175.0f;
static const float U_LOST_MAX         = 148.0f;
static const float LOST_RAMP_S        = 0.045f; // Keo ve line nhanh gap doi neu lo tuot mep!
static const float LOST_GIVEUP_S      = 1.25f;
static const float LOST_PRED_T        = 0.026f;
static const float REACQ_VX           = 125.0f;
static const float REACQ_CALM_S       = 0.040f;

// ==================== KHOI DONG "BAT CONG TAC -> THA XUONG" ====================
static const float ARM_SETTLE_S       = 0.065f;
static const float ARM_REARM_AIR_S    = 0.095f;
static const int   ARM_STABLE_DELTA   = 230;
static const int   START_SPAN_MIN     = 340;

// ==================== NGA TU & BAN DO DIA DANH (VONG BANG + CHUNG KET) ====================
static const float WING_ON             = 0.52f;
static const float WING_OFF            = 0.34f;
static const float WING_BASE_MAX       = 0.45f;
static const float WING_DELTA_MIN      = 0.25f;
static const int   WING_DEBOUNCE_TICKS = 2;
static const int   CROSS_CONFIRM_TICKS = 2;
static const int   CROSS_CONFIRM_SKEW  = 3;
static const float WING_MIN_CHORD_MM   = 4.2f;
static const float WING_LATCH_MM       = 42.0f;
static const float WING_MAX_SKEW_MM    = 36.0f;
static const float CROSS_PASS_MAX_MM   = 170.0f;
static const float CROSS_PASS_MAX_S    = 0.60f;
static const float CROSS_EXIT_MARGIN   = 10.0f;
static const float U_REF_TAU           = 0.15f;

static const float MAP_START_TO_C1_MM  = 220.0f;
static const float MAP_LOOP1_MIN_YAW   = 2.80f;
static const float MAP_LOOP1_MIN_MM    = 300.0f;
static const float MAP_UTURN_MIN_YAW   = 2.05f;
static const float MAP_UTURN_MIN_MM    = 320.0f;
static const float ARM_AFTER_U_DIST_MM = 65.0f;  // Sau nay neu co go toi mau: tang len 250..320

// ==================== RE TAT "TURBO ACTIVE-BRAKE SNAP" TAI VONG TRON 2 ====================
static const float P_ARC_OUTER         = 255.0f;
static const float P_ARC_INNER_SNAP    = -118.0f;
static const float ARC_BRAKE_MIN_S     = 0.024f;
static const float ARC_BRAKE_MAX_S     = 0.036f;
static const float P_APPROACH_OUTER    = 215.0f;
static const float PIVOT_OLD_ON        = 0.45f;
static const float PIVOT_OLD_GONE      = 0.28f;
static const float PIVOT_MIN_T         = 0.095f;
static const float PIVOT_MIN_YAW       = 0.72f;
static const float PIVOT_RELAX_T       = 0.16f;
static const float T_PIVOT_MAX         = 0.85f;
static const float PIVOT_SWEEP_MMPS    = 280.0f;
static const float HANDOVER_T          = 0.13f;
static const float V_HANDOVER          = 230.0f;
static const float F_IN_HANDOVER       = 10.0f;
static const float U_MAX_HANDOVER      = 125.0f;

// ==================== THICH UNG NEN & BO QUAN LY DONG THUC CHONG SUT AP ====================
static const float CM_DEAD             = 0.05f;
static const float CM_MAX              = 0.72f;
static const float CM_TAU_UP           = 0.035f;
static const float CM_TAU_DN           = 0.026f;
static const float CM_ABSORB_MIN       = 0.05f;
static const float CM_ABSORB_RATE      = 0.04f;
static const float PEAK_TARGET         = 0.88f;
static const float PEAK_MIN            = 0.28f;
static const float PEAK_TAU_UP         = 0.018f;
static const float PEAK_TAU_DN         = 0.090f;

static const int   ADC_RAW_LO_OK        = 20;
static const int   ADC_RAW_HI_OK        = 3850;
static const int   ADC_FRAME_HI_OK      = 3950;
static const int   LINE_FRAME_SPAN_MIN  = 140;
static const int   CM_SPAN_MIN          = 235;
static const int   MIN_CONTRAST_SPAN    = 350;
static const int   INIT_CONTRAST_SPAN   = 850;
static const int   EFF_SPAN_MIN         = 315;
static const int   WHITE_SEEN_MAX_DARK  = 2000;
static const int   WHITE_DRIFT_MAX_DARK = 1200;
static const int   WHITE_CAL_MAX_DARK   = 1600;

static const float BEMF_K               = 0.78f;
static const float BEMF_TAU_UP_S        = 0.058f;
static const float BEMF_TAU_DN_S        = 0.044f;
static const float BEMF_TAU_BRAKE_S     = 0.015f;
static const float TRUE_I_SINGLE_MAX    = 146.0f;
static const float TRUE_I_TOTAL_BUDGET  = 198.0f; // Giu 4xAA chong sut ap ra cua, duong thang van full 255!
static const int   REVERSE_PAUSE_TICKS  = 1;

// ==================== TIEN ICH TOAN HOC ====================
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }
static inline int   roundi(float v) { return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)); }
static inline int   absi(int v) { return (v < 0) ? -v : v; }
static inline int   darker(int a, int ref)   { return IR_POL * (a - ref); }
static inline int   darkerOf(int a, int b)   { return (darker(a, b) >= 0) ? a : b; }
static inline int   brighterOf(int a, int b) { return (darker(a, b) <= 0) ? a : b; }

// ==================== KHOA CHAN MOTOR SIEU SOM (DIET LOI QUAY NGUOC LUC BOOT) ====================
static inline void forceMotorPinsLowNow() {
#if defined(ESP32)
    const gpio_num_t mp[4] = {
        (gpio_num_t)PWM_PIN_L_A, (gpio_num_t)PWM_PIN_L_B,
        (gpio_num_t)PWM_PIN_R_A, (gpio_num_t)PWM_PIN_R_B
    };
    for (int i = 0; i < 4; i++) {
        gpio_pullup_dis(mp[i]);
        gpio_pulldown_en(mp[i]);
        gpio_set_direction(mp[i], GPIO_MODE_OUTPUT);
        gpio_set_level(mp[i], 0);
    }
#endif
}

static void __attribute__((constructor)) earlyBootMotorKill() {
    forceMotorPinsLowNow();
}

static void bootChirpPreLEDC() {
#if ENABLE_BOOT_CHIRP
    for (int i = 0; i < 50; i++) {
        digitalWrite(PWM_PIN_L_A, HIGH);
        delayMicroseconds(38);
        digitalWrite(PWM_PIN_L_A, LOW);
        delayMicroseconds(332);
    }
#endif
}

// ==================== PHAN CUNG LEDC 5000 Hz CHUAN ====================
static void setupMotorsHardware() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(PWM_PIN_L_A, 5000, 8, CH_L_A);
    ledcAttachChannel(PWM_PIN_L_B, 5000, 8, CH_L_B);
    ledcAttachChannel(PWM_PIN_R_A, 5000, 8, CH_R_A);
    ledcAttachChannel(PWM_PIN_R_B, 5000, 8, CH_R_B);
#elif defined(ESP32)
    ledcSetup(CH_L_A, 5000, 8); ledcSetup(CH_L_B, 5000, 8);
    ledcSetup(CH_R_A, 5000, 8); ledcSetup(CH_R_B, 5000, 8);
    ledcAttachPin(PWM_PIN_L_A, CH_L_A); ledcAttachPin(PWM_PIN_L_B, CH_L_B);
    ledcAttachPin(PWM_PIN_R_A, CH_R_A); ledcAttachPin(PWM_PIN_R_B, CH_R_B);
#endif
}

static inline void writeLEDC(int channel, int pwm) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(channel, pwm);
#elif defined(ESP32)
    ledcWrite(channel, pwm);
#else
    analogWrite(channel, pwm);
#endif
}

// ==================== BO QUAN LY DONG THUC + ACTIVE SHORT-BRAKE ====================
static int   prev_log_pwm_l = 0, prev_log_pwm_r = 0;
static int   zero_cross_hold_l = 0, zero_cross_hold_r = 0;
static float bemf_l = 0.0f, bemf_r = 0.0f;

static inline int applyN20Deadzone(int pwm) {
    if (pwm == 0) return 0;
    int mag = absi(pwm);
    if (mag >= 250) return (pwm > 0) ? 255 : -255;
    int out;
    if (mag < 15) {
        int knee = MIN_N20_PWM + ((15 * (255 - MIN_N20_PWM)) / 255);
        out = (mag * knee) / 15;
    } else {
        out = constrain(MIN_N20_PWM + ((mag * (255 - MIN_N20_PWM)) / 255), 0, 255);
    }
    return (pwm > 0) ? out : -out;
}

static inline int limitSingleTrueCurrent(int target, int prev, float bemf, int& hold) {
    if (hold > 0) { hold--; return 0; }
    if (prev != 0 && target != 0 && ((target > 0) != (prev > 0))) {
        hold = REVERSE_PAUSE_TICKS;
        return 0;
    }
    if (target != 0 && ((target > 0 && bemf < -42.0f) || (target < 0 && bemf > 42.0f))) return 0;
    float e_eff = BEMF_K * bemf;
    if (target > 0 && (float)target > e_eff + TRUE_I_SINGLE_MAX)
        target = roundi(e_eff + TRUE_I_SINGLE_MAX);
    else if (target < 0 && (float)target < e_eff - TRUE_I_SINGLE_MAX)
        target = roundi(e_eff - TRUE_I_SINGLE_MAX);
    return constrain(target, -255, 255);
}

static inline float estTrueCurrent(int out, float bemf, bool brk) {
    if (brk || out == 0) return 0.0f;
    float e_eff = BEMF_K * bemf;
    return (out > 0) ? max2f(0.0f, (float)out - e_eff) : max2f(0.0f, (float)(-out) + e_eff);
}

static void resetMotorState() {
    prev_log_pwm_l = prev_log_pwm_r = 0;
    zero_cross_hold_l = zero_cross_hold_r = 0;
    bemf_l = bemf_r = 0.0f;
    writeLEDC(CH_L_A, 0); writeLEDC(CH_L_B, 0);
    writeLEDC(CH_R_A, 0); writeLEDC(CH_R_B, 0);
}

static void setMotorHardware(int pwm_l, int pwm_r, bool brake_l, bool brake_r, float dt) {
    int out_l = 0, out_r = 0;

    if (brake_l) { zero_cross_hold_l = REVERSE_PAUSE_TICKS; out_l = 0; }
    else         { out_l = limitSingleTrueCurrent(applyN20Deadzone(pwm_l), prev_log_pwm_l, bemf_l, zero_cross_hold_l); }

    if (brake_r) { zero_cross_hold_r = REVERSE_PAUSE_TICKS; out_r = 0; }
    else         { out_r = limitSingleTrueCurrent(applyN20Deadzone(roundi((float)pwm_r * RIGHT_MOTOR_TRIM)), prev_log_pwm_r, bemf_r, zero_cross_hold_r); }

    float i_l = estTrueCurrent(out_l, bemf_l, brake_l);
    float i_r = estTrueCurrent(out_r, bemf_r, brake_r);
    if (i_l + i_r > TRUE_I_TOTAL_BUDGET) {
        float excess = (i_l + i_r) - TRUE_I_TOTAL_BUDGET;
        if (i_r > i_l + 15.0f) {
            int cut_r = min(absi(out_r), roundi(excess));
            if (out_r > 0) out_r -= cut_r; else if (out_r < 0) out_r += cut_r;
            excess -= (float)cut_r;
            if (excess > 0.5f) {
                int cut_l = min(absi(out_l), roundi(excess));
                if (out_l > 0) out_l -= cut_l; else if (out_l < 0) out_l += cut_l;
            }
        } else if (i_l > i_r + 15.0f) {
            int cut_l = min(absi(out_l), roundi(excess));
            if (out_l > 0) out_l -= cut_l; else if (out_l < 0) out_l += cut_l;
            excess -= (float)cut_l;
            if (excess > 0.5f) {
                int cut_r = min(absi(out_r), roundi(excess));
                if (out_r > 0) out_r -= cut_r; else if (out_r < 0) out_r += cut_r;
            }
        } else {
            int trim = roundi(0.5f * excess);
            if (out_l > 0) out_l = max(0, out_l - trim); else if (out_l < 0) out_l = min(0, out_l + trim);
            if (out_r > 0) out_r = max(0, out_r - trim); else if (out_r < 0) out_r = min(0, out_r + trim);
        }
    }

    prev_log_pwm_l = out_l;
    prev_log_pwm_r = out_r;

    float tau_l = brake_l ? BEMF_TAU_BRAKE_S : ((fabsf((float)out_l) > fabsf(bemf_l)) ? BEMF_TAU_UP_S : BEMF_TAU_DN_S);
    float tau_r = brake_r ? BEMF_TAU_BRAKE_S : ((fabsf((float)out_r) > fabsf(bemf_r)) ? BEMF_TAU_UP_S : BEMF_TAU_DN_S);
    bemf_l += ((float)out_l - bemf_l) * (dt / (tau_l + dt));
    bemf_r += ((float)out_r - bemf_r) * (dt / (tau_r + dt));

    int hw_l = out_l, hw_r = out_r;
    bool hw_brk_l = brake_l, hw_brk_r = brake_r;
    if (SWAP_MOTORS_LR) {
        int tmp = hw_l; hw_l = hw_r; hw_r = tmp;
        bool tb = hw_brk_l; hw_brk_l = hw_brk_r; hw_brk_r = tb;
    }
    if (INVERT_LEFT_MOTOR)  hw_l = -hw_l;
    if (INVERT_RIGHT_MOTOR) hw_r = -hw_r;

    if (hw_brk_l)      { writeLEDC(CH_L_A, 255);  writeLEDC(CH_L_B, 255); }
    else if (hw_l >= 0){ writeLEDC(CH_L_A, hw_l); writeLEDC(CH_L_B, 0);   }
    else               { writeLEDC(CH_L_A, 0);    writeLEDC(CH_L_B, -hw_l); }

    if (hw_brk_r)      { writeLEDC(CH_R_A, 255);  writeLEDC(CH_R_B, 255); }
    else if (hw_r >= 0){ writeLEDC(CH_R_A, hw_r); writeLEDC(CH_R_B, 0);   }
    else               { writeLEDC(CH_R_A, 0);    writeLEDC(CH_R_B, -hw_r); }
}

// ==================== CAU TRUC DU LIEU & MIXER ====================
struct SensorData {
    float ir[4];
    int   raw_ir[4];
    int   frame_span;
    int   frame_bright;
    bool  in_air;
    float elapsed_sec;
};

static void mixWheels(float v, float u, float f_in, float& vl, float& vr) {
    float a_max = 0.5f * (255.0f - f_in);
    float a = fabsf(u);
    if (a > a_max) a = a_max;
    float ve = clampf(v, f_in + a, 255.0f - a);
    if (u >= 0.0f) { vl = ve + a; vr = ve - a; }
    else           { vl = ve - a; vr = ve + a; }
}

// ==================== BO NAO DIEU KHIEN VIP v7.3 ====================
class Brain {
public:
    enum Mode : uint8_t {
        M_ARMED = 0,
        M_RUN,
        M_CROSS_PASS,
        M_TURN_ARC
    };
    enum MapState : uint8_t {
        MAP_TO_LOOP1_IN = 0,
        MAP_IN_LOOP1,
        MAP_TO_U_TURN,
        MAP_ARM_SHORTCUT,
        MAP_AFTER_SHORTCUT
    };

    Mode     mode;
    MapState map_state;
    float    mode_t, mode_d, launch_t;

    float x, vx, x_slow, L_eff;
    bool  need_reinit;
    float integ, u_cmd;
    float risk_f, risk_v_f, risk_dbg;

    float v_cmd, v_est, w_est, dist_mm, v_scale;
    bool  v_cal_valid;
    float line_w_est;
    float seg_dist, seg_yaw, seg_yaw_max, u_done_dist, turn_yaw;

    int   last_side, lost_side;
    bool  in_loss, meas_ok, is_gap_loss, on_crossbar;
    float lost_t, lost_d, lost_x;
    float handover_t, reacq_calm_t, reacq_vx;
    float air_t, all_black_t, off_track_t;

    float armed_stable_t, rearm_hold_t;
    int   prev_raw[4], arm_bright[4], arm_dark[4];
    bool  prev_raw_valid, cal_just_locked;

    bool  base_init;
    float base_o[2];
    bool  w_on[2], w_rose[2], w_fell[2], w_sep;
    int   w_ticks[2];
    float w_cand_d[2], w_rise_d[2], w_fall_d[2], w_half_fall_d[2], w_last_d[2];
    float cross_sec, center_odo, end_odo;
    bool  center_refined;
    float u_ref, u_hold;
    int   cross_confirm, crossbar_ticks, early_sc_ticks, cross_count, clear_ticks, white_ticks;
    float last_cross_dist;

    int   turn_dir;
    bool  pivot_old_seen, pivot_has_z, pivot_left_crossbar, pivot_old_in_clear;
    float pivot_prev_z, pivot_vx_est;

    Brain() { resetToArmed(); }

    void resetToArmed() {
        mode = M_ARMED;
        map_state = MAP_TO_LOOP1_IN;
        mode_t = mode_d = launch_t = 0.0f;
        x = vx = x_slow = 0.0f; L_eff = 63.0f; need_reinit = true;
        integ = u_cmd = 0.0f;
        risk_f = risk_v_f = risk_dbg = 0.0f;
        v_cmd = 0.0f; v_est = w_est = dist_mm = 0.0f; v_scale = 1.0f; v_cal_valid = false;
        line_w_est = DEFAULT_LINE_W_MM;
        seg_dist = seg_yaw = seg_yaw_max = u_done_dist = turn_yaw = 0.0f;
        last_side = lost_side = 1;
        in_loss = meas_ok = is_gap_loss = on_crossbar = false;
        lost_t = lost_d = lost_x = 0.0f;
        handover_t = reacq_calm_t = reacq_vx = 0.0f;
        air_t = all_black_t = off_track_t = 0.0f;
        armed_stable_t = 0.0f; rearm_hold_t = 0.14f;
        prev_raw_valid = cal_just_locked = false;
        base_init = false; base_o[0] = base_o[1] = 0.0f;
        cross_confirm = crossbar_ticks = early_sc_ticks = cross_count = clear_ticks = white_ticks = 0;
        last_cross_dist = -1.0e6f;
        cross_sec = 1.0f; center_odo = end_odo = 0.0f; center_refined = false;
        u_ref = u_hold = 0.0f;
        turn_dir = 1;
        pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
        pivot_prev_z = pivot_vx_est = 0.0f;
        clearWings();
    }

    static bool isCrossbar(const float* s) {
        return (min4f(s[0], s[1], s[2], s[3]) >= 0.44f) ||
               (s[0] >= 0.50f && s[3] >= 0.50f && max2f(s[1], s[2]) >= 0.40f);
    }

    static bool hasRealLine(const float* s, int frame_span) {
        if (frame_span < LINE_FRAME_SPAN_MIN) return false;
        float s_max = max4f(s[0], s[1], s[2], s[3]);
        float s_min = min4f(s[0], s[1], s[2], s[3]);
        if (s_min > 0.85f) return false;
        float in_max = max2f(s[1], s[2]), out_max = max2f(s[0], s[3]), out_min = min2f(s[0], s[3]);
        float min_contrast = (s_min > 0.30f) ? 0.20f : 0.14f;
        if (s_max >= 0.18f && ((s_max - s_min) >= min_contrast || (s_max >= 0.72f && in_max > out_max + 0.04f))) return true;
        return (in_max >= 0.055f && out_min < 0.06f && (in_max - out_max) >= 0.04f);
    }

    void step(const SensorData& sd, float dt, float hw_bemf_l, float hw_bemf_r,
              int& pl, int& pr, bool& brk_l, bool& brk_r) {
        pl = 0; pr = 0; brk_l = false; brk_r = false;
        cal_just_locked = false;
        if (!(dt > 0.0005f && dt < 0.02f)) dt = 0.002f;
        const float* s = sd.ir;

        // 1. CHE DO CHO THA XUONG (M_ARMED)
        if (mode == M_ARMED) {
            if (rearm_hold_t > 0.0f) rearm_hold_t = max2f(0.0f, rearm_hold_t - dt);

            int max_delta = 0;
            if (prev_raw_valid) {
                for (int i = 0; i < 4; i++) max_delta = max(max_delta, absi(sd.raw_ir[i] - prev_raw[i]));
            }
            for (int i = 0; i < 4; i++) prev_raw[i] = sd.raw_ir[i];
            prev_raw_valid = true;

            bool bright_ok   = (IR_POL < 0) ? (sd.frame_bright > ADC_RAW_LO_OK + 350) : (sd.frame_bright < ADC_RAW_HI_OK - 350);
            bool stable_line = (!sd.in_air) && (rearm_hold_t <= 0.0f) && bright_ok &&
                               (sd.frame_span >= START_SPAN_MIN) && (max_delta <= ARM_STABLE_DELTA);

            if (stable_line) {
                if (armed_stable_t <= 0.0001f) {
                    for (int i = 0; i < 4; i++) arm_bright[i] = arm_dark[i] = sd.raw_ir[i];
                } else {
                    for (int i = 0; i < 4; i++) {
                        arm_bright[i] = brighterOf(arm_bright[i], sd.raw_ir[i]);
                        arm_dark[i]   = darkerOf(arm_dark[i], sd.raw_ir[i]);
                    }
                }
                armed_stable_t += dt;
                if (armed_stable_t >= ARM_SETTLE_S) {
                    cal_just_locked = true;
                    mode = M_RUN;
                    map_state = MAP_TO_LOOP1_IN;
                    mode_t = mode_d = launch_t = 0.0f;
                    x = vx = x_slow = 0.0f; L_eff = 63.0f; need_reinit = true;
                    integ = u_cmd = u_ref = u_hold = 0.0f;
                    risk_f = risk_v_f = risk_dbg = 0.0f;
                    v_cmd = V_START;
                    dist_mm = seg_dist = seg_yaw = seg_yaw_max = u_done_dist = 0.0f;
                    in_loss = is_gap_loss = on_crossbar = false;
                    lost_t = lost_d = air_t = all_black_t = off_track_t = 0.0f;
                    cross_count = early_sc_ticks = 0;
                    last_cross_dist = -1.0e6f;
                    clearWings();
                }
            } else {
                armed_stable_t = max2f(0.0f, armed_stable_t - 2.5f * dt);
            }
            return;
        }

        // 2. KIEM TRA AN TOAN
        air_t = sd.in_air ? (air_t + dt) : 0.0f;
        float min_ir   = min4f(s[0], s[1], s[2], s[3]);
        bool real_line = hasRealLine(s, sd.frame_span);
        bool crossbar  = isCrossbar(s);
        on_crossbar    = crossbar;

        bool all_black = (min_ir > 0.87f) && (sd.frame_span < 400 || min_ir > 0.94f);
        bool no_line   = !all_black && !real_line && !crossbar;
        bool spinning  = (mode == M_TURN_ARC);
        bool in_cross  = (mode == M_CROSS_PASS);

        float all_blk_lim = (map_state == MAP_AFTER_SHORTCUT) ? 0.44f : 0.28f;

        if (air_t > ARM_REARM_AIR_S) { resetToArmed(); rearm_hold_t = 0.28f; return; }
        if (all_black && !spinning && !in_cross) {
            all_black_t += dt;
            if (all_black_t > all_blk_lim) { resetToArmed(); rearm_hold_t = 0.30f; return; }
        } else all_black_t = 0.0f;
        if (no_line && !spinning && !in_cross) {
            off_track_t += dt;
            if (off_track_t > LOST_GIVEUP_S) { resetToArmed(); rearm_hold_t = 0.30f; return; }
        } else off_track_t = 0.0f;

        // 3. ODOMETRY
        launch_t += dt;
        updateOdometry(dt, hw_bemf_l, hw_bemf_r);

        // 4. UOC LUONG & BAN DO
        if (mode == M_RUN) runEstimate(sd, dt);

        // 5. XUAT LENH 2 BANH + ACTIVE BRAKE
        float vl = 0.0f, vr = 0.0f;
        switch (mode) {
            case M_RUN:        outRun(dt, vl, vr);                        break;
            case M_CROSS_PASS: outCrossPass(s, dt, vl, vr);               break;
            case M_TURN_ARC:   outTurnArc(sd, dt, vl, vr, brk_l, brk_r);  break;
            default:           vl = vr = 0.0f;                            break;
        }
        pl = roundi(clampf(vl, -255.0f, 255.0f));
        pr = roundi(clampf(vr, -255.0f, 255.0f));
    }

private:
    void rampSpeed(float target, float dt, float dn = ACC_DN) {
        if (target > v_cmd) { v_cmd += ACC_UP * dt; if (v_cmd > target) v_cmd = target; }
        else                { v_cmd -= dn * dt;     if (v_cmd < target) v_cmd = target; }
    }

    void updateOdometry(float dt, float hw_bemf_l, float hw_bemf_r) {
        float vl_mm = v_scale * SPEED_MMPS_AT_255 * (hw_bemf_l / 255.0f);
        float vr_mm = v_scale * SPEED_MMPS_AT_255 * (hw_bemf_r / 255.0f);
        v_est = 0.5f * (vl_mm + vr_mm);
        w_est = (vl_mm - vr_mm) / TRACK_WIDTH_MM;

        float ds = max2f(0.0f, v_est) * dt;
        dist_mm  += ds;
        seg_dist += ds;
        seg_yaw  += w_est * dt;
        if (seg_yaw > seg_yaw_max) seg_yaw_max = seg_yaw;
    }

    bool measure(const SensorData& sd, float& z, float& L_out) {
        const float* s = sd.ir;
        if (!hasRealLine(s, sd.frame_span)) return false;
        float s_max = max4f(s[0], s[1], s[2], s[3]);
        float w_thr = max2f(W_FLOOR, W_PEAK_REL * s_max);
        float inv   = 1.0f / max2f(1.0f - w_thr, 0.20f);
        float w_raw[4], w_vor[4], wsum_vor = 0.0f, wsum_raw = 0.0f;
        for (int i = 0; i < 4; i++) {
            float c = (s[i] - w_thr) * inv;
            w_raw[i] = (c > 0.0f) ? c : 0.0f;
            w_vor[i] = w_raw[i] * SENSOR_VORONOI[i];
            wsum_raw += w_raw[i];
            wsum_vor += w_vor[i];
        }
        if (wsum_vor < WSUM_MIN) return false;
        float num_x = 0.0f, num_y = 0.0f;
        for (int i = 0; i < 4; i++) {
            num_x += w_vor[i] * SENSOR_X_MM[i];
            num_y += w_raw[i] * SENSOR_Y_MM[i];
        }
        float xc = num_x / wsum_vor;
        L_out = clampf(num_y / max2f(wsum_raw, 1e-4f), 60.0f, 64.84f);

        if (s[3] >= s[0]) {
            float b = clampf(1.0f - (w_raw[0] + w_raw[1] + w_raw[2]) / 0.32f, 0.0f, 1.0f);
            xc += b * K_EXT_MM * (1.0f - min2f(s[3], 1.0f));
        } else {
            float b = clampf(1.0f - (w_raw[1] + w_raw[2] + w_raw[3]) / 0.32f, 0.0f, 1.0f);
            xc -= b * K_EXT_MM * (1.0f - min2f(s[0], 1.0f));
        }
        z = clampf(xc, -X_LIM_MM, X_LIM_MM);

        if (fabsf(z) < 5.0f && s[0] < 0.25f && s[3] < 0.25f) {
            float w_est_m = clampf(14.0f + 7.5f * (s[1] + s[2]) + 4.0f * (s[0] + s[3]), 16.0f, 30.0f);
            line_w_est += 0.008f * (w_est_m - line_w_est);
        }
        return true;
    }

    void updateWingBase(const float* s, float dt) {
        if (!base_init) { base_o[0] = s[0]; base_o[1] = s[3]; base_init = true; }
        const float ab = dt / (0.15f + dt);
        base_o[0] += ((s[0] < 0.45f) ? ab : ab * 0.125f) * (s[0] - base_o[0]);
        base_o[1] += ((s[3] < 0.45f) ? ab : ab * 0.125f) * (s[3] - base_o[1]);
    }

    void clearWings() {
        for (int k = 0; k < 2; k++) {
            w_on[k] = w_rose[k] = w_fell[k] = false; w_ticks[k] = 0;
            w_cand_d[k] = w_rise_d[k] = w_fall_d[k] = w_half_fall_d[k] = w_last_d[k] = dist_mm;
        }
        w_sep = false; cross_confirm = crossbar_ticks = early_sc_ticks = 0;
    }

    void trackWings(const float* s, float dt) {
        const int idx[2] = { 0, 3 };
        if (mode == M_RUN) {
            if (w_fell[0] && !w_rose[1] && x > 4.0f)  { w_rose[0] = w_fell[0] = false; w_ticks[0] = 0; }
            if (w_fell[1] && !w_rose[0] && x < -4.0f) { w_rose[1] = w_fell[1] = false; w_ticks[1] = 0; }
        }
        for (int k = 0; k < 2; k++) {
            float v = s[idx[k]];
            if (w_on[k]) {
                w_last_d[k] = dist_mm;
                if (v >= WING_ON) w_half_fall_d[k] = dist_mm;
                if (v < WING_OFF) {
                    w_on[k] = false;
                    if (mode == M_RUN && (w_half_fall_d[k] - w_rise_d[k]) < WING_MIN_CHORD_MM && !w_rose[1 - k]) {
                        w_rose[k] = w_fell[k] = false; w_ticks[k] = 0;
                    } else { w_fell[k] = true; w_fall_d[k] = dist_mm; }
                }
            } else if (!w_rose[k]) {
                bool opp_active  = (s[idx[1 - k]] >= 0.34f) || (w_ticks[1 - k] > 0) || w_rose[1 - k];
                bool corner_mask = (mode == M_RUN) && !opp_active &&
                                   ((k == 0 && x < -7.5f && s[2] < 0.20f) || (k == 1 && x > 7.5f && s[1] < 0.20f));
                if (!corner_mask && v >= WING_ON && base_o[k] < WING_BASE_MAX && (v - base_o[k]) >= WING_DELTA_MIN) {
                    if (w_ticks[k] == 0) w_cand_d[k] = dist_mm;
                    if (++w_ticks[k] >= WING_DEBOUNCE_TICKS) {
                        w_on[k] = w_rose[k] = true; w_fell[k] = false;
                        w_rise_d[k] = w_cand_d[k]; w_half_fall_d[k] = w_last_d[k] = dist_mm;
                    }
                } else w_ticks[k] = 0;
            } else if (mode == M_RUN && (((dist_mm - w_last_d[k]) > WING_LATCH_MM) ||
                       (w_fell[k] && !w_rose[1 - k] && !w_sep && (dist_mm - w_fall_d[k]) > 14.0f))) {
                w_rose[k] = w_fell[k] = false; w_ticks[k] = 0;
            }
        }
        if (w_rose[0] || w_rose[1] || w_ticks[0] > 0 || w_ticks[1] > 0) {
            bool dip_sep    = (s[0] >= WING_ON && s[1] < s[0] - 0.05f && s[2] > 0.20f) ||
                              (s[3] >= WING_ON && s[2] < s[3] - 0.05f && s[1] > 0.20f);
            bool both_outer = (s[0] >= WING_ON && s[3] >= WING_ON && (s[1] > 0.28f || s[2] > 0.28f));
            bool edge_skew  = (s[0] > 0.72f && s[0] >= s[1] - 0.18f && s[2] > 0.32f) ||
                              (s[3] > 0.72f && s[3] >= s[2] - 0.18f && s[1] > 0.32f);
            if (dip_sep || both_outer || edge_skew) w_sep = true;
        } else w_sep = false;
        updateWingBase(s, dt);
    }

    bool isCrossAllowed() const {
        if (handover_t > 0.05f) return false;
        if (in_loss && !on_crossbar) return false;
        if ((dist_mm - last_cross_dist) < 180.0f) return false;
        switch (map_state) {
            case MAP_TO_LOOP1_IN:  return (seg_dist >= MAP_START_TO_C1_MM);
            case MAP_IN_LOOP1:     return (fabsf(seg_yaw) >= MAP_LOOP1_MIN_YAW) && (seg_dist >= MAP_LOOP1_MIN_MM);
            case MAP_TO_U_TURN:    return false;
            case MAP_ARM_SHORTCUT: return ((dist_mm - u_done_dist) >= ARM_AFTER_U_DIST_MM);
            default:               return false;
        }
    }

    bool detectCross(const float* s, bool allowed) {
        if (allowed && map_state == MAP_ARM_SHORTCUT) {
            bool early_hit = ((s[0] >= 0.34f && s[3] >= 0.38f && max2f(s[1], s[2]) >= 0.32f) ||
                              (min4f(s[0], s[1], s[2], s[3]) >= 0.36f)) &&
                             (fabsf(x) < 15.0f);
            early_sc_ticks = early_hit ? (early_sc_ticks + 1) : 0;
            if (early_sc_ticks >= 2) return true;
        } else {
            early_sc_ticks = 0;
        }

        crossbar_ticks = (allowed && isCrossbar(s)) ? (crossbar_ticks + 1) : 0;
        if (crossbar_ticks >= CROSS_CONFIRM_TICKS) return true;

        bool skew_ok     = fabsf(w_rise_d[1] - w_rise_d[0]) <= WING_MAX_SKEW_MM;
        bool center      = (s[1] > 0.32f || s[2] > 0.32f || (s[0] >= WING_ON && s[3] >= WING_ON));
        bool both_strong = (s[0] >= WING_ON && s[3] >= WING_ON && (s[1] >= 0.38f || s[2] >= 0.38f));
        float c0 = (w_on[0] ? dist_mm : w_half_fall_d[0]) - w_rise_d[0];
        float c1 = (w_on[1] ? dist_mm : w_half_fall_d[1]) - w_rise_d[1];
        bool both_chord  = (max2f(c0, c1) >= WING_MIN_CHORD_MM) && (min2f(c0, c1) >= 3.2f) && (w_on[0] || w_on[1]);
        bool trig = allowed && w_rose[0] && w_rose[1] && w_sep && center && skew_ok && (both_strong || both_chord);
        cross_confirm = trig ? (cross_confirm + 1) : 0;
        return cross_confirm >= (both_strong ? CROSS_CONFIRM_TICKS : CROSS_CONFIRM_SKEW);
    }

    void planCross() {
        float t = min2f(fabsf(w_rise_d[1] - w_rise_d[0]) / (SENSOR_X_MM[3] - SENSOR_X_MM[0]), 3.0f);
        cross_sec = sqrtf(1.0f + t * t);
        float thick = line_w_est * cross_sec;
        center_odo = 0.5f * (w_rise_d[0] + w_rise_d[1]) + 0.5f * thick;
        end_odo    = max2f(w_rise_d[0], w_rise_d[1]) + thick + CROSS_EXIT_MARGIN;
        center_refined = false;
    }

    void refineCenter() {
        if (center_refined || !(w_fell[0] && w_fell[1])) return;
        center_refined = true;
        float L0 = 0.5f * (w_half_fall_d[0] + w_fall_d[0]) - w_rise_d[0];
        float L1 = 0.5f * (w_half_fall_d[1] + w_fall_d[1]) - w_rise_d[1];
        float L_min_ok = 0.45f * line_w_est, L_max_ok = 3.20f * line_w_est * cross_sec;
        if (!(L0 > L_min_ok && L1 > L_min_ok && L0 < L_max_ok && L1 < L_max_ok && fabsf(L0 - L1) < 0.45f * max2f(L0, L1))) return;
        float L_meas = (fabsf(L0 - L1) > 0.20f * max2f(L0, L1)) ? min2f(L0, L1) : (0.5f * (L0 + L1));
        float skew_term = (line_w_est * fabsf(w_rise_d[1] - w_rise_d[0])) / (SENSOR_X_MM[3] - SENSOR_X_MM[0]);
        float r = clampf(line_w_est / sqrtf(max2f(L_meas * L_meas - skew_term * skew_term, 0.25f * L_meas * L_meas)), 0.72f, 1.32f);
        float v_old = v_scale;
        v_scale = clampf(v_old + (!v_cal_valid ? 0.60f : 0.30f) * (clampf(v_old * r, 0.72f, 1.32f) - v_old), 0.72f, 1.32f);
        v_cal_valid = true;
    }

    void enterCross() {
        cross_count++;
        for (int k = 0; k < 2; k++) {
            if (!w_rose[k]) {
                w_on[k] = w_rose[k] = true; w_fell[k] = false;
                w_rise_d[k] = (w_ticks[k] > 0) ? w_cand_d[k] : dist_mm;
                w_half_fall_d[k] = w_last_d[k] = dist_mm;
            }
        }
        last_cross_dist = dist_mm;
        u_hold = clampf(u_ref, -22.0f, 22.0f);
        clear_ticks = white_ticks = cross_confirm = crossbar_ticks = early_sc_ticks = 0;
        mode_t = mode_d = 0.0f;
        planCross();

        if (map_state == MAP_TO_LOOP1_IN) {
            map_state = MAP_IN_LOOP1;
            seg_dist = seg_yaw = seg_yaw_max = 0.0f;
            mode = M_CROSS_PASS;
        } else if (map_state == MAP_IN_LOOP1) {
            map_state = MAP_TO_U_TURN;
            seg_dist = seg_yaw = seg_yaw_max = 0.0f;
            mode = M_CROSS_PASS;
        } else if (map_state == MAP_ARM_SHORTCUT) {
            map_state = MAP_AFTER_SHORTCUT;
            seg_dist = seg_yaw = seg_yaw_max = 0.0f;
#if ENABLE_SHORTCUT
            turn_dir = +1;
            mode = M_TURN_ARC;
            turn_yaw = 0.0f;
            pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
            pivot_prev_z = pivot_vx_est = 0.0f;
            clearWings();
#else
            mode = M_CROSS_PASS;
#endif
        } else {
            mode = M_CROSS_PASS;
        }
    }

    void runEstimate(const SensorData& sd, float dt) {
        const float* s = sd.ir;
        bool crossbar = isCrossbar(s);

        if (map_state == MAP_TO_U_TURN) {
            float yaw_swing_left = seg_yaw_max - seg_yaw;
            if (yaw_swing_left >= MAP_UTURN_MIN_YAW && seg_dist >= MAP_UTURN_MIN_MM && fabsf(u_cmd) < 55.0f) {
                map_state = MAP_ARM_SHORTCUT;
                u_done_dist = dist_mm;
            }
        }

        bool allowed = isCrossAllowed();
        if (allowed) trackWings(s, dt);
        else { clearWings(); updateWingBase(s, dt); }
        bool crossing = detectCross(s, allowed);

        if (reacq_calm_t > 0.0f) reacq_calm_t = max2f(0.0f, reacq_calm_t - dt);

        if (crossbar) {
            in_loss = is_gap_loss = meas_ok = false; lost_t = lost_d = 0.0f;
            x *= 0.92f; vx *= 0.85f;
        } else {
            float z = 0.0f, L_m = L_eff;
            bool ok = measure(sd, z, L_m);
            if (ok && in_loss && is_gap_loss && lost_t < GAP_REACQ_MAX && max4f(s[0], s[1], s[2], s[3]) < GAP_REACQ_MIN)
                ok = false;
            meas_ok = ok;
            if (meas_ok) {
                L_eff += 0.20f * (L_m - L_eff);
                if (in_loss) {
                    bool was_gap = is_gap_loss;
                    in_loss = is_gap_loss = false;
                    if (lost_t > 0.012f) {
                        need_reinit = true;
                        reacq_vx = (!was_gap && fabsf(lost_x) >= GAP_X_MM) ? -(float)lost_side * REACQ_VX : 0.0f;
                        if (was_gap) reacq_calm_t = REACQ_CALM_S;
                    }
                    if (lost_t > 0.12f) integ *= 0.25f;
                    lost_t = lost_d = 0.0f;
                }
                if (!need_reinit) {
                    float xp = x + vx * dt, r = z - xp;
                    if (fabsf(r) > JUMP_MM) need_reinit = true;
                    else {
                        float beta_eff = (reacq_calm_t > 0.0f) ? (AB_BETA * 0.30f) : AB_BETA;
                        x  = xp + AB_ALPHA * r;
                        vx = clampf(vx + (beta_eff / dt) * r, -1500.0f, 1500.0f);
                    }
                }
                if (need_reinit) { x = x_slow = z; vx = reacq_vx; reacq_vx = 0.0f; need_reinit = false; }
                x_slow += (x - x_slow) * (dt / (0.028f + dt));
                if (fabsf(x) > 2.5f) last_side = (x > 0.0f) ? 1 : -1;
            } else {
                if (!in_loss) {
                    in_loss = true; lost_t = lost_d = 0.0f;
                    is_gap_loss = (fabsf(x_slow) < GAP_X_MM);
                    lost_x = is_gap_loss ? x_slow : clampf(x + LOST_PRED_T * vx, -X_LIM_MM, X_LIM_MM);
                    lost_side = (fabsf(lost_x) > 1.5f) ? ((lost_x > 0.0f) ? 1 : -1) : last_side;
                }
                lost_t += dt; lost_d += max2f(0.0f, v_est) * dt;
                x = clampf(x + vx * dt, -X_LIM_MM, X_LIM_MM);
                vx *= 0.94f;
            }
        }
        if (crossing) enterCross();
    }

    void outRun(float dt, float& vl, float& vr) {
        float f_in = F_IN_MIN, a_cap = 0.5f * (255.0f - f_in);
        bool handover = false;
        if (handover_t > 0.0f) {
            handover = true; handover_t = max2f(0.0f, handover_t - dt);
            f_in = F_IN_HANDOVER; a_cap = min2f(U_MAX_HANDOVER, 0.5f * (255.0f - f_in));
        }

        float u = 0.0f, v_t = V_MAX, dn_rate = ACC_DN;

        if (on_crossbar && !in_loss) {
            u = clampf(u_ref * 0.45f, -22.0f, 22.0f);
            risk_f   -= risk_f   * (dt / (RISK_TAU_DN + dt));
            risk_v_f -= risk_v_f * (dt / (RISK_TAU_DN + dt));
        } else if (!in_loss) {
            float ae = fabsf(x), s_dir = (x >= 0.0f) ? 1.0f : -1.0f;
            float v_out = (ae > 3.5f) ? max2f(0.0f, s_dir * vx) : 0.0f;
            float risk   = clampf((ae + T_PRED   * v_out - RISK_X0)   / (RISK_X1   - RISK_X0),   0.0f, 1.0f);
            float risk_v = clampf((ae + T_PRED_V * v_out - RISK_V_X0) / (RISK_V_X1 - RISK_V_X0), 0.0f, 1.0f);

            risk_f   += (risk   - risk_f)   * (dt / (((risk   > risk_f)   ? RISK_TAU_UP : RISK_TAU_DN) + dt));
            risk_v_f += (risk_v - risk_v_f) * (dt / (((risk_v > risk_v_f) ? RISK_TAU_UP : RISK_TAU_DN) + dt));

            if (!handover) { f_in = F_IN_MIN + (F_IN_EMERG - F_IN_MIN) * risk_f; a_cap = 0.5f * (255.0f - f_in); }

            // Do loi 2 vung: em o tam duong thang, tang vot luc keo khi dat ra mep cua gap!
            float L_gain = 63.0f / L_eff;
            float norm_e = ae / KP_NL_MM;
            float kp = KP * L_gain * (1.0f + norm_e + KP_NL_QUAD * norm_e * norm_e);
            float kd = KD * (1.0f + KD_NL * min2f(fabsf(vx) / KD_NL_VX, 1.0f));
            if (reacq_calm_t > 0.0f) kd *= 0.35f;

            float u_raw = kp * x + integ + kd * vx;

            float dyn = clampf(fabsf(vx) / I_VX_REF, 0.0f, 1.0f);
            if (ae > I_DEAD_MM && ae < I_ACTIVE_MM && !(fabsf(u_raw) > a_cap && ((u_raw > 0.0f) == (x > 0.0f))))
                integ += KI_CURVE * (x - s_dir * I_DEAD_MM) * dt * (1.0f - dyn);
            float leak = 1.0f / I_LEAK_TAU + I_LEAK_DYN * dyn;
            if (ae > 1.0f && integ * x < 0.0f) leak *= I_LEAK_WRONG;
            integ = clampf(integ - integ * (leak * dt), -I_MAX, I_MAX);

            u = clampf(u_raw, -a_cap, a_cap);
            if (!handover) {
                float lim = (U_SLEW_CALM + (U_SLEW_FAST - U_SLEW_CALM) * max2f(risk_f, risk_v_f)) * dt;
                u = clampf(clampf(u, u_cmd - lim, u_cmd + lim), -a_cap, a_cap);
            }

            v_t = V_MAX - K_SPD_E * max2f(0.0f, ae - E_SPD_DEAD_MM) - K_SPD_R * risk_v_f;
            v_t = clampf(v_t, V_CORNER_MIN - (V_CORNER_MIN - V_BRAKE_MIN) * risk_v_f, V_MAX);
            dn_rate = ACC_DN_SOFT + (ACC_DN - ACC_DN_SOFT) * risk_v_f;
        } else {
            if (is_gap_loss && lost_d < GAP_HOLD_MM) {
                u = clampf(u_ref * 0.45f + integ * 0.25f, -18.0f, 18.0f);
                v_t = min2f(v_cmd, V_GAP_MAX);
                risk_f   -= risk_f   * (dt / (RISK_TAU_DN + dt));
                risk_v_f -= risk_v_f * (dt / (RISK_TAU_DN + dt));
            } else {
                if (!handover) { f_in = F_IN_LOST; a_cap = 0.5f * (255.0f - f_in); }
                float lx = fabsf(lost_x);
                float u0 = min2f(KP * (1.0f + lx / KP_NL_MM) * max2f(lx, 20.0f), U_LOST_MAX);
                u = (float)lost_side * min2f(u0 + (U_LOST_MAX - u0) * clampf(lost_t / LOST_RAMP_S, 0.0f, 1.0f), a_cap);
                v_t = V_LOST;
                risk_f = risk_v_f = 1.0f;
            }
        }

        if (launch_t < SOFT_START_S) {
            float v_ss = V_START + (V_MAX - V_START) * (launch_t / SOFT_START_S);
            if (v_t > v_ss) v_t = v_ss;
        }
        if (handover) {
            float v_ho = (fabsf(x) > 9.0f || fabsf(vx) > 150.0f) ? V_HANDOVER : 245.0f;
            if (v_t > v_ho) v_t = v_ho;
        }

        rampSpeed(v_t, dt, dn_rate);
        mixWheels(v_cmd, u, f_in, vl, vr);
        u_cmd = u;
        risk_dbg = max2f(risk_f, risk_v_f);

        if (!handover && !in_loss && !on_crossbar && !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0)
            u_ref += (u - u_ref) * (dt / (U_REF_TAU + dt));
    }

    void outCrossPass(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt; mode_d += max2f(0.0f, v_est) * dt;
        trackWings(s, dt); refineCenter();
        u_hold -= u_hold * (dt / 0.10f);
        bool clear = w_fell[0] && w_fell[1] &&
                     (s[0] < clampf(max2f(0.32f, base_o[0] + 0.14f), 0.32f, 0.55f)) &&
                     (s[3] < clampf(max2f(0.32f, base_o[1] + 0.14f), 0.32f, 0.55f));
        clear_ticks = clear ? (clear_ticks + 1) : 0;
        rampSpeed(V_MAX, dt);
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;

        if ((clear_ticks >= 3 && mode_d > 14.0f) ||
            (((w_fell[0] && w_fell[1]) || mode_d > 70.0f) && dist_mm >= end_odo) ||
            mode_d > CROSS_PASS_MAX_MM || mode_t > CROSS_PASS_MAX_S) {
            mode = M_RUN; need_reinit = true; in_loss = is_gap_loss = false;
            risk_f = risk_v_f = 0.0f;
            lost_t = lost_d = 0.0f; last_cross_dist = dist_mm; clearWings();
        }
    }

    void outTurnArc(const SensorData& sd, float dt, float& vl, float& vr, bool& brk_l, bool& brk_r) {
        const float* s = sd.ir;
        mode_t   += dt;
        turn_yaw += w_est * dt;

        float old_out  = (turn_dir > 0) ? s[0] : s[3];
        float old_in   = (turn_dir > 0) ? s[1] : s[2];
        float new_in_s = (turn_dir > 0) ? s[2] : s[1];
        float new_out  = (turn_dir > 0) ? s[3] : s[0];
        float inner_on = (line_w_est < 22.0f) ? 0.25f : 0.36f;

        if (!pivot_left_crossbar && (!isCrossbar(s) || mode_t >= 0.038f)) {
            if (old_out < 0.36f || new_out < 0.36f || mode_t >= 0.038f)
                pivot_left_crossbar = true;
        }
        if (pivot_left_crossbar) {
            if (old_in < PIVOT_OLD_GONE) pivot_old_in_clear = true;
            if (mode_t >= 0.025f) {
                if (old_out > PIVOT_OLD_ON) { if (++white_ticks >= 2) pivot_old_seen = true; }
                else white_ticks = 0;
            }
        }

        bool old_gone = (pivot_old_seen || mode_t > PIVOT_RELAX_T) && (old_out < PIVOT_OLD_GONE);
        bool new_in   = (new_in_s > inner_on) || (pivot_old_in_clear && old_in > inner_on);

        float zp = 0.0f, L_tmp = L_eff;
        if (old_gone && measure(sd, zp, L_tmp)) {
            if (pivot_has_z) {
                float v_inst = (zp - pivot_prev_z) / dt;
                if (v_inst * (float)(-turn_dir) > 30.0f && fabsf(v_inst) < 1200.0f)
                    pivot_vx_est = (pivot_vx_est == 0.0f) ? v_inst : (0.5f * pivot_vx_est + 0.5f * v_inst);
            }
            pivot_prev_z = zp; pivot_has_z = true;
        } else pivot_has_z = false;

        float inner_bemf = (turn_dir > 0) ? bemf_r : bemf_l;
        bool in_brake_phase = (mode_t < ARC_BRAKE_MIN_S) ||
                              (mode_t < ARC_BRAKE_MAX_S && inner_bemf > 35.0f);

        float p_outer = P_ARC_OUTER;
        float p_inner = 0.0f;
        bool  brk_in  = false;

        if (in_brake_phase) {
            p_outer = P_ARC_OUTER;
            brk_in  = true;
        } else if (!old_gone) {
            p_outer = P_ARC_OUTER;
            p_inner = P_ARC_INNER_SNAP;
        } else if (new_out > 0.24f) {
            p_outer = P_APPROACH_OUTER;
            brk_in  = true;
        } else {
            p_outer = P_ARC_OUTER;
            p_inner = P_ARC_INNER_SNAP;
        }

        if (turn_dir > 0) { vl = p_outer; vr = p_inner; brk_l = false;  brk_r = brk_in; }
        else              { vl = p_inner; vr = p_outer; brk_l = brk_in; brk_r = false;  }
        v_cmd = V_HANDOVER;

        bool angle_ok = (fabsf(turn_yaw) >= PIVOT_MIN_YAW) || (mode_t >= 0.120f);
        if (old_gone && new_in && angle_ok && mode_t >= PIVOT_MIN_T) {
            float z = 0.0f;
            if (!measure(sd, z, L_tmp)) z = (float)turn_dir * 10.0f;
            x = x_slow = z;
            vx = (fabsf(pivot_vx_est) > 30.0f) ? pivot_vx_est : (-(float)turn_dir * PIVOT_SWEEP_MMPS * v_scale);
            need_reinit = in_loss = is_gap_loss = false;
            integ = u_cmd = risk_f = risk_v_f = lost_t = lost_d = reacq_vx = 0.0f;
            handover_t = HANDOVER_T; v_cmd = V_HANDOVER;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings(); mode = M_RUN; mode_t = 0.0f;
            return;
        }
        if (mode_t > T_PIVOT_MAX) {
            x = x_slow = vx = integ = risk_f = risk_v_f = handover_t = lost_t = lost_d = reacq_vx = 0.0f;
            need_reinit = true; in_loss = is_gap_loss = false; v_cmd = V_LOST;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings(); mode = M_RUN; mode_t = 0.0f;
        }
    }
};

// ==================== BIEN TOAN CUC ====================
Brain         brain;
int           ir_white_cal[4]  = { ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM };
int           ir_black_cal[4]  = { ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM };
int           adc_white_anchor = ADC_WHITE_NOM;
int           track_span_est   = 1500;
float         floor_cm         = 0.0f;
float         local_peak       = 1.0f;
unsigned long startMicros      = 0;
unsigned long lastStepMicros   = 0;
uint32_t      loopFrameCount   = 0;

static inline int readIRFiltered(int pin) {
    (void)analogRead(pin);
    int a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
    int hi = (a > b) ? a : b, lo = (a < b) ? a : b;
    return (c > hi) ? hi : ((c < lo) ? lo : c);
}

static void commitArmedCalibration(const int bright_seen[4], const int dark_seen[4]) {
    int g_white = bright_seen[0], g_dark = dark_seen[0];
    for (int i = 1; i < 4; i++) {
        g_white = brighterOf(g_white, bright_seen[i]);
        g_dark  = darkerOf(g_dark, dark_seen[i]);
    }
    if ((IR_POL < 0) ? (g_white <= ADC_RAW_LO_OK + 200) : (g_white >= ADC_RAW_HI_OK - 200)) {
        g_white = ADC_WHITE_NOM; g_dark = ADC_BLACK_NOM;
    }
    adc_white_anchor = g_white;
    int init_span = darker(g_dark, g_white);
    bool has_line = (init_span >= START_SPAN_MIN && g_dark > ADC_RAW_LO_OK && g_dark < ADC_RAW_HI_OK);
    if (!has_line) {
        g_dark = constrain(g_white + IR_POL * 1500, ADC_RAW_LO_OK + 50, ADC_RAW_HI_OK - 50);
        init_span = absi(g_dark - g_white);
    }
    track_span_est = constrain(init_span, INIT_CONTRAST_SPAN, 2800);
    int w_tol = min(init_span / 3, 750);
    for (int i = 0; i < 4; i++) {
        ir_white_cal[i] = (darker(bright_seen[i], g_white) < w_tol) ? bright_seen[i] : g_white;
        int dk = dark_seen[i];
        ir_black_cal[i] = (has_line && darker(dk, ir_white_cal[i]) >= START_SPAN_MIN && dk > ADC_RAW_LO_OK && dk < ADC_RAW_HI_OK)
                          ? dk : darkerOf(g_dark, ir_white_cal[i] + IR_POL * track_span_est);
    }
    floor_cm = 0.0f; local_peak = 1.0f;
}

// ==================== BLACKBOX ====================
#if ENABLE_BLACKBOX
#define BB_N 4500
struct BBRec {
    uint16_t t10;
    int16_t  x10, u, pl, pr, yaw_deg;
    uint8_t  s[4], mode, map_st, cm, rk;
};
struct Blackbox {
    static BBRec    buf[BB_N];
    static uint16_t count;
    static bool     dumped, was_running;
    static void clear() { count = 0; dumped = false; }
    static void record(const SensorData& sd, int pl, int pr) {
        if (count >= BB_N) return;
        BBRec& r = buf[count++];
        r.t10     = (uint16_t)clampf(sd.elapsed_sec * 100.0f, 0.0f, 65535.0f);
        r.x10     = (int16_t)clampf(brain.x * 10.0f, -32000.0f, 32000.0f);
        r.u       = (int16_t)clampf(brain.u_cmd, -32000.0f, 32000.0f);
        r.pl      = (int16_t)pl;
        r.pr      = (int16_t)pr;
        r.yaw_deg = (int16_t)clampf(brain.seg_yaw * 57.2958f, -32000.0f, 32000.0f);
        for (int i = 0; i < 4; i++) r.s[i] = (uint8_t)clampf(sd.ir[i] * 255.0f, 0.0f, 255.0f);
        r.mode   = (uint8_t)brain.mode;
        r.map_st = (uint8_t)brain.map_state;
        r.cm     = (uint8_t)clampf(floor_cm * 255.0f, 0.0f, 255.0f);
        r.rk     = (uint8_t)clampf(brain.risk_dbg * 255.0f, 0.0f, 255.0f);
    }
    static void dump() {
        if (count < 10) return;
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(20);
#endif
        Serial.println("\n=== VIP v7.3 BLACKBOX START ===");
        Serial.println("t,x_mm,u,pl,pr,s0,s1,s2,s3,mode,map_st,yaw_deg,cm,risk");
        for (uint16_t i = 0; i < count; i++) {
            const BBRec& r = buf[i];
            Serial.printf("%.2f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n",
                          r.t10 * 0.01f, r.x10 * 0.1f, (int)r.u, (int)r.pl, (int)r.pr,
                          (int)r.s[0], (int)r.s[1], (int)r.s[2], (int)r.s[3],
                          (int)r.mode, (int)r.map_st, (int)r.yaw_deg,
                          r.cm / 255.0f, r.rk / 255.0f);
            if ((i & 0x0F) == 0) delay(1);
        }
        Serial.println("=== VIP v7.3 BLACKBOX END (go 'd' de xa lai) ===");
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(0);
#endif
    }
};
BBRec    Blackbox::buf[BB_N];
uint16_t Blackbox::count       = 0;
bool     Blackbox::dumped      = false;
bool     Blackbox::was_running = false;
#endif

// ==================== SETUP ====================
void setup() {
    forceMotorPinsLowNow();
#if defined(ESP32)
    setCpuFrequencyMhz(80);
    analogReadResolution(12);
    gpio_reset_pin((gpio_num_t)W_LED_ON);
    gpio_reset_pin((gpio_num_t)IR_LED_ON);
#endif
    pinMode(W_LED_ON, OUTPUT);  digitalWrite(W_LED_ON, LOW);
    pinMode(IR_LED_ON, OUTPUT); digitalWrite(IR_LED_ON, HIGH);
    for (int i = 0; i < 4; i++) pinMode(PIN_IR[i], INPUT);

#if ENABLE_SERIAL_TELEMETRY || ENABLE_BLACKBOX
    Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
#endif

    bootChirpPreLEDC();
    setupMotorsHardware();
    resetMotorState();

    for (int i = 0; i < 4; i++) {
        ir_white_cal[i] = ADC_WHITE_NOM;
        ir_black_cal[i] = ADC_BLACK_NOM;
    }
    adc_white_anchor = ADC_WHITE_NOM;
    track_span_est   = 1500;
    floor_cm         = 0.0f;
    local_peak       = 1.0f;
    brain.resetToArmed();

    startMicros = lastStepMicros = micros();
}

// ==================== LOOP 500 Hz ====================
void loop() {
    unsigned long nowMicros  = micros();
    unsigned long diffMicros = nowMicros - lastStepMicros;
    if (diffMicros < 2000) return;
    lastStepMicros = nowMicros;
    float dt = (float)diffMicros * 1e-6f;
    if (dt > 0.025f) dt = 0.002f;
    loopFrameCount++;

    SensorData sd;
    sd.elapsed_sec = (float)(nowMicros - startMicros) * 1e-6f;
    int frame_min_raw = 4095, frame_max_raw = 0;
    for (int i = 0; i < 4; i++) {
        int raw = readIRFiltered(PIN_IR[i]);
        sd.raw_ir[i] = raw;
        if (raw < frame_min_raw) frame_min_raw = raw;
        if (raw > frame_max_raw) frame_max_raw = raw;
    }
    int frame_span   = frame_max_raw - frame_min_raw;
    int frame_bright = (IR_POL > 0) ? frame_min_raw : frame_max_raw;
    sd.frame_span    = frame_span;
    sd.frame_bright  = frame_bright;
    sd.in_air        = (IR_POL < 0) ? (frame_max_raw <= ADC_RAW_LO_OK + 60) : (frame_min_raw >= ADC_FRAME_HI_OK);

    bool raw_in_range = (IR_POL < 0) ? (frame_min_raw > ADC_RAW_LO_OK) : (frame_max_raw < ADC_FRAME_HI_OK);
    bool slow_tick    = ((loopFrameCount & 0x1F) == 0);
    bool running      = (brain.mode != Brain::M_ARMED);

    if (running && raw_in_range && frame_span > MIN_CONTRAST_SPAN) {
        if (darker(frame_bright, adc_white_anchor) < 0)
            adc_white_anchor = (adc_white_anchor * 3 + frame_bright) / 4;
        else if (slow_tick && darker(frame_bright, adc_white_anchor) > 150) {
            if ((IR_POL < 0) ? (frame_bright > ADC_RAW_LO_OK + 350) : (frame_bright < ADC_RAW_HI_OK - 350))
                adc_white_anchor += IR_POL;
        }
    }

    bool valid_contrast = running && (frame_span > MIN_CONTRAST_SPAN) &&
                          (darker(frame_bright, adc_white_anchor) < WHITE_SEEN_MAX_DARK) && raw_in_range;
    bool wing_busy = brain.w_on[0] || brain.w_on[1] || brain.w_rose[0] || brain.w_rose[1] ||
                     brain.on_crossbar || (brain.mode != Brain::M_RUN);

    float n_raw[4]; int eff_span[4], span_i_arr[4];
    int max_ch_span = MIN_CONTRAST_SPAN, span_sum = 0;
    int black_learn_thr = (track_span_est > MIN_CONTRAST_SPAN * 2) ? (track_span_est / 2) : MIN_CONTRAST_SPAN;

    for (int i = 0; i < 4; i++) {
        int raw = sd.raw_ir[i];
        bool raw_ok    = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK) : (raw < ADC_RAW_HI_OK);
        bool bright_ok = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK + 400) : (raw < ADC_RAW_HI_OK - 400);
        if (running && raw_ok && bright_ok && darker(raw, ir_white_cal[i]) < 0) {
            ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            if (darker(raw, adc_white_anchor) < 0) adc_white_anchor = (adc_white_anchor * 3 + raw) / 4;
        }
        if (valid_contrast && !wing_busy && (brain.off_track_t < 0.04f || frame_span < INIT_CONTRAST_SPAN)) {
            int dw = darker(raw, ir_white_cal[i]);
            if (slow_tick && dw < ((frame_span < INIT_CONTRAST_SPAN) ? 650 : 280) &&
                darker(ir_white_cal[i], adc_white_anchor) < WHITE_DRIFT_MAX_DARK)
                ir_white_cal[i] += IR_POL;
            dw = darker(raw, ir_white_cal[i]);
            if (brain.off_track_t < 0.04f && raw_ok && dw > black_learn_thr && darker(raw, ir_black_cal[i]) > 0)
                ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
        }
        int min_blk = ir_white_cal[i] + IR_POL * MIN_CONTRAST_SPAN;
        if (slow_tick && valid_contrast && brain.off_track_t < 0.04f &&
            darker(ir_black_cal[i], ir_white_cal[i]) > MIN_CONTRAST_SPAN + 200)
            ir_black_cal[i] -= 4 * IR_POL;
        if (darker(ir_black_cal[i], min_blk) < 0) ir_black_cal[i] = min_blk;
        int span_i = darker(ir_black_cal[i], ir_white_cal[i]);
        span_i_arr[i] = span_i;
        if (span_i > max_ch_span) max_ch_span = span_i;
        span_sum += span_i;
    }

    int shared_floor = max((max_ch_span * 74) / 100, EFF_SPAN_MIN);
    for (int i = 0; i < 4; i++) {
        eff_span[i] = max(span_i_arr[i], shared_floor);
        n_raw[i] = (float)darker(sd.raw_ir[i], ir_white_cal[i]) / (float)eff_span[i];
    }
    if (slow_tick && valid_contrast && !wing_busy)
        track_span_est = (track_span_est * 3 + (span_sum / 4)) / 4;

#if !ENABLE_FLOOR_TRACK
    (void)eff_span; local_peak = 1.0f;
#else
    if (!running) { floor_cm = 0.0f; local_peak = 1.0f; }
    else {
        float n_min = min4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        float n_max = max4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        bool surf_line_ok = (frame_span >= CM_SPAN_MIN) && ((n_max - n_min) >= 0.16f) && raw_in_range && (brain.off_track_t < 0.04f);

        float cm_tgt = floor_cm;
        if (surf_line_ok && n_min <= CM_MAX) cm_tgt = max2f(0.0f, n_min - CM_DEAD);
        else if (raw_in_range && n_min < floor_cm) cm_tgt = max2f(0.0f, n_min - CM_DEAD);
        floor_cm = clampf(floor_cm + (cm_tgt - floor_cm) * (dt / (((cm_tgt > floor_cm) ? CM_TAU_UP : CM_TAU_DN) + dt)), 0.0f, CM_MAX - CM_DEAD);

        if (surf_line_ok && !brain.in_loss && !brain.on_crossbar && brain.mode == Brain::M_RUN) {
            float pk_obs = clampf((n_max - floor_cm) / PEAK_TARGET, PEAK_MIN, 1.0f);
            bool centered = (max2f(n_raw[1], n_raw[2]) >= max2f(n_raw[0], n_raw[3])) && (fabsf(brain.x) < 12.0f);
            if (pk_obs > local_peak) local_peak += (pk_obs - local_peak) * (dt / (PEAK_TAU_UP + dt));
            else if (centered)       local_peak += (pk_obs - local_peak) * (dt / (PEAK_TAU_DN + dt));
            local_peak = clampf(local_peak, PEAK_MIN, 1.0f);
        }
        if (slow_tick && surf_line_ok && !wing_busy && floor_cm > CM_ABSORB_MIN) {
            for (int i = 0; i < 4; i++) {
                int w = ir_white_cal[i] + IR_POL * (int)(floor_cm * CM_ABSORB_RATE * (float)eff_span[i]);
                if (darker(w, adc_white_anchor) > WHITE_CAL_MAX_DARK) w = adc_white_anchor + IR_POL * WHITE_CAL_MAX_DARK;
                ir_white_cal[i] = w;
            }
        }
    }
#endif

    const float denom = clampf(local_peak, PEAK_MIN, 1.0f);
    for (int i = 0; i < 4; i++) sd.ir[i] = clampf((n_raw[i] - floor_cm) / denom, 0.0f, 1.0f);

    int pwm_l = 0, pwm_r = 0;
    bool brk_l = false, brk_r = false;
    brain.step(sd, dt, bemf_l, bemf_r, pwm_l, pwm_r, brk_l, brk_r);

    if (brain.cal_just_locked) {
        commitArmedCalibration(brain.arm_bright, brain.arm_dark);
        startMicros = nowMicros;
#if ENABLE_BLACKBOX
        Blackbox::clear();
#endif
    }

    if (brain.mode == Brain::M_ARMED) resetMotorState();
    else                              setMotorHardware(pwm_l, pwm_r, brk_l, brk_r, dt);

#if ENABLE_BLACKBOX
    if (brain.mode != Brain::M_ARMED) {
        Blackbox::was_running = true;
        if ((loopFrameCount % 10) == 0) Blackbox::record(sd, brk_l ? -999 : pwm_l, brk_r ? -999 : pwm_r);
    } else {
        if (Blackbox::was_running && !Blackbox::dumped && Blackbox::count >= 10 && Serial) {
            Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros();
        }
        while (Serial.available() > 0) {
            int ch = Serial.read();
            if (ch == 'd' || ch == 'D') { Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros(); }
        }
    }
#endif
}
