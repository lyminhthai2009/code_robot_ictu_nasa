// ============================================================================
// COBOT LINE FOLLOWER v7.2 "HYPER-SMOOTH DOB" - ESP32-C3 - 1000 Hz
// Kien truc: 2D Photometric + 2-DOF Q-Filter Curvature DOB + MX1616H Slow Decay
// Triet tieu 100% dao dong 3.8Hz, toi da toc do om cua (Outer-Wheel 255 PWM)
// ============================================================================
#include <Arduino.h>
#include <math.h>
#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ========================== CAU HINH CHE DO THI DAU ==========================
#define TRACK_PROFILE            0   // 0 = Vong Bang | 1 = Vong Chung Ket
#define ENABLE_SHORTCUT          1   // 1 = Re tat Vong tron 2 (Nga tu thu 3)
#define ENABLE_RACING_LINE       0   // De 0 khi test san so 8; bat 1 khi thi dung chieu san that!
#define ENABLE_FLOOR_TRACK       1   // 1 = Thich ung nen & do cao doc cau 14 do
#define USE_SLOW_DECAY           1   // 1 = MX1616H 20kHz Slow Decay Active Brake
#define USE_ADC_DMA              1   // 1 = ADC Continuous DMA 40kHz (ESP32 Core >= 3.0)
#define ENABLE_BLACKBOX          1   // 1 = Luu hop den 45s, nhac xe len tu dong xa Serial

// Chieu lap hinh thang: +1 neu hang s2-s3 (2.25cm) nam PHIA TRUOC, -1 neu nam PHIA SAU
#define INNER_SENSORS_AHEAD      (+1)

// ========================== KE HOACH NGA TU & BAN DO =========================
#define TURN_RIGHT (+1)
#define TURN_LEFT  (-1)
static const int8_t CROSS_PLAN[]   = { 0, 0, TURN_RIGHT };
static const int    CROSS_PLAN_LEN = (int)(sizeof(CROSS_PLAN) / sizeof(CROSS_PLAN[0]));

#if TRACK_PROFILE == 0
static const float CROSS_MIN_DIST_TABLE[4] = { 340.0f, 650.0f, 1050.0f, 450.0f };
#else
static const float CROSS_MIN_DIST_TABLE[4] = { 240.0f, 430.0f,  680.0f, 320.0f };
#endif

// ========================== HINH HOC CAM BIEN HINH THANG =====================
static const float SENSOR_X_MM[4]      = { -20.0f, -11.25f, 11.25f, 20.0f };
static const float SENSOR_VORONOI_W[4] = {  0.70f,   1.25f,  1.25f,  0.70f };
static const float SENSOR_ROW_SIGN[4]  = {  0.0f, (float)INNER_SENSORS_AHEAD, (float)INNER_SENSORS_AHEAD, 0.0f };
static const float DEFAULT_LINE_W_MM   = 24.5f;
static const float PHOTO_EDGE_TAU_MM   = 2.65f;
static const float K_LEAD_2D           = 0.25f;  // Nhe nhang, khong thoi nhieu vao dao ham
static const float X_LIM_MM            = 33.0f;

static const float SPEED_MMPS_AT_255   = 580.0f; // Tang len 680 neu dung pin Lithium 7.2V

// ========================== TOC DO & OUTER-WHEEL MAX =========================
static const float V_CEIL_MAX          = 255.0f; // Banh ngoai LUON ghim 255 PWM!
static const float V_CEIL_EMERG        = 236.0f; // Chi giam tran nhe khi lech cuc bien >23mm
static const float V_INNER_CALM_MIN    =  25.0f; // Duong thang/cua vua: banh trong luon quay tien
static const float V_INNER_MID_MIN     =   0.0f; // Lech lon: phanh cung Active Brake
static const float V_INNER_HARD_MIN    = -55.0f; // Lech cuc gat >17mm: lui nhe cuu cua
static const float LAUNCH_RAMP_S       = 0.065f;
static const float LAUNCH_V0           = 135.0f;

// ========================== 2-DOF Q-FILTER DOB + NONLINEAR PD ================
// Ty le vang D/P ~ 0.048..0.055s => CHONG LAC TUYET DOI, KHONG BAO GIO DA NGUOC LAI!
static const float KP_BASE             = 5.8f;   // PWM_diff / mm o gan tam (em ru duong thang)
static const float KP_NL_SCALE         = 0.85f;  // Tang do sac muot khi lech ra cua
static const float KP_NL_QUAD          = 0.25f;  // Tang manh o mep ngoai chong vang cua
static const float KP_NL_REF_MM        = 14.0f;
static const float KD_BASE             = 0.32f;  // PWM_diff / (mm/s) o tam
static const float KD_CURVE            = 0.44f;  // PWM_diff / (mm/s) khi cua
static const float X_FILT_TAU_S        = 0.004f; // Loc vi tri 4ms (tre cuc thap!)
static const float VX_FILT_TAU_S       = 0.014f; // Loc van toc 14ms (sach bong nhieu ADC)

// Q-Filter Curvature DOB (tach bang thong <0.8Hz => mien nhiem 100% voi lac 4Hz!)
static const float DOB_GAIN            = 0.70f;  // Tu dong bu 70% do cong cua, doc cau & lech motor
static const float DOB_TAU_S           = 0.200f; // Hang so thoi gian cham 200ms (khong cong huong!)
static const float DOB_RELEASE_TAU_S   = 0.045f; // Xa nhanh khi thoat cua ra thang hoac cua chu S
static const float DOB_MAX_U           = 160.0f; // Gioi han tran u_ff (0.70*160 = +/-112 PWM bu cua)

static const float U_DIFF_MAX          = 315.0f;
static const float U_SLEW_CALM         = 5500.0f;  // Danh lai cuc muot o vung tam
static const float U_SLEW_FAST         = 14000.0f; // Phan xa cuc nhanh khi lech bien

// Racing Line (chi kich hoat khi ENABLE_RACING_LINE = 1)
static const float RACE_BIAS_LOOP1_MM  = -3.0f;
static const float RACE_BIAS_U1_MM     = -2.5f;
static const float RACE_BIAS_POST_MM   = +2.0f;

// ========================== GO GIAM TOC & MAT VACH ===========================
static const float GAP_X_MM            = 10.5f;
static const float GAP_HOLD_MM         = 88.0f;
static const float GAP_REACQ_MAX_S     = 0.12f;
static const float GAP_REACQ_MIN_IR    = 0.30f;
static const float LOST_OUTER_PWM      = 248.0f;
static const float LOST_INNER_PWM      = -105.0f;
static const float LOST_RAMP_S         = 0.055f;
static const float LOST_GIVEUP_S       = 1.15f;
static const float LOST_PRED_T         = 0.022f;

// ========================== NGA TU & FLYING APEX CUT =========================
static const float WING_ON             = 0.50f;
static const float WING_OFF            = 0.32f;
static const float WING_BASE_MAX       = 0.45f;
static const float WING_DELTA_MIN      = 0.25f;
static const float WING_DEBOUNCE_S     = 0.0045f;
static const float CROSS_CONFIRM_S     = 0.0030f;
static const float CROSS_CONFIRM_SK_S  = 0.0050f;
static const float WING_MIN_CHORD_MM   = 4.5f;
static const float WING_LATCH_MM       = 58.0f;
static const float CROSS_PASS_MAX_MM   = 175.0f;
static const float CROSS_PASS_MAX_S    = 0.55f;
static const float CROSS_EXIT_MARGIN   = 11.0f;

// Flying Apex Cut cho Nga tu 3 (Vong tron 2)
static const float APEX_OUTER_KICK     = 255.0f;
static const float APEX_INNER_BRAKE_T  = 0.012f;
static const float APEX_INNER_KICK     = -122.0f;
static const float APEX_OUTER_CATCH    = 250.0f;
static const float APEX_INNER_CATCH    = -28.0f;
static const float APEX_CATCH_OUT_THR  = 0.62f;
static const float PIVOT_OLD_ON        = 0.46f;
static const float PIVOT_OLD_GONE      = 0.25f;
static const float PIVOT_MIN_T         = 0.095f;
static const float PIVOT_RELAX_T       = 0.175f;
static const float T_PIVOT_MAX         = 0.85f;
static const float PIVOT_SWEEP_MMPS    = 260.0f;
static const float HANDOVER_T          = 0.080f;
static const float V_HANDOVER          = 245.0f;

// ========================== THICH UNG NEN & DOC CAU ==========================
static const float CM_DEAD             = 0.05f;
static const float CM_MAX              = 0.72f;
static const float CM_TAU_UP           = 0.030f;
static const float CM_TAU_DN           = 0.022f;
static const float CM_ABSORB_MIN       = 0.05f;
static const float CM_ABSORB_RATE      = 0.04f;
static const float PEAK_TARGET         = 0.88f;
static const float PEAK_MIN            = 0.28f;
static const float PEAK_TAU_UP         = 0.015f;
static const float PEAK_TAU_DN         = 0.065f;

// ========================== PHAN CUNG ESP32-C3 COBOT =========================
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5
#define CH_L_A 0
#define CH_L_B 1
#define CH_R_A 2
#define CH_R_B 3
#define SENSOR_1_PIN 0
#define SENSOR_2_PIN 1
#define SENSOR_3_PIN 3
#define SENSOR_4_PIN 4
#define W_LED_ON  20
#define IR_LED_ON 21

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

static const int ADC_RAW_LO_OK        = 20;
static const int ADC_RAW_HI_OK        = 3850;
static const int ADC_FRAME_HI_OK      = 3950;
static const int LINE_FRAME_SPAN_MIN  = 135;
static const int CM_SPAN_MIN          = 230;
static const int MIN_CONTRAST_SPAN    = 340;
static const int INIT_CONTRAST_SPAN   = 880;
static const int EFF_SPAN_MIN         = 310;
static const int WHITE_SEEN_MAX_DARK  = 2000;
static const int WHITE_DRIFT_MAX_DARK = 1200;
static const int WHITE_CAL_MAX_DARK   = 1600;

const uint8_t PIN_IR[4]          = { SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN };
const int     MOTOR_PWM_FREQ     = 20000;
const int     MIN_N20_PWM        = USE_SLOW_DECAY ? 18 : 36;
const float   RIGHT_MOTOR_TRIM   = 1.00f;
const bool    INVERT_LEFT_MOTOR  = false;
const bool    INVERT_RIGHT_MOTOR = false;
const bool    SWAP_MOTORS_LR     = false;

static const float REV_BRAKE_TIME_S = 0.010f;
static const float BEMF_TAU_S       = 0.035f;

// ========================== HAM TIEN ICH TOAN HOC ============================
static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }
static inline int   roundi(float v) { return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)); }
static inline int   darker(int a, int ref)   { return IR_POL * (a - ref); }
static inline int   darkerOf(int a, int b)   { return (darker(a, b) >= 0) ? a : b; }
static inline int   brighterOf(int a, int b) { return (darker(a, b) <= 0) ? a : b; }
static inline float fastSigmoid(float t) {
    if (t >= 10.0f)  return 0.9999546f;
    if (t <= -10.0f) return 0.0000454f;
    return 1.0f / (1.0f + expf(-t));
}

struct SensorData {
    float ir[4];
    int   raw_ir[4];
    int   frame_span;
    float elapsed_sec;
};

static inline void allocateOuterMax(float v_ceil, float u_diff, float v_in_min, float& vl, float& vr) {
    if (u_diff >= 0.0f) {
        vl = v_ceil;
        vr = v_ceil - u_diff;
        if (vr < v_in_min) vr = v_in_min;
    } else {
        vr = v_ceil;
        vl = v_ceil + u_diff;
        if (vl < v_in_min) vl = v_in_min;
    }
}

// ========================== BO NAO HYPER-SMOOTH DOB v7.2 =====================
class HyperBrain {
public:
    enum Mode : uint8_t { M_RUN = 0, M_CROSS_PASS, M_TURN_APEX };
    Mode  mode;
    float mode_t, mode_d;

    // Trang thai vi tri & van toc sach + Q-Filter Curvature DOB
    float x, vx, x_slow, dy_shift_est;
    float u_dob;        // Uoc luong do cong cua / doc cau cham (<0.8Hz)
    bool  need_reinit;
    float u_diff_cmd, u_ref;

    float v_est, dist_mm, v_scale;
    bool  v_cal_valid;
    float line_w_est;
    int   last_side;

    bool  in_loss, meas_ok, is_gap_loss, on_crossbar;
    float lost_t, lost_d, lost_x;
    int   lost_side;
    int   bump_pulse_cnt;
    float last_gap_end_d;
    float bump_lock_t;

    float handover_t, run_t;
    bool  was_lifted, has_seen_line;
    float off_track_t, all_black_t, air_t;

    bool  base_init;
    float base_o[2];
    bool  w_on[2], w_rose[2], w_fell[2], w_sep;
    float w_deb_t[2];
    float w_cand_d[2], w_rise_d[2], w_fall_d[2], w_half_fall_d[2], w_last_d[2];
    float cross_sec, center_odo, end_odo;
    bool  center_refined;
    float u_hold;
    float cross_conf_t, crossbar_t, clear_t, white_t;
    int   cross_count, turn_dir;
    float last_cross_dist;

    bool  pivot_old_seen, pivot_has_z, pivot_left_crossbar, pivot_old_in_clear;
    float pivot_prev_z, pivot_vx_est;

    HyperBrain() { reset(); }

    void reset() {
        mode = M_RUN; mode_t = mode_d = 0.0f;
        x = vx = x_slow = dy_shift_est = 0.0f;
        u_dob = 0.0f;
        need_reinit = true;
        u_diff_cmd = u_ref = u_hold = 0.0f;
        v_est = dist_mm = 0.0f; v_scale = 1.0f; v_cal_valid = false;
        line_w_est = DEFAULT_LINE_W_MM; last_side = 1;
        in_loss = meas_ok = is_gap_loss = on_crossbar = false;
        lost_t = lost_d = lost_x = 0.0f; lost_side = 1;
        bump_pulse_cnt = 0; last_gap_end_d = -1.0e5f; bump_lock_t = 0.0f;
        handover_t = run_t = 0.0f;
        was_lifted = has_seen_line = false;
        off_track_t = all_black_t = air_t = 0.0f;
        base_init = false; base_o[0] = base_o[1] = 0.0f;
        cross_conf_t = crossbar_t = clear_t = white_t = 0.0f;
        cross_count = 0; turn_dir = 1; last_cross_dist = -1.0e6f;
        cross_sec = 1.0f; center_odo = end_odo = 0.0f; center_refined = false;
        pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
        pivot_prev_z = pivot_vx_est = 0.0f;
        clearWings();
    }

    static bool isCrossbar(const float* s) {
        return (min4f(s[0], s[1], s[2], s[3]) >= 0.46f) ||
               (s[0] >= 0.52f && s[3] >= 0.52f && max2f(s[1], s[2]) >= 0.42f);
    }

    static bool hasRealLine(const float* s, int frame_span) {
        if (frame_span < LINE_FRAME_SPAN_MIN) return false;
        float s_max = max4f(s[0], s[1], s[2], s[3]);
        float s_min = min4f(s[0], s[1], s[2], s[3]);
        if (s_min > 0.86f) return false;
        float in_max = max2f(s[1], s[2]), out_max = max2f(s[0], s[3]), out_min = min2f(s[0], s[3]);
        float min_contrast = (s_min > 0.30f) ? 0.19f : 0.13f;
        if (s_max >= 0.16f && ((s_max - s_min) >= min_contrast || (s_max >= 0.70f && in_max > out_max + 0.04f))) return true;
        return (in_max >= 0.055f && out_min < 0.06f && (in_max - out_max) >= 0.04f);
    }

    bool measurePhotometric2D(const SensorData& sd, float& z_lead, float& z_center, bool allow_w_learn) {
        const float* s = sd.ir;
        if (!hasRealLine(s, sd.frame_span)) return false;

        float s_max = max4f(s[0], s[1], s[2], s[3]);
        float w_thr = max2f(0.04f, 0.09f * s_max);
        float inv   = 1.0f / max2f(1.0f - w_thr, 0.20f);
        float w_raw[4], wsum = 0.0f, num = 0.0f;
        for (int i = 0; i < 4; i++) {
            float c = (s[i] - w_thr) * inv;
            w_raw[i] = (c > 0.0f) ? c : 0.0f;
            float wv = w_raw[i] * SENSOR_VORONOI_W[i];
            wsum += wv;
            num  += wv * SENSOR_X_MM[i];
        }
        if (wsum < 0.032f) return false;
        float x0 = num / wsum;
        if (s[3] >= s[0]) {
            float b = clampf(1.0f - (w_raw[0] + w_raw[1] + w_raw[2]) / 0.30f, 0.0f, 1.0f);
            x0 += b * 10.0f * (1.0f - min2f(s[3], 1.0f));
        } else {
            float b = clampf(1.0f - (w_raw[1] + w_raw[2] + w_raw[3]) / 0.30f, 0.0f, 1.0f);
            x0 -= b * 10.0f * (1.0f - min2f(s[0], 1.0f));
        }
        x0 = clampf(x0, -X_LIM_MM, X_LIM_MM);

        float xc = x0;
        float dy = 0.0f;
        const float half_w  = 0.5f * line_w_est;
        const float inv_tau = 1.0f / PHOTO_EDGE_TAU_MM;

        for (int it = 0; it < 3; it++) {
            float h00 = 0.015f, h01 = 0.0f, h11 = 0.65f;
            float b0  = 0.0f,   b1  = 0.0f;

            for (int i = 0; i < 4; i++) {
                float ci = xc + SENSOR_ROW_SIGN[i] * dy;
                float aL = fastSigmoid((SENSOR_X_MM[i] - ci + half_w) * inv_tau);
                float aR = fastSigmoid((SENSOR_X_MM[i] - ci - half_w) * inv_tau);
                float sh = aL - aR;
                float r  = s[i] - sh;
                float jc = (aR * (1.0f - aR) - aL * (1.0f - aL)) * inv_tau;
                float jd = SENSOR_ROW_SIGN[i] * jc;

                h00 += jc * jc;
                h01 += jc * jd;
                h11 += jd * jd;
                b0  += jc * r;
                b1  += jd * r;
            }
            float inv_det = 1.0f / (h00 * h11 - h01 * h01);
            float dx = clampf(( h11 * b0 - h01 * b1) * inv_det, -6.0f, 6.0f);
            float dd = clampf((-h01 * b0 + h00 * b1) * inv_det, -2.0f, 2.0f);
            xc = clampf(xc + dx, -X_LIM_MM, X_LIM_MM);
            dy = clampf(dy + dd, -3.5f, 3.5f);
            if (fabsf(dx) < 0.08f && fabsf(dd) < 0.05f) break;
        }

        if (allow_w_learn && fabsf(xc) < 4.5f && fabsf(u_diff_cmd) < 60.0f &&
            !w_rose[0] && !w_rose[1] && !on_crossbar && s_max > 0.50f) {
            float grad_w = 0.0f;
            for (int i = 0; i < 4; i++) {
                float ci = xc + SENSOR_ROW_SIGN[i] * dy;
                float aL = fastSigmoid((SENSOR_X_MM[i] - ci + half_w) * inv_tau);
                float aR = fastSigmoid((SENSOR_X_MM[i] - ci - half_w) * inv_tau);
                float r  = s[i] - (aL - aR);
                float jw = 0.5f * (aL * (1.0f - aL) + aR * (1.0f - aR)) * inv_tau;
                grad_w += jw * r;
            }
            line_w_est = clampf(line_w_est + 0.04f * grad_w, 18.0f, 27.0f);
        }

        dy_shift_est = 0.85f * dy_shift_est + 0.15f * dy;
        z_center = xc;
        z_lead   = clampf(xc + K_LEAD_2D * dy_shift_est, -X_LIM_MM, X_LIM_MM);
        return true;
    }

    void step(const SensorData& sd, float dt, int hw_pl, int hw_pr, int& pl, int& pr, bool& coast_out) {
        pl = 0; pr = 0; coast_out = true;
        if (!(dt > 0.00025f && dt < 0.02f)) dt = 0.001f;
        const float* s = sd.ir;

        int max_raw = sd.raw_ir[0], min_raw = sd.raw_ir[0];
        for (int i = 1; i < 4; i++) {
            if (sd.raw_ir[i] > max_raw) max_raw = sd.raw_ir[i];
            if (sd.raw_ir[i] < min_raw) min_raw = sd.raw_ir[i];
        }
        bool in_air = (IR_POL < 0) ? (max_raw <= ADC_RAW_LO_OK + 60) : (min_raw >= ADC_FRAME_HI_OK);
        air_t = in_air ? (air_t + dt) : 0.0f;

        float min_ir   = min4f(s[0], s[1], s[2], s[3]);
        bool real_line = hasRealLine(s, sd.frame_span);
        bool crossbar  = isCrossbar(s);
        on_crossbar    = crossbar;
        bool all_black = (min_ir > 0.87f) && (sd.frame_span < 420 || min_ir > 0.94f);
        bool no_line   = !all_black && !real_line && !crossbar;
        bool spinning  = (mode == M_TURN_APEX);
        bool in_cross  = (mode == M_CROSS_PASS);

        if (air_t > 0.09f) {
            was_lifted = true;
        } else if (was_lifted) {
            if (real_line && !crossbar && min_ir < 0.65f && !in_air) { reset(); has_seen_line = true; }
        } else if (!has_seen_line) {
            if (real_line && min_ir < 0.75f && !in_air) { has_seen_line = true; off_track_t = all_black_t = 0.0f; }
            else off_track_t += dt;
        } else {
            if (all_black && !spinning && !in_cross) { all_black_t += dt; if (all_black_t > 0.25f) was_lifted = true; }
            else all_black_t = 0.0f;
            if (no_line && !spinning && !in_cross) { off_track_t += dt; if (off_track_t > LOST_GIVEUP_S) was_lifted = true; }
            else off_track_t = 0.0f;
        }
        if (was_lifted || !has_seen_line) {
            v_est = 0.0f; coast_out = true; return;
        }

        run_t += dt;
        updateSpeedEstimate(dt, hw_pl, hw_pr);

        if (mode == M_RUN) runEstimate(sd, dt);
        if (was_lifted) { v_est = 0.0f; coast_out = true; return; }

        float vl = 0.0f, vr = 0.0f;
        switch (mode) {
            case M_RUN:        outRunDOB(dt, vl, vr);         break;
            case M_CROSS_PASS: outCrossPass(s, dt, vl, vr);   break;
            case M_TURN_APEX:  outFlyingApex(sd, dt, vl, vr); break;
        }
        pl = roundi(clampf(vl, -255.0f, 255.0f));
        pr = roundi(clampf(vr, -255.0f, 255.0f));
        coast_out = false;
    }

private:
    void updateSpeedEstimate(float dt, int hw_pl, int hw_pr) {
        float c_hw = max2f(0.0f, 0.5f * (float)(hw_pl + hw_pr));
        float tgt  = v_scale * SPEED_MMPS_AT_255 * (c_hw / 255.0f);
        float tau  = (tgt < v_est) ? 0.045f : 0.032f;
        v_est += (tgt - v_est) * (dt / (tau + dt));
        dist_mm += v_est * dt;
    }

    float getRacingBiasMM() const {
#if !ENABLE_RACING_LINE
        return 0.0f;
#else
        if (bump_lock_t > 0.0f || handover_t > 0.0f) return 0.0f;
        float d_since = dist_mm - last_cross_dist;
        if (cross_count == 1 && d_since > 60.0f) return RACE_BIAS_LOOP1_MM;
        if (cross_count == 2 && d_since > 260.0f && d_since < 900.0f) return RACE_BIAS_U1_MM;
        if (cross_count == 3 && d_since > 680.0f && d_since < 1300.0f) return RACE_BIAS_POST_MM;
        return 0.0f;
#endif
    }

    void updateWingBase(const float* s, float dt) {
        if (!base_init) { base_o[0] = s[0]; base_o[1] = s[3]; base_init = true; }
        const float ab = dt / (0.14f + dt);
        base_o[0] += ((s[0] < 0.45f) ? ab : ab * 0.125f) * (s[0] - base_o[0]);
        base_o[1] += ((s[3] < 0.45f) ? ab : ab * 0.125f) * (s[3] - base_o[1]);
    }

    void clearWings() {
        for (int k = 0; k < 2; k++) {
            w_on[k] = w_rose[k] = w_fell[k] = false; w_deb_t[k] = 0.0f;
            w_cand_d[k] = w_rise_d[k] = w_fall_d[k] = w_half_fall_d[k] = w_last_d[k] = dist_mm;
        }
        w_sep = false; cross_conf_t = crossbar_t = 0.0f;
    }

    void trackWings(const float* s, float dt) {
        const int idx[2] = { 0, 3 };
        for (int k = 0; k < 2; k++) {
            float v = s[idx[k]];
            if (w_on[k]) {
                w_last_d[k] = dist_mm;
                if (v >= WING_ON) w_half_fall_d[k] = dist_mm;
                if (v < WING_OFF) {
                    w_on[k] = false;
                    if (mode == M_RUN && (w_half_fall_d[k] - w_rise_d[k]) < WING_MIN_CHORD_MM && !w_rose[1 - k]) {
                        w_rose[k] = w_fell[k] = false; w_deb_t[k] = 0.0f;
                    } else { w_fell[k] = true; w_fall_d[k] = dist_mm; }
                }
            } else if (!w_rose[k]) {
                bool opp_active  = (s[idx[1 - k]] >= 0.32f) || (w_deb_t[1 - k] > 0.0f) || w_rose[1 - k];
                bool corner_mask = (mode == M_RUN) && !opp_active &&
                                   ((k == 0 && x < -10.0f && s[2] < 0.15f) || (k == 1 && x > 10.0f && s[1] < 0.15f));
                if (!corner_mask && v >= WING_ON && base_o[k] < WING_BASE_MAX && (v - base_o[k]) >= WING_DELTA_MIN) {
                    if (w_deb_t[k] == 0.0f) w_cand_d[k] = dist_mm;
                    w_deb_t[k] += dt;
                    if (w_deb_t[k] >= WING_DEBOUNCE_S) {
                        w_on[k] = w_rose[k] = true; w_fell[k] = false;
                        w_rise_d[k] = w_cand_d[k]; w_half_fall_d[k] = w_last_d[k] = dist_mm;
                    }
                } else w_deb_t[k] = 0.0f;
            } else if (mode == M_RUN && (((dist_mm - w_last_d[k]) > WING_LATCH_MM) ||
                       (w_fell[k] && !w_rose[1 - k] && !w_sep && (dist_mm - w_fall_d[k]) > 18.0f))) {
                w_rose[k] = w_fell[k] = false; w_deb_t[k] = 0.0f;
            }
        }
        if (w_rose[0] || w_rose[1] || w_deb_t[0] > 0.0f || w_deb_t[1] > 0.0f) {
            bool dip_sep    = (s[0] >= WING_ON && s[1] < s[0] - 0.05f && s[2] > 0.18f) ||
                              (s[3] >= WING_ON && s[2] < s[3] - 0.05f && s[1] > 0.18f);
            bool both_outer = (s[0] >= WING_ON && s[3] >= WING_ON && (s[1] > 0.25f || s[2] > 0.25f));
            bool edge_skew  = (s[0] > 0.72f && s[0] >= s[1] - 0.20f && s[2] > 0.30f) ||
                              (s[3] > 0.72f && s[3] >= s[2] - 0.20f && s[1] > 0.30f);
            if (dip_sep || both_outer || edge_skew) w_sep = true;
        } else w_sep = false;
        updateWingBase(s, dt);
    }

    float getMinCrossSpacing() const {
        int idx = (cross_count < 3) ? cross_count : 3;
        return CROSS_MIN_DIST_TABLE[idx] * (v_cal_valid ? 0.86f : 0.78f);
    }

    bool detectCross(const float* s, bool allowed, float dt) {
        crossbar_t = (allowed && isCrossbar(s)) ? (crossbar_t + dt) : 0.0f;
        if (crossbar_t >= CROSS_CONFIRM_S) return true;

        bool skew_ok = fabsf(w_rise_d[1] - w_rise_d[0]) <= (v_cal_valid ? 60.0f : 75.0f);
        bool center  = (s[1] > 0.30f || s[2] > 0.30f || (s[0] >= WING_ON && s[3] >= WING_ON));
        float c0 = (w_on[0] ? dist_mm : w_half_fall_d[0]) - w_rise_d[0];
        float c1 = (w_on[1] ? dist_mm : w_half_fall_d[1]) - w_rise_d[1];
        bool both_strong = (s[0] >= WING_ON && s[3] >= WING_ON && (s[1] >= 0.38f || s[2] >= 0.38f));
        bool both_chord  = (max2f(c0, c1) >= WING_MIN_CHORD_MM) && (min2f(c0, c1) >= 3.2f);
        bool trig = allowed && w_rose[0] && w_rose[1] && w_sep && center && skew_ok && (both_strong || both_chord);
        cross_conf_t = trig ? (cross_conf_t + dt) : 0.0f;
        return cross_conf_t >= (both_strong ? CROSS_CONFIRM_S : CROSS_CONFIRM_SK_S);
    }

    void planCross() {
        float t = min2f(fabsf(w_rise_d[1] - w_rise_d[0]) / (SENSOR_X_MM[3] - SENSOR_X_MM[0]), 3.0f);
        cross_sec  = sqrtf(1.0f + t * t);
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
        float r = clampf(line_w_est / sqrtf(max2f(L_meas * L_meas - skew_term * skew_term, 0.25f * L_meas * L_meas)), 0.72f, 1.38f);
        float v_old = v_scale;
        v_scale = clampf(v_old + (!v_cal_valid ? 0.60f : 0.30f) * (clampf(v_old * r, 0.72f, 1.38f) - v_old), 0.72f, 1.38f);
        v_cal_valid = true;
        v_est *= (v_scale / v_old);
    }

    void enterCross() {
        int idx = cross_count++;
        int act = 0;
#if ENABLE_SHORTCUT
        if (idx < CROSS_PLAN_LEN) act = CROSS_PLAN[idx];
#endif
        for (int k = 0; k < 2; k++) {
            if (!w_rose[k]) {
                w_on[k] = w_rose[k] = true; w_fell[k] = false;
                w_rise_d[k] = (w_deb_t[k] > 0.0f) ? w_cand_d[k] : dist_mm;
                w_half_fall_d[k] = w_last_d[k] = dist_mm;
            }
        }
        last_cross_dist = dist_mm;
        u_hold = u_ref * 0.50f;
        clear_t = white_t = cross_conf_t = crossbar_t = 0.0f;
        mode_t = mode_d = 0.0f;
        bump_lock_t = 0.0f;
        planCross();

        if (act == 0) {
            mode = M_CROSS_PASS;
        } else {
            turn_dir = (act > 0) ? 1 : -1;
            enterFlyingApex();
        }
    }

    void enterFlyingApex() {
        mode = M_TURN_APEX; mode_t = mode_d = 0.0f; white_t = 0.0f;
        pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
        pivot_prev_z = pivot_vx_est = 0.0f;
        clearWings();
    }

    void runEstimate(const SensorData& sd, float dt) {
        const float* s = sd.ir;
        bool crossbar = isCrossbar(s);
        bool allowed  = (handover_t <= 0.03f) && (!in_loss || crossbar) && (run_t > 0.10f) &&
                        ((dist_mm - last_cross_dist) >= getMinCrossSpacing());
        if (allowed) trackWings(s, dt);
        else { clearWings(); updateWingBase(s, dt); }
        bool crossing = detectCross(s, allowed, dt);

        if (bump_lock_t > 0.0f) bump_lock_t = max2f(0.0f, bump_lock_t - dt);

        if (crossbar) {
            in_loss = is_gap_loss = meas_ok = false; lost_t = lost_d = 0.0f;
            x *= expf(-dt / 0.030f);
            vx *= expf(-dt / 0.020f);
        } else {
            float z_lead = 0.0f, z_ctr = 0.0f;
            bool ok = measurePhotometric2D(sd, z_lead, z_ctr, !in_loss && handover_t <= 0.0f);
            if (ok && in_loss && is_gap_loss && lost_t < GAP_REACQ_MAX_S && max4f(s[0], s[1], s[2], s[3]) < GAP_REACQ_MIN_IR) ok = false;
            meas_ok = ok;

            if (meas_ok) {
                if (in_loss) {
                    bool was_gap = is_gap_loss;
                    in_loss = is_gap_loss = false;
                    if (was_gap && lost_d > 5.0f) {
                        if ((dist_mm - last_gap_end_d) < 120.0f) bump_pulse_cnt++;
                        else bump_pulse_cnt = 1;
                        last_gap_end_d = dist_mm;
                        if (bump_pulse_cnt >= 2) bump_lock_t = 0.080f;
                    }
                    if (lost_t > 0.015f) {
                        x  = z_lead;
                        vx = (!was_gap && fabsf(lost_x) >= GAP_X_MM) ? (-(float)lost_side * 120.0f) : 0.0f;
                        if (!was_gap) u_dob *= 0.35f;
                        need_reinit = false;
                    }
                    lost_t = lost_d = 0.0f;
                }

                if (need_reinit || fabsf(z_lead - x) > 26.0f) {
                    x = x_slow = z_lead; vx = 0.0f;
                    need_reinit = false;
                } else {
                    float x_prev = x;
                    x += (z_lead - x) * (dt / (X_FILT_TAU_S + dt));
                    float vx_raw = clampf((x - x_prev) / dt, -1400.0f, 1400.0f);
                    vx += (vx_raw - vx) * (dt / (VX_FILT_TAU_S + dt));
                }
                x_slow += (z_ctr - x_slow) * (dt / (0.025f + dt));
                if (fabsf(z_ctr) > 2.5f) last_side = (z_ctr > 0.0f) ? 1 : -1;
            } else {
                if (!in_loss) {
                    in_loss = true; lost_t = lost_d = 0.0f;
                    is_gap_loss = (fabsf(x_slow) < GAP_X_MM) && (fabsf(u_dob) < 90.0f || bump_lock_t > 0.0f);
                    lost_x = is_gap_loss ? x_slow : clampf(x + LOST_PRED_T * vx, -X_LIM_MM, X_LIM_MM);
                    lost_side = (fabsf(lost_x) > 1.5f) ? ((lost_x > 0.0f) ? 1 : -1) : last_side;
                }
                lost_t += dt; lost_d += v_est * dt;
                x = clampf(x + vx * dt, -X_LIM_MM, X_LIM_MM);
                vx *= expf(-dt / 0.030f);
                if (lost_t > LOST_GIVEUP_S) was_lifted = true;
            }
        }
        if (crossing) enterCross();
    }

    void outRunDOB(float dt, float& vl, float& vr) {
        float v_ceil = V_CEIL_MAX;
        if (run_t < LAUNCH_RAMP_S) {
            v_ceil = LAUNCH_V0 + (V_CEIL_MAX - LAUNCH_V0) * (run_t / LAUNCH_RAMP_S);
        }
        bool handover = false;
        if (handover_t > 0.0f) {
            handover = true; handover_t -= dt;
            v_ceil = min2f(v_ceil, V_HANDOVER);
        }

        float u_diff = 0.0f;
        float v_in_min = V_INNER_CALM_MIN;

        if (on_crossbar && !in_loss) {
            u_diff = clampf(u_ref * 0.45f, -45.0f, 45.0f);
            v_in_min = 180.0f;
        } else if (!in_loss) {
            float x_ref = getRacingBiasMM();
            float e     = x - x_ref;
            float ae    = fabsf(e);

            // Vong trong: Nonlinear PD ty le vang (D/P ~ 0.045..0.055s -> sieu em!)
            float ne = ae / KP_NL_REF_MM;
            float kp = KP_BASE * (1.0f + KP_NL_SCALE * ne + KP_NL_QUAD * ne * ne);
            float kd = KD_BASE + (KD_CURVE - KD_BASE) * clampf((ae - 4.0f) / 14.0f, 0.0f, 1.0f);
            if (bump_lock_t > 0.0f) { kp *= 0.80f; kd *= 0.55f; }

            float u_pd  = kp * e + kd * vx;
            float u_raw = u_pd + DOB_GAIN * u_dob;

            // Cap nhat Q-Filter Curvature DOB (cham 200ms khi bam cua, xa nhanh 45ms khi doi huong!)
            if (!handover && !w_rose[0] && !w_rose[1]) {
                float u_clamped = clampf(u_raw, -DOB_MAX_U, DOB_MAX_U);
                if (ae > 2.2f && (e * u_dob < 0.0f)) {
                    // Vua thoat cua hoac dao chieu cua chu S -> xa nhanh u_dob!
                    u_dob += (0.0f - u_dob) * (dt / (DOB_RELEASE_TAU_S + dt));
                } else {
                    u_dob += (u_clamped - u_dob) * (dt / (DOB_TAU_S + dt));
                }
                u_dob = clampf(u_dob, -DOB_MAX_U, DOB_MAX_U);
            }

            // Slew mem o vung tam (xoa rung), nhanh o mep cua (chong vang)
            float slew = U_SLEW_CALM + (U_SLEW_FAST - U_SLEW_CALM) * clampf((ae - 5.0f) / 12.0f, 0.0f, 1.0f);
            float max_du = slew * dt;
            u_diff = clampf(clampf(u_raw, u_diff_cmd - max_du, u_diff_cmd + max_du), -U_DIFF_MAX, U_DIFF_MAX);

            // Gioi han banh trong muot: giu toc do cua cuc cao!
            if (ae <= 9.0f) {
                v_in_min = V_INNER_CALM_MIN;
            } else if (ae <= 17.0f) {
                float t = (ae - 9.0f) / 8.0f;
                v_in_min = V_INNER_CALM_MIN + (V_INNER_MID_MIN - V_INNER_CALM_MIN) * t;
            } else {
                float t = clampf((ae - 17.0f) / 7.0f, 0.0f, 1.0f);
                v_in_min = V_INNER_MID_MIN + (V_INNER_HARD_MIN - V_INNER_MID_MIN) * t;
            }

            if (ae > 23.5f && !handover) {
                v_ceil = min2f(v_ceil, V_CEIL_EMERG);
            }
        } else {
            if (is_gap_loss && lost_d < GAP_HOLD_MM) {
                u_diff   = clampf(u_ref * 0.35f, -24.0f, 24.0f);
                v_in_min = 215.0f;
            } else {
                float r = clampf(lost_t / LOST_RAMP_S, 0.0f, 1.0f);
                float v_out = min2f(v_ceil, LOST_OUTER_PWM);
                float v_in  = 20.0f + (LOST_INNER_PWM - 20.0f) * r;
                if (lost_side > 0) { vl = v_out; vr = v_in; }
                else               { vl = v_in;  vr = v_out; }
                u_diff_cmd = vl - vr;
                return;
            }
        }

        allocateOuterMax(v_ceil, u_diff, v_in_min, vl, vr);
        u_diff_cmd = vl - vr;

        if (!handover && !in_loss && !on_crossbar && !w_rose[0] && !w_rose[1] && w_deb_t[0] == 0.0f && w_deb_t[1] == 0.0f) {
            u_ref += (u_diff_cmd - u_ref) * (dt / (0.14f + dt));
        }
    }

    void outCrossPass(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt; mode_d += v_est * dt;
        trackWings(s, dt); refineCenter();
        u_hold -= u_hold * (dt / 0.08f);
        u_dob  *= expf(-dt / 0.12f);

        bool clear = w_fell[0] && w_fell[1] &&
                     (s[0] < clampf(max2f(0.32f, base_o[0] + 0.14f), 0.32f, 0.55f)) &&
                     (s[3] < clampf(max2f(0.32f, base_o[1] + 0.14f), 0.32f, 0.55f));
        clear_t = clear ? (clear_t + dt) : 0.0f;

        allocateOuterMax(V_CEIL_MAX, clampf(u_hold, -55.0f, 55.0f), 180.0f, vl, vr);
        u_diff_cmd = vl - vr;

        if ((clear_t >= 0.005f && mode_d > 14.0f) ||
            (((w_fell[0] && w_fell[1]) || mode_d > 70.0f) && dist_mm >= end_odo) ||
            mode_d > CROSS_PASS_MAX_MM || mode_t > CROSS_PASS_MAX_S) {
            mode = M_RUN; need_reinit = true; in_loss = is_gap_loss = false;
            lost_t = lost_d = 0.0f; last_cross_dist = dist_mm; clearWings();
        }
    }

    void outFlyingApex(const SensorData& sd, float dt, float& vl, float& vr) {
        const float* s = sd.ir;
        mode_t += dt;
        float old_out  = (turn_dir > 0) ? s[0] : s[3];
        float old_in   = (turn_dir > 0) ? s[1] : s[2];
        float new_in_s = (turn_dir > 0) ? s[2] : s[1];
        float new_out  = (turn_dir > 0) ? s[3] : s[0];
        float inner_on = (line_w_est < 21.5f) ? 0.24f : 0.35f;

        if (!pivot_left_crossbar && (!isCrossbar(s) || mode_t >= 0.040f)) {
            if (old_out < 0.35f || new_out < 0.35f || mode_t >= 0.040f)
                pivot_left_crossbar = true;
        }
        if (pivot_left_crossbar) {
            if (old_in < PIVOT_OLD_GONE) pivot_old_in_clear = true;
            if (mode_t >= 0.030f) {
                if (old_out > PIVOT_OLD_ON) {
                    white_t += dt;
                    if (white_t >= 0.0035f) pivot_old_seen = true;
                } else white_t = 0.0f;
            }
        }
        bool old_gone = (pivot_old_seen || mode_t > PIVOT_RELAX_T) && (old_out < PIVOT_OLD_GONE);
        bool new_in   = (new_in_s > inner_on) || (pivot_old_in_clear && old_in > inner_on);

        float z_lead = 0.0f, z_ctr = 0.0f;
        if (old_gone && measurePhotometric2D(sd, z_lead, z_ctr, false)) {
            if (pivot_has_z) {
                float v_inst = (z_ctr - pivot_prev_z) / dt;
                if (v_inst * (float)(-turn_dir) > 30.0f && fabsf(v_inst) < 1500.0f)
                    pivot_vx_est = (pivot_vx_est == 0.0f) ? v_inst : (0.5f * pivot_vx_est + 0.5f * v_inst);
            }
            pivot_prev_z = z_ctr; pivot_has_z = true;
        } else pivot_has_z = false;

        float p_outer, p_inner;
        if (!old_gone) {
            p_outer = APEX_OUTER_KICK;
            p_inner = (mode_t < APEX_INNER_BRAKE_T) ? 0.0f : APEX_INNER_KICK;
        } else if (new_out > APEX_CATCH_OUT_THR) {
            p_outer = APEX_OUTER_CATCH;
            p_inner = APEX_INNER_CATCH;
        } else {
            p_outer = APEX_OUTER_KICK;
            p_inner = (mode_t < APEX_INNER_BRAKE_T) ? 0.0f : APEX_INNER_KICK;
        }

        if (turn_dir > 0) { vl = p_outer; vr = p_inner; }
        else              { vl = p_inner; vr = p_outer; }
        u_diff_cmd = vl - vr;

        if (old_gone && new_in && mode_t >= PIVOT_MIN_T) {
            float zl = 0.0f, zc = 0.0f;
            if (!measurePhotometric2D(sd, zl, zc, false)) zc = (float)turn_dir * 8.0f;
            x = x_slow = zc;
            vx = (fabsf(pivot_vx_est) > 30.0f) ? pivot_vx_est : (-(float)turn_dir * PIVOT_SWEEP_MMPS * v_scale);
            u_dob = 0.0f;
            need_reinit = in_loss = is_gap_loss = false;
            lost_t = lost_d = u_diff_cmd = 0.0f;
            handover_t = HANDOVER_T;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings(); mode = M_RUN; mode_t = 0.0f;
            return;
        }
        if (mode_t > T_PIVOT_MAX) {
            x = vx = x_slow = u_dob = handover_t = lost_t = lost_d = u_diff_cmd = 0.0f;
            need_reinit = true; in_loss = is_gap_loss = false;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings(); mode = M_RUN; mode_t = 0.0f;
        }
    }
};

// ========================== BIEN TOAN CUC & PHAN CUNG ========================
HyperBrain brain;
int   ir_white_cal[4] = { ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM };
int   ir_black_cal[4] = { ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM };
int   adc_white_anchor = ADC_WHITE_NOM, track_span_est = 1500;
float floor_cm = 0.0f, local_peak = 1.0f;

unsigned long startMicros = 0, lastStepMicros = 0;
uint32_t loopFrameCount = 0;
int   prev_hw_pwm_l = 0, prev_hw_pwm_r = 0;
float rev_brake_t_l = 0.0f, rev_brake_t_r = 0.0f;
float bemf_l = 0.0f, bemf_r = 0.0f;

// ========================== ADC CONTINUOUS DMA (CORE 3.x) ====================
#if defined(ESP32) && defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3) && USE_ADC_DMA
#define ADC_DMA_ACTIVE 1
static volatile bool adc_dma_ready = false;
static bool adc_dma_ok = false;
static int  dma_latest_raw[4] = { ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM };

void IRAM_ATTR onAdcDmaDone() {
    adc_dma_ready = true;
}

void initIRAdcHardware() {
    analogContinuousSetWidth(12);
    analogContinuousSetAtten(ADC_11db);
    if (analogContinuous(PIN_IR, 4, 10, 40000, &onAdcDmaDone)) {
        adc_dma_ok = analogContinuousStart();
    }
}

bool acquireIRFrame(int out_raw[4], uint32_t timeout_ms) {
    if (!adc_dma_ok) {
        for (int i = 0; i < 4; i++) {
            (void)analogRead(PIN_IR[i]);
            out_raw[i] = (analogRead(PIN_IR[i]) + analogRead(PIN_IR[i])) >> 1;
        }
        return true;
    }
    adc_continuous_result_t* res = NULL;
    if (analogContinuousRead(&res, timeout_ms) && res != NULL) {
        adc_dma_ready = false;
        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 4; k++) {
                if (res[i].pin == PIN_IR[k]) {
                    dma_latest_raw[k] = res[i].avg_read_raw;
                    break;
                }
            }
        }
        for (int k = 0; k < 4; k++) out_raw[k] = dma_latest_raw[k];
        return true;
    }
    for (int k = 0; k < 4; k++) out_raw[k] = dma_latest_raw[k];
    return false;
}
#else
#define ADC_DMA_ACTIVE 0
void initIRAdcHardware() {
#if defined(ESP32)
    analogReadResolution(12);
#endif
    for (int i = 0; i < 4; i++) pinMode(PIN_IR[i], INPUT);
}

bool acquireIRFrame(int out_raw[4], uint32_t) {
    for (int i = 0; i < 4; i++) {
        (void)analogRead(PIN_IR[i]);
        int a = analogRead(PIN_IR[i]);
        int b = analogRead(PIN_IR[i]);
        out_raw[i] = (a + b) >> 1;
    }
    return true;
}
#endif

// ========================== DIEU KHIEN MX1616H 20kHz SLOW DECAY ==============
void setupMotorsHardware() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(PWM_PIN_L_A, MOTOR_PWM_FREQ, 8, CH_L_A);
    ledcAttachChannel(PWM_PIN_L_B, MOTOR_PWM_FREQ, 8, CH_L_B);
    ledcAttachChannel(PWM_PIN_R_A, MOTOR_PWM_FREQ, 8, CH_R_A);
    ledcAttachChannel(PWM_PIN_R_B, MOTOR_PWM_FREQ, 8, CH_R_B);
#elif defined(ESP32)
    ledcSetup(CH_L_A, MOTOR_PWM_FREQ, 8); ledcSetup(CH_L_B, MOTOR_PWM_FREQ, 8);
    ledcSetup(CH_R_A, MOTOR_PWM_FREQ, 8); ledcSetup(CH_R_B, MOTOR_PWM_FREQ, 8);
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

static inline int applyDeadzone(int pwm) {
    if (pwm == 0) return 0;
    int mag = abs(pwm);
    if (mag >= 250) return (pwm > 0) ? 255 : -255;
    int out;
    if (mag < 12) {
        int knee = MIN_N20_PWM + ((12 * (255 - MIN_N20_PWM)) / 255);
        out = (mag * knee) / 12;
    } else {
        out = constrain(MIN_N20_PWM + ((mag * (255 - MIN_N20_PWM)) / 255), 0, 255);
    }
    return (pwm > 0) ? out : -out;
}

static inline int guardReverseChannel(int target, int prev, float bemf, float& brake_t, float dt) {
    if (brake_t > 0.0f) {
        brake_t = max2f(0.0f, brake_t - dt);
        return 0;
    }
    if (prev != 0 && target != 0 && ((target > 0) != (prev > 0))) {
        if (fabsf(bemf) > 55.0f) {
            brake_t = REV_BRAKE_TIME_S;
            return 0;
        }
    }
    return constrain(target, -255, 255);
}

static inline void driveMX1616Channel(int ch_a, int ch_b, int pwm, bool coast) {
    if (coast) { writeLEDC(ch_a, 0); writeLEDC(ch_b, 0); return; }
#if USE_SLOW_DECAY
    if (pwm > 0) {
        writeLEDC(ch_a, 255); writeLEDC(ch_b, 255 - pwm);
    } else if (pwm < 0) {
        writeLEDC(ch_a, 255 + pwm); writeLEDC(ch_b, 255);
    } else {
        writeLEDC(ch_a, 255); writeLEDC(ch_b, 255);
    }
#else
    if (pwm >= 0) { writeLEDC(ch_a, pwm); writeLEDC(ch_b, 0); }
    else          { writeLEDC(ch_a, 0);   writeLEDC(ch_b, -pwm); }
#endif
}

void setMotorHardware(int pwm_l, int pwm_r, float dt, bool coast) {
    if (coast) {
        prev_hw_pwm_l = prev_hw_pwm_r = 0;
        rev_brake_t_l = rev_brake_t_r = 0.0f;
        bemf_l *= 0.9f; bemf_r *= 0.9f;
        driveMX1616Channel(CH_L_A, CH_L_B, 0, true);
        driveMX1616Channel(CH_R_A, CH_R_B, 0, true);
        return;
    }
    if (SWAP_MOTORS_LR) { int tmp = pwm_l; pwm_l = pwm_r; pwm_r = tmp; }
    if (INVERT_LEFT_MOTOR)  pwm_l = -pwm_l;
    if (INVERT_RIGHT_MOTOR) pwm_r = -pwm_r;

    int out_l = guardReverseChannel(applyDeadzone(pwm_l), prev_hw_pwm_l, bemf_l, rev_brake_t_l, dt);
    int out_r = guardReverseChannel(applyDeadzone(roundi((float)pwm_r * RIGHT_MOTOR_TRIM)), prev_hw_pwm_r, bemf_r, rev_brake_t_r, dt);

    prev_hw_pwm_l = out_l; prev_hw_pwm_r = out_r;
    float alpha_m = dt / (BEMF_TAU_S + dt);
    bemf_l += ((float)out_l - bemf_l) * alpha_m;
    bemf_r += ((float)out_r - bemf_r) * alpha_m;

    driveMX1616Channel(CH_L_A, CH_L_B, out_l, false);
    driveMX1616Channel(CH_R_A, CH_R_B, out_r, false);
}

// ========================== HOP DEN BLACKBOX v7.2 ============================
#if ENABLE_BLACKBOX
#define BB_N 4500
struct BBRec {
    uint16_t t10;
    int16_t  x10, dob, u, pl, pr;
    uint8_t  s[4], mode, cc, w_mm, cm;
};
struct Blackbox {
    static BBRec buf[BB_N];
    static uint16_t count;
    static bool dumped, was_lifted_prev;
    static void clear() { count = 0; dumped = false; }
    static void record(const SensorData& sd, int pl, int pr) {
        if (count >= BB_N) return;
        BBRec& r = buf[count++];
        r.t10   = (uint16_t)clampf(sd.elapsed_sec * 100.0f, 0.0f, 65535.0f);
        r.x10   = (int16_t)clampf(brain.x * 10.0f, -32000.0f, 32000.0f);
        r.dob   = (int16_t)clampf(brain.u_dob, -32000.0f, 32000.0f);
        r.u     = (int16_t)clampf(brain.u_diff_cmd, -32000.0f, 32000.0f);
        r.pl    = (int16_t)pl; r.pr = (int16_t)pr;
        for (int i = 0; i < 4; i++) r.s[i] = (uint8_t)clampf(sd.ir[i] * 255.0f, 0.0f, 255.0f);
        r.mode  = (uint8_t)brain.mode;
        r.cc    = (uint8_t)brain.cross_count;
        r.w_mm  = (uint8_t)clampf(brain.line_w_est * 5.0f, 0.0f, 255.0f);
        r.cm    = (uint8_t)clampf(floor_cm * 255.0f, 0.0f, 255.0f);
    }
    static void dump() {
        if (count < 10) return;
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(50);
#endif
        Serial.println("\n=== HYPER-SMOOTH v7.2 BLACKBOX START ===");
        Serial.println("t,x_mm,u_dob,u_diff,pl,pr,s0,s1,s2,s3,mode,cross,lineW,cm");
        for (uint16_t i = 0; i < count; i++) {
            const BBRec& r = buf[i];
            Serial.printf("%.2f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%.2f\n",
                          r.t10 * 0.01f, r.x10 * 0.1f, (int)r.dob, (int)r.u, (int)r.pl, (int)r.pr,
                          (int)r.s[0], (int)r.s[1], (int)r.s[2], (int)r.s[3],
                          (int)r.mode, (int)r.cc, r.w_mm * 0.2f, r.cm / 255.0f);
            if ((i & 0x0F) == 0) delay(1);
        }
        Serial.println("=== BLACKBOX END (go 'd' de xa lai) ===");
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(0);
#endif
    }
};
BBRec Blackbox::buf[BB_N]; uint16_t Blackbox::count = 0;
bool Blackbox::dumped = false, Blackbox::was_lifted_prev = false;
#endif

// ========================== SETUP & CALIBRATION ==============================
void setup() {
#if defined(RTC_CNTL_BROWN_OUT_REG) && defined(RTC_CNTL_BROWN_OUT_ENA)
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#endif
    pinMode(PWM_PIN_L_A, OUTPUT); digitalWrite(PWM_PIN_L_A, LOW);
    pinMode(PWM_PIN_L_B, OUTPUT); digitalWrite(PWM_PIN_L_B, LOW);
    pinMode(PWM_PIN_R_A, OUTPUT); digitalWrite(PWM_PIN_R_A, LOW);
    pinMode(PWM_PIN_R_B, OUTPUT); digitalWrite(PWM_PIN_R_B, LOW);
    pinMode(IR_LED_ON, OUTPUT);   digitalWrite(IR_LED_ON, HIGH);
    pinMode(W_LED_ON, OUTPUT);    digitalWrite(W_LED_ON, LOW);
#if defined(ESP32)
    WiFi.mode(WIFI_OFF); btStop();
#endif
#if ENABLE_BLACKBOX
    Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
#endif
    setupMotorsHardware();
    setMotorHardware(0, 0, 0.001f, true);
    initIRAdcHardware();

    int cur_raw[4];
    unsigned long t0 = millis();
    while (millis() - t0 < 1200) {
        acquireIRFrame(cur_raw, 10);
        bool ok0 = (IR_POL < 0) ? (cur_raw[0] > ADC_RAW_LO_OK + 350) : (cur_raw[0] < ADC_RAW_HI_OK - 350);
        bool ok3 = (IR_POL < 0) ? (cur_raw[3] > ADC_RAW_LO_OK + 350) : (cur_raw[3] < ADC_RAW_HI_OK - 350);
        if (ok0 || ok3) break;
        delay(8);
    }

    int prev_c[4]; acquireIRFrame(prev_c, 10);
    t0 = millis();
    while (millis() - t0 < 90) {
        delay(10); acquireIRFrame(cur_raw, 10);
        int md = 0;
        for (int i = 0; i < 4; i++) { md = max(md, abs(cur_raw[i] - prev_c[i])); prev_c[i] = cur_raw[i]; }
        if (md < 100) break;
    }

    int bright_seen[4], dark_seen[4];
    acquireIRFrame(cur_raw, 10);
    for (int i = 0; i < 4; i++) bright_seen[i] = dark_seen[i] = cur_raw[i];
    for (int s = 0; s < 24; s++) {
        acquireIRFrame(cur_raw, 10);
        for (int i = 0; i < 4; i++) {
            bright_seen[i] = brighterOf(bright_seen[i], cur_raw[i]);
            dark_seen[i]   = darkerOf(dark_seen[i], cur_raw[i]);
        }
        delay(2);
    }

    int g_white = bright_seen[0], g_dark = dark_seen[0];
    for (int i = 1; i < 4; i++) {
        g_white = brighterOf(g_white, bright_seen[i]);
        g_dark  = darkerOf(g_dark, dark_seen[i]);
    }
    if ((IR_POL < 0) ? (g_white <= ADC_RAW_LO_OK + 200) : (g_white >= ADC_RAW_HI_OK - 200)) {
        g_white = ADC_WHITE_NOM; g_dark = ADC_BLACK_NOM;
        for (int i = 0; i < 4; i++) dark_seen[i] = ADC_BLACK_NOM;
    }
    adc_white_anchor = g_white;
    int init_span = darker(g_dark, g_white);
    bool has_line = (init_span >= INIT_CONTRAST_SPAN && g_dark > ADC_RAW_LO_OK && g_dark < ADC_RAW_HI_OK);
    if (!has_line) {
        g_dark = constrain(g_white + IR_POL * 1500, ADC_RAW_LO_OK + 50, ADC_RAW_HI_OK - 50);
        init_span = abs(g_dark - g_white);
    }
    track_span_est = constrain(init_span, INIT_CONTRAST_SPAN, 2800);
    int w_tol = has_line ? min(init_span / 3, 850) : 950;
    for (int i = 0; i < 4; i++) {
        ir_white_cal[i] = (darker(bright_seen[i], g_white) < w_tol) ? bright_seen[i] : g_white;
        int dk = dark_seen[i];
        ir_black_cal[i] = (has_line && darker(dk, ir_white_cal[i]) >= INIT_CONTRAST_SPAN && dk > ADC_RAW_LO_OK && dk < ADC_RAW_HI_OK)
                          ? dk : darkerOf(g_dark, ir_white_cal[i] + IR_POL * track_span_est);
    }

    brain.reset(); floor_cm = 0.0f; local_peak = 1.0f; bemf_l = bemf_r = 0.0f;
    startMicros = lastStepMicros = micros();
}

// ========================== VONG LAP DIEU KHIEN 1000 Hz ======================
void loop() {
    unsigned long nowMicros = micros();
    unsigned long diffMicros = nowMicros - lastStepMicros;

#if ADC_DMA_ACTIVE
    if (adc_dma_ok) {
        if (!adc_dma_ready && diffMicros < 1200) return;
    } else
#endif
    {
        if (diffMicros < 1000) return;
    }

    lastStepMicros = nowMicros;
    float dt = (float)diffMicros * 1e-6f;
    if (dt > 0.020f || dt < 0.00025f) dt = 0.001f;
    loopFrameCount++;

    SensorData sData;
    sData.elapsed_sec = (float)(nowMicros - startMicros) * 1e-6f;
    acquireIRFrame(sData.raw_ir, 0);

    int frame_min_raw = 4095, frame_max_raw = 0;
    for (int i = 0; i < 4; i++) {
        int raw = sData.raw_ir[i];
        if (raw < frame_min_raw) frame_min_raw = raw;
        if (raw > frame_max_raw) frame_max_raw = raw;
    }
    int  frame_span   = frame_max_raw - frame_min_raw;
    sData.frame_span  = frame_span;
    int  frame_bright = (IR_POL > 0) ? frame_min_raw : frame_max_raw;
    bool raw_in_range = (IR_POL < 0) ? (frame_min_raw > ADC_RAW_LO_OK) : (frame_max_raw < ADC_FRAME_HI_OK);
    bool slow_tick    = ((loopFrameCount & 0x3F) == 0);

    if (raw_in_range && frame_span > MIN_CONTRAST_SPAN) {
        if (darker(frame_bright, adc_white_anchor) < 0) adc_white_anchor = (adc_white_anchor * 3 + frame_bright) / 4;
        else if (slow_tick && darker(frame_bright, adc_white_anchor) > 150) {
            if ((IR_POL < 0) ? (frame_bright > ADC_RAW_LO_OK + 350) : (frame_bright < ADC_RAW_HI_OK - 350)) adc_white_anchor += IR_POL;
        }
    }
    bool valid_contrast = (frame_span > MIN_CONTRAST_SPAN) && (darker(frame_bright, adc_white_anchor) < WHITE_SEEN_MAX_DARK) && raw_in_range;
    bool wing_busy = brain.w_on[0] || brain.w_on[1] || brain.w_rose[0] || brain.w_rose[1] || brain.on_crossbar || (brain.mode != HyperBrain::M_RUN);

    float n_raw[4]; int eff_span[4], span_i_arr[4];
    int max_ch_span = MIN_CONTRAST_SPAN, span_sum = 0;
    int black_learn_thr = (track_span_est > MIN_CONTRAST_SPAN * 2) ? (track_span_est / 2) : MIN_CONTRAST_SPAN;

    for (int i = 0; i < 4; i++) {
        int raw = sData.raw_ir[i];
        bool raw_ok    = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK)       : (raw < ADC_RAW_HI_OK);
        bool bright_ok = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK + 400) : (raw < ADC_RAW_HI_OK - 400);
        if (raw_ok && bright_ok && darker(raw, ir_white_cal[i]) < 0) {
            ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            if (darker(raw, adc_white_anchor) < 0) adc_white_anchor = (adc_white_anchor * 3 + raw) / 4;
        }
        if (valid_contrast && !brain.was_lifted && !wing_busy && (brain.off_track_t < 0.04f || frame_span < INIT_CONTRAST_SPAN)) {
            int dw = darker(raw, ir_white_cal[i]);
            if (slow_tick && dw < ((frame_span < INIT_CONTRAST_SPAN) ? 650 : 280) && darker(ir_white_cal[i], adc_white_anchor) < WHITE_DRIFT_MAX_DARK)
                ir_white_cal[i] += IR_POL;
            dw = darker(raw, ir_white_cal[i]);
            if (brain.has_seen_line && brain.off_track_t < 0.04f && raw_ok && dw > black_learn_thr && darker(raw, ir_black_cal[i]) > 0)
                ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
        }
        int min_blk = ir_white_cal[i] + IR_POL * MIN_CONTRAST_SPAN;
        if (slow_tick && valid_contrast && brain.has_seen_line && !brain.was_lifted && brain.off_track_t < 0.04f &&
            darker(ir_black_cal[i], ir_white_cal[i]) > MIN_CONTRAST_SPAN + 200) ir_black_cal[i] -= 4 * IR_POL;
        if (darker(ir_black_cal[i], min_blk) < 0) ir_black_cal[i] = min_blk;
        int span_i = darker(ir_black_cal[i], ir_white_cal[i]);
        span_i_arr[i] = span_i;
        if (span_i > max_ch_span) max_ch_span = span_i;
        span_sum += span_i;
    }

    int shared_floor = max((max_ch_span * 74) / 100, EFF_SPAN_MIN);
    for (int i = 0; i < 4; i++) {
        eff_span[i] = max(span_i_arr[i], shared_floor);
        n_raw[i] = (float)darker(sData.raw_ir[i], ir_white_cal[i]) / (float)eff_span[i];
    }
    if (slow_tick && valid_contrast && !brain.was_lifted && !wing_busy)
        track_span_est = (track_span_est * 3 + (span_sum / 4)) / 4;

#if !ENABLE_FLOOR_TRACK
    (void)eff_span; local_peak = 1.0f;
#else
    if (brain.was_lifted) { floor_cm = 0.0f; local_peak = 1.0f; }
    else {
        float n_min = min4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        float n_max = max4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        bool surf_line_ok = (frame_span >= CM_SPAN_MIN) && ((n_max - n_min) >= 0.16f) && raw_in_range && (brain.off_track_t < 0.04f);

        float cm_tgt = floor_cm;
        if (surf_line_ok && n_min <= CM_MAX) cm_tgt = max2f(0.0f, n_min - CM_DEAD);
        else if (raw_in_range && n_min < floor_cm) cm_tgt = max2f(0.0f, n_min - CM_DEAD);
        floor_cm = clampf(floor_cm + (cm_tgt - floor_cm) * (dt / (((cm_tgt > floor_cm) ? CM_TAU_UP : CM_TAU_DN) + dt)), 0.0f, CM_MAX - CM_DEAD);

        if (surf_line_ok && !brain.in_loss && !brain.on_crossbar && brain.mode == HyperBrain::M_RUN) {
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
    for (int i = 0; i < 4; i++) sData.ir[i] = clampf((n_raw[i] - floor_cm) / denom, 0.0f, 1.0f);

    int pwm_l = 0, pwm_r = 0;
    bool coast = true;
    brain.step(sData, dt, prev_hw_pwm_l, prev_hw_pwm_r, pwm_l, pwm_r, coast);
    setMotorHardware(pwm_l, pwm_r, dt, coast);

#if ENABLE_BLACKBOX
    if (!brain.was_lifted) {
        if (Blackbox::was_lifted_prev) { Blackbox::clear(); Blackbox::was_lifted_prev = false; }
        if (brain.has_seen_line && (loopFrameCount % 10) == 0) Blackbox::record(sData, pwm_l, pwm_r);
    } else {
        Blackbox::was_lifted_prev = true;
        if (!Blackbox::dumped && Blackbox::count >= 10 && Serial) {
            Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros();
        }
        while (Serial.available() > 0) {
            int ch = Serial.read();
            if (ch == 'd' || ch == 'D') { Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros(); }
        }
    }
#endif
}
