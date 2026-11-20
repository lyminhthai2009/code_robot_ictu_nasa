// COBOT LINE FOLLOWER v7.2 - ESP32-C3 - 500 Hz - ADRC + Peak-Cluster 0ms + 10-bit Smooth PWM
#include <Arduino.h>
#include <math.h>
#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#define ENABLE_SHORTCUT          1
#define ENABLE_FLOOR_TRACK       1
#define ENABLE_SERIAL_TELEMETRY  0
#define ENABLE_BLACKBOX          1

// 1. THÔNG SỐ LÕI ADRC & QUY HOẠCH TỐC ĐỘ
static const float OMEGA_O          = 34.0f;            // (rad/s) Băng thông ESO
static const float OMEGA_C0         = 15.0f;            // (rad/s) Băng thông điều khiển nền
static const float OMEGA_C_NL_GAIN  = 0.30f;            // Tăng cứng mượt khi lệch biên
static const float OMEGA_C_NL_MM    = 12.0f;            // (mm) Ngưỡng độ lệch thích ứng
static const float ZETA_C           = 0.95f;            // Hệ số giảm chấn tới hạn
static const float B0_GAIN          = 55.0f;            // ((mm/s^2)/PWM) Độ lợi vật lý b0
static const float ESO_U_LAG_TAU    = 0.015f;           // (s) Trễ pha cơ-điện của u_eff
static const float E_GATE_MM        = 5.5f;             // (mm) Cổng chống sốc Huber
static const float Z3_MAX           = B0_GAIN * 100.0f; // (mm/s^2) Trần z3 = 100 PWM
static const float PLATEAU_L3_UP    = 0.65f;            // Hệ số tăng z3 trên mặt bằng ±14.4mm
static const float ESO_LOW_V_MMPS   = 80.0f;            // (mm/s) Suy giảm z3 chống trôi khi chậm
static const float ESO_LOW_V_DECAY  = 0.98f;

static const float V_MAX            = 255.0f;
static const float V_BRAKE_MIN      = 155.0f;
static const float V_START          = 145.0f;
static const float K_CURV           = 0.70f;
static const float U_CURV_DEAD      = 25.0f;
static const float U_CURV_TAU_UP    = 0.020f;
static const float U_CURV_TAU_DN    = 0.075f;
static const float K_RISK_V         = 85.0f;
static const float F_IN_MIN         = 45.0f;
static const float F_IN_TIGHT       = -15.0f;
static const float CURV_TIGHT_0     = 65.0f;
static const float CURV_TIGHT_SPAN  = 35.0f;
static const float ACC_UP           = 2400.0f;
static const float ACC_DN_SOFT      = 1400.0f;
static const float ACC_DN           = 3600.0f;

// 2. NGÃ TƯ, CẢM BIẾN & MẤT LINE
#define TURN_RIGHT (+1)
#define TURN_LEFT  (-1)
static const int8_t CROSS_PLAN[]            = { 0, 0, TURN_RIGHT };
static const int    CROSS_PLAN_LEN          = (int)(sizeof(CROSS_PLAN) / sizeof(CROSS_PLAN[0]));
static const float  CROSS_MIN_DIST_TABLE[4] = { 350.0f, 700.0f, 1100.0f, 500.0f };

static const float SENSOR_X_MM[4]        = { -20.0f, -11.25f, 11.25f, 20.0f };
static const float SENSOR_VORONOI_W[4]   = { 0.70f, 1.25f, 1.25f, 0.70f };
static const float DEFAULT_LINE_W_MM     = 24.0f;
static const float SPEED_MMPS_AT_255     = 540.0f;

static const float PC_BG_REL         = 0.04f;
static const float PC_TIE_EPS        = 0.03f;
static const float PC_VALLEY_EPS     = 0.05f;
static const float PC_DS_MIN         = 0.14f;
static const float WSUM_MIN          = 0.035f;
static const float K_EXT_MM          = 9.5f;
static const float X_LIM_MM          = 30.0f;

static const float GAP_X_MM          = 11.5f;
static const float GAP_HOLD_MM       = 78.0f;
static const float GAP_REACQ_MAX     = 0.14f;
static const float GAP_REACQ_MIN     = 0.35f;
static const float GAP_Z3_DECAY_TAU  = 0.25f;
static const float GAP_U_MAX         = 60.0f;
static const float V_GAP_MAX         = 245.0f;
static const float V_LOST            = 160.0f;
static const float U_LOST_MAX        = 135.0f;
static const float LOST_RAMP_S       = 0.08f;
static const float LOST_GIVEUP_S     = 1.25f;
static const float LOST_PRED_T       = 0.028f;
static const float REACQ_VX          = 120.0f;
static const float F_IN_LOST         = -15.0f;

static const float T_PRED            = 0.035f;
static const float RISK_X0           = 10.0f;
static const float RISK_X1           = 20.0f;
static const float RISK_TAU_UP       = 0.015f;
static const float RISK_TAU_DN       = 0.080f;

static const float WING_ON               = 0.52f;
static const float WING_OFF              = 0.34f;
static const float WING_BASE_MAX         = 0.45f;
static const float WING_DELTA_MIN        = 0.26f;
static const int   WING_DEBOUNCE_TICKS   = 3;
static const int   CROSS_CONFIRM_TICKS   = 2;
static const int   CROSS_CONFIRM_SKEW    = 3;
static const float WING_MIN_CHORD_MM     = 5.0f;
static const float WING_LATCH_MM         = 60.0f;
static const float CROSS_PASS_MAX_MM     = 190.0f;
static const float CROSS_PASS_MAX_S      = 0.70f;
static const float CROSS_EXIT_MARGIN_MM  = 14.0f;
static const float U_REF_TAU             = 0.18f;

static const float V_ALIGN            = 80.0f;
static const float ARC_ALIGN_MM       = 0.0f;
static const float ALIGN_FALLBACK_MM  = 25.0f;
static const float ALIGN_FALLBACK_S   = 0.12f;
static const float P_ARC_OUTER        = 205.0f;
static const float P_ARC_INNER_CLEAR  = 0.0f;
static const float P_ARC_INNER_TIGHT  = -105.0f;
static const float ARC_TIGHT_T        = 0.020f;
static const float P_APPROACH_OUTER   = 135.0f;
static const float P_APPROACH_INNER   = -45.0f;
static const float PIVOT_OLD_ON       = 0.48f;
static const float PIVOT_OLD_GONE     = 0.26f;
static const float PIVOT_MIN_T        = 0.110f;
static const float PIVOT_RELAX_T      = 0.20f;
static const float T_PIVOT_MAX        = 1.00f;
static const float PIVOT_SWEEP_MMPS   = 250.0f;
static const float HANDOVER_T         = 0.22f;
static const float V_HANDOVER         = 155.0f;
static const float F_IN_HANDOVER      = -10.0f;
static const float U_MAX_HANDOVER     = 125.0f;

static const float CM_DEAD            = 0.05f;
static const float CM_MAX             = 0.72f;
static const float CM_TAU_UP          = 0.035f;
static const float CM_TAU_DN          = 0.028f;
static const float CM_ABSORB_MIN      = 0.05f;
static const float CM_ABSORB_RATE     = 0.04f;
static const float PEAK_TARGET        = 0.88f;
static const float PEAK_MIN           = 0.30f;
static const float PEAK_TAU_UP        = 0.020f;
static const float PEAK_TAU_DN        = 0.120f;

// 3. PHẦN CỨNG (5000 Hz, 10-bit LEDC)
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

static const int PWM_FREQ_HZ  = 5000;
static const int PWM_RES_BITS = 10;
static const int PWM_MAX_DUTY = 1023;

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
static const int LINE_FRAME_SPAN_MIN  = 140;
static const int CM_SPAN_MIN          = 240;
static const int MIN_CONTRAST_SPAN    = 360;
static const int INIT_CONTRAST_SPAN   = 900;
static const int EFF_SPAN_MIN         = 320;
static const int WHITE_SEEN_MAX_DARK  = 2000;
static const int WHITE_DRIFT_MAX_DARK = 1200;
static const int WHITE_CAL_MAX_DARK   = 1600;

const int   PIN_IR[4]          = {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN};
const float N20_DEADZONE_D0    = 36.0f;
const float N20_DEADZONE_KNEE  = 12.0f;
const float RIGHT_MOTOR_TRIM   = 1.00f;
const bool  INVERT_LEFT_MOTOR  = false;
const bool  INVERT_RIGHT_MOTOR = false;
const bool  SWAP_MOTORS_LR     = false;

static const float BEMF_TAU_S         = 0.042f;
static const float BEMF_SINGLE_MAX_DV = 155.0f;
static const float BEMF_TOTAL_BUDGET  = 245.0f;
static const float BEMF_FLIP_GUARD    = 60.0f;

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }
static inline int   roundi(float v) { return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)); }
static inline int   darker(int a, int ref)   { return IR_POL * (a - ref); }
static inline int   darkerOf(int a, int b)   { return (darker(a, b) >= 0) ? a : b; }
static inline int   brighterOf(int a, int b) { return (darker(a, b) <= 0) ? a : b; }

struct SensorData {
    float ir[4];
    int   raw_ir[4];
    int   frame_span;
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

class Brain {
public:
    enum Mode : uint8_t { M_RUN = 0, M_CROSS_PASS, M_TURN_ALIGN, M_TURN_PIVOT };
    Mode  mode;
    float mode_t, mode_d;
    float z1, z2, z3, u_eff_f;
    float x, vx, x_slow;
    bool  need_reinit, edge_pinned;
    float u_curv, u_cmd, risk_f, risk_dbg;
    float v_cmd, v_est, dist_mm, v_scale;
    bool  v_cal_valid;
    float line_w_est;
    int   last_side;
    bool  in_loss, meas_ok, is_gap_loss, on_crossbar;
    float lost_t, lost_d, lost_x;
    int   lost_side;
    float handover_t, reacq_vx;
    bool  was_lifted, has_seen_line;
    float off_track_t, all_black_t, air_t, run_t;

    bool  base_init;
    float base_o[2];
    bool  w_on[2], w_rose[2], w_fell[2], w_sep;
    int   w_ticks[2];
    float w_cand_d[2], w_rise_d[2], w_fall_d[2], w_half_fall_d[2], w_last_d[2];
    float cross_sec, center_odo, end_odo, align_rem_mm;
    bool  center_refined;
    float u_ref, u_hold;
    int   cross_confirm, crossbar_ticks, cross_count, clear_ticks, white_ticks, turn_dir;
    float last_cross_dist;
    bool  pivot_old_seen, pivot_has_z, pivot_left_crossbar, pivot_old_in_clear;
    float pivot_prev_z, pivot_vx_est;
    bool  ev_cross, ev_align_end, ev_pivot_end;
    int   ev_idx, ev_act;
    float ev_center_d, ev_al_t, ev_al_d, ev_al_rem, ev_pv_t, ev_pv_z;
    uint8_t ev_al_wf;
    float prev_pl, prev_pr;

    Brain() { reset(); }

    void reset() {
        mode = M_RUN; mode_t = mode_d = 0.0f;
        z1 = z2 = z3 = u_eff_f = 0.0f;
        x = vx = x_slow = 0.0f;
        need_reinit = true; edge_pinned = false;
        u_curv = u_cmd = risk_f = risk_dbg = 0.0f;
        v_cmd = V_START; v_est = dist_mm = 0.0f; v_scale = 1.0f; v_cal_valid = false;
        line_w_est = DEFAULT_LINE_W_MM; last_side = 1;
        in_loss = meas_ok = is_gap_loss = on_crossbar = false;
        lost_t = lost_d = lost_x = 0.0f; lost_side = 1;
        handover_t = reacq_vx = 0.0f;
        was_lifted = has_seen_line = false;
        off_track_t = all_black_t = air_t = run_t = 0.0f;
        base_init = false; base_o[0] = base_o[1] = 0.0f;
        cross_confirm = crossbar_ticks = cross_count = clear_ticks = white_ticks = 0;
        last_cross_dist = 0.0f; turn_dir = 1;
        pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
        pivot_prev_z = pivot_vx_est = 0.0f;
        cross_sec = 1.0f; center_odo = end_odo = align_rem_mm = 0.0f; center_refined = false;
        u_ref = u_hold = 0.0f;
        ev_cross = ev_align_end = ev_pivot_end = false;
        ev_idx = ev_act = 0; ev_center_d = ev_al_t = ev_al_d = ev_al_rem = ev_pv_t = ev_pv_z = 0.0f; ev_al_wf = 0;
        clearWings(); prev_pl = prev_pr = 0.0f;
    }

    static bool isCrossbar(const float* s) {
        return (min4f(s[0], s[1], s[2], s[3]) >= 0.46f) ||
               (s[0] >= 0.52f && s[3] >= 0.52f && max2f(s[1], s[2]) >= 0.42f && min2f(s[1], s[2]) >= 0.32f);
    }

    static bool hasRealLine(const float* s, int frame_span) {
        if (frame_span < LINE_FRAME_SPAN_MIN) return false;
        float s_max = max4f(s[0], s[1], s[2], s[3]), s_min = min4f(s[0], s[1], s[2], s[3]);
        if (s_min > 0.85f) return false;
        float in_max = max2f(s[1], s[2]), out_max = max2f(s[0], s[3]), out_min = min2f(s[0], s[3]);
        float min_contrast = (s_min > 0.30f) ? 0.20f : 0.14f;
        if (s_max >= 0.18f && ((s_max - s_min) >= min_contrast || (s_max >= 0.72f && in_max > out_max + 0.04f))) return true;
        return (in_max >= 0.055f && out_min < 0.06f && (in_max - out_max) >= 0.04f);
    }

    static bool isOuterPlateau(const float* s) {
        return (s[2] > 0.72f && s[3] > 0.72f && s[1] < 0.12f && s[0] < 0.16f) ||
               (s[0] > 0.72f && s[1] > 0.72f && s[2] < 0.12f && s[3] < 0.16f);
    }

    void step(const SensorData& sd, float dt, float hw_pl, float hw_pr, float u_eff, float& pl, float& pr) {
        pl = 0.0f; pr = 0.0f;
        if (!(dt > 0.0005f && dt < 0.02f)) dt = 0.002f;
        if (!isfinite(u_eff)) u_eff = 0.0f;
        u_eff = clampf(u_eff, -255.0f, 255.0f);
        u_eff_f += (u_eff - u_eff_f) * (dt / (ESO_U_LAG_TAU + dt));

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
        bool all_black = (min_ir > 0.86f) && (sd.frame_span < 420 || min_ir > 0.93f);
        bool no_line   = !all_black && !real_line && !crossbar;
        bool spinning  = (mode == M_TURN_PIVOT);
        bool in_cross  = (mode == M_CROSS_PASS || mode == M_TURN_ALIGN);

        if (air_t > 0.10f) was_lifted = true;
        else if (was_lifted) {
            if (real_line && !crossbar && min_ir < 0.65f && !in_air) { reset(); has_seen_line = true; }
        } else if (!has_seen_line) {
            if (real_line && min_ir < 0.75f && !in_air) { has_seen_line = true; off_track_t = all_black_t = 0.0f; }
            else off_track_t += dt;
        } else {
            if (all_black && !spinning && !in_cross) { all_black_t += dt; if (all_black_t > 0.28f) was_lifted = true; }
            else all_black_t = 0.0f;
            if (no_line && !spinning && !in_cross) { off_track_t += dt; if (off_track_t > LOST_GIVEUP_S) was_lifted = true; }
            else off_track_t = 0.0f;
        }
        if (was_lifted) { stopOutputs(); return; }

        run_t += dt;
        updateSpeedEstimate(dt, hw_pl, hw_pr);
        if (!has_seen_line) { stopOutputs(); return; }

        if (mode == M_RUN) runEstimate(sd, dt, u_eff_f);
        if (was_lifted) { stopOutputs(); return; }

        float vl = 0.0f, vr = 0.0f;
        switch (mode) {
            case M_RUN:        outRun(dt, vl, vr);          break;
            case M_CROSS_PASS: outCrossPass(s, dt, vl, vr); break;
            case M_TURN_ALIGN: outAlign(s, dt, vl, vr);     break;
            case M_TURN_PIVOT: outPivot(sd, dt, vl, vr);    break;
        }
        if (!isfinite(vl)) vl = 0.0f;
        if (!isfinite(vr)) vr = 0.0f;
        pl = clampf(vl, -255.0f, 255.0f);
        pr = clampf(vr, -255.0f, 255.0f);
        prev_pl = pl; prev_pr = pr;
    }

private:
    void stopOutputs() { v_cmd = v_est = 0.0f; prev_pl = prev_pr = 0.0f; }

    inline void syncState() {
        if (!isfinite(z1)) z1 = 0.0f;
        if (!isfinite(z2)) z2 = 0.0f;
        if (!isfinite(z3)) z3 = 0.0f;
        z1 = clampf(z1, -X_LIM_MM * 1.2f, X_LIM_MM * 1.2f);
        z2 = clampf(z2, -1600.0f, 1600.0f);
        z3 = clampf(z3, -Z3_MAX, Z3_MAX);
        x = z1; vx = z2;
    }

    void rampSpeed(float target, float dt, float dn = ACC_DN) {
        if (target > v_cmd) { v_cmd += ACC_UP * dt; if (v_cmd > target) v_cmd = target; }
        else                { v_cmd -= dn * dt;     if (v_cmd < target) v_cmd = target; }
    }

    void updateSpeedEstimate(float dt, float hw_pl, float hw_pr) {
        float c_hw = max2f(0.0f, 0.5f * (hw_pl + hw_pr));
        float tgt  = v_scale * SPEED_MMPS_AT_255 * (c_hw / 255.0f);
        float tau  = (tgt < v_est) ? 0.095f : 0.042f;
        v_est += (tgt - v_est) * (dt / (tau + dt));
        dist_mm += v_est * dt;
    }

    bool measure(const SensorData& sd, float z_hint, float& z) const {
        const float* s = sd.ir;
        if (!hasRealLine(s, sd.frame_span)) return false;
        float s_min = min4f(s[0], s[1], s[2], s[3]), s_max = max4f(s[0], s[1], s[2], s[3]), ds = s_max - s_min;
        bool faint_ok = (max2f(s[1], s[2]) >= 0.055f && min2f(s[0], s[3]) < 0.06f && ds >= 0.04f);
        if (ds < PC_DS_MIN && !faint_ok) return false;

        float bg_thr = s_min + PC_BG_REL * ds, r[4], r_max = 0.0f;
        for (int i = 0; i < 4; i++) {
            float ri = s[i] - bg_thr;
            r[i] = (ri > 0.0f) ? ri : 0.0f;
            if (r[i] > r_max) r_max = r[i];
        }
        if (r_max <= 1e-4f) return false;

        int p = 0; float best_dist = 1.0e9f;
        for (int i = 0; i < 4; i++) {
            if ((r_max - r[i]) <= PC_TIE_EPS) {
                float d = fabsf(SENSOR_X_MM[i] - z_hint);
                if (d < best_dist) { best_dist = d; p = i; }
            }
        }
        bool in_c[4] = { false, false, false, false };
        in_c[p] = true;
        for (int i = p - 1; i >= 0; i--) { if (r[i] > 0.0f && r[i] <= r[i + 1] + PC_VALLEY_EPS) in_c[i] = true; else break; }
        for (int i = p + 1; i < 4; i++)  { if (r[i] > 0.0f && r[i] <= r[i - 1] + PC_VALLEY_EPS) in_c[i] = true; else break; }

        float wsum = 0.0f, num = 0.0f; int c_count = 0;
        for (int i = 0; i < 4; i++) {
            if (!in_c[i]) continue;
            float wv = r[i] * SENSOR_VORONOI_W[i];
            wsum += wv; num += wv * SENSOR_X_MM[i]; c_count++;
        }
        if (wsum < WSUM_MIN || c_count == 0) return false;
        float xc = num / wsum;
        if (in_c[3] && !in_c[0] && !in_c[1]) {
            float other = in_c[2] ? r[2] : 0.0f;
            float b = (c_count == 1) ? 1.0f : clampf(1.0f - other / 0.25f, 0.0f, 1.0f);
            xc += b * K_EXT_MM * (1.0f - min2f(s[3], 1.0f));
        } else if (in_c[0] && !in_c[2] && !in_c[3]) {
            float other = in_c[1] ? r[1] : 0.0f;
            float b = (c_count == 1) ? 1.0f : clampf(1.0f - other / 0.25f, 0.0f, 1.0f);
            xc -= b * K_EXT_MM * (1.0f - min2f(s[0], 1.0f));
        }
        z = clampf(xc, -X_LIM_MM, X_LIM_MM);
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
        w_sep = false; cross_confirm = 0; crossbar_ticks = 0;
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
                        w_rose[k] = w_fell[k] = false; w_ticks[k] = 0;
                    } else { w_fell[k] = true; w_fall_d[k] = dist_mm; }
                }
            } else if (!w_rose[k]) {
                bool opp_active  = (s[idx[1 - k]] >= 0.32f) || (w_ticks[1 - k] > 0) || w_rose[1 - k];
                bool corner_mask = (mode == M_RUN) && !opp_active &&
                                   ((k == 0 && x < -10.5f && s[2] < 0.15f) || (k == 1 && x > 10.5f && s[1] < 0.15f));
                if (!corner_mask && v >= WING_ON && base_o[k] < WING_BASE_MAX && (v - base_o[k]) >= WING_DELTA_MIN) {
                    if (w_ticks[k] == 0) w_cand_d[k] = dist_mm;
                    if (++w_ticks[k] >= WING_DEBOUNCE_TICKS) {
                        w_on[k] = w_rose[k] = true; w_fell[k] = false;
                        w_rise_d[k] = w_cand_d[k]; w_half_fall_d[k] = w_last_d[k] = dist_mm;
                    }
                } else w_ticks[k] = 0;
            } else if (mode == M_RUN && (((dist_mm - w_last_d[k]) > WING_LATCH_MM) ||
                       (w_fell[k] && !w_rose[1 - k] && !w_sep && (dist_mm - w_fall_d[k]) > 18.0f))) {
                w_rose[k] = w_fell[k] = false; w_ticks[k] = 0;
            }
        }
        if (w_rose[0] || w_rose[1] || w_ticks[0] > 0 || w_ticks[1] > 0) {
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
        return CROSS_MIN_DIST_TABLE[idx] * (v_cal_valid ? 0.85f : 0.78f);
    }

    bool detectCross(const float* s, bool allowed) {
        crossbar_ticks = (allowed && isCrossbar(s)) ? (crossbar_ticks + 1) : 0;
        if (crossbar_ticks >= CROSS_CONFIRM_TICKS) return true;
        bool skew_ok = fabsf(w_rise_d[1] - w_rise_d[0]) <= (v_cal_valid ? 60.0f : 75.0f);
        bool center  = (s[1] > 0.30f || s[2] > 0.30f || (s[0] >= WING_ON && s[3] >= WING_ON));
        float c0 = (w_on[0] ? dist_mm : w_half_fall_d[0]) - w_rise_d[0];
        float c1 = (w_on[1] ? dist_mm : w_half_fall_d[1]) - w_rise_d[1];
        bool both_strong = (s[0] >= WING_ON && s[3] >= WING_ON && (s[1] >= 0.38f || s[2] >= 0.38f));
        bool both_chord  = (max2f(c0, c1) >= WING_MIN_CHORD_MM) && (min2f(c0, c1) >= 3.5f);
        bool trig = allowed && w_rose[0] && w_rose[1] && w_sep && center && skew_ok && (both_strong || both_chord);
        cross_confirm = trig ? (cross_confirm + 1) : 0;
        return cross_confirm >= (both_strong ? CROSS_CONFIRM_TICKS : CROSS_CONFIRM_SKEW);
    }

    void planCross() {
        float t = min2f(fabsf(w_rise_d[1] - w_rise_d[0]) / (SENSOR_X_MM[3] - SENSOR_X_MM[0]), 3.0f);
        cross_sec  = sqrtf(1.0f + t * t);
        float thick = line_w_est * cross_sec;
        center_odo = 0.5f * (w_rise_d[0] + w_rise_d[1]) + 0.5f * thick;
        end_odo    = max2f(w_rise_d[0], w_rise_d[1]) + thick + CROSS_EXIT_MARGIN_MM;
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
        center_odo = 0.25f * (w_rise_d[0] + w_half_fall_d[0] + w_rise_d[1] + w_half_fall_d[1]);
        float skew_term = (line_w_est * fabsf(w_rise_d[1] - w_rise_d[0])) / (SENSOR_X_MM[3] - SENSOR_X_MM[0]);
        float r = clampf(line_w_est / sqrtf(max2f(L_meas * L_meas - skew_term * skew_term, 0.25f * L_meas * L_meas)), 0.70f, 1.35f);
        float v_old = v_scale;
        v_scale = clampf(v_old + (!v_cal_valid ? 0.65f : 0.35f) * (clampf(v_old * r, 0.70f, 1.35f) - v_old), 0.70f, 1.35f);
        v_cal_valid = true;
        v_est *= (v_scale / v_old);
    }

    void enterCross() {
        int idx = cross_count++, act = 0;
#if ENABLE_SHORTCUT
        if (idx < CROSS_PLAN_LEN) act = CROSS_PLAN[idx];
#endif
        for (int k = 0; k < 2; k++) {
            if (!w_rose[k]) {
                w_on[k] = w_rose[k] = true; w_fell[k] = false;
                w_rise_d[k] = (w_ticks[k] > 0) ? w_cand_d[k] : dist_mm;
                w_half_fall_d[k] = w_last_d[k] = dist_mm;
            }
        }
        last_cross_dist = dist_mm;
        u_hold = clampf(u_ref, -25.0f, 25.0f);
        clear_ticks = white_ticks = cross_confirm = crossbar_ticks = 0;
        mode_t = mode_d = 0.0f;
        planCross();
        align_rem_mm = ARC_ALIGN_MM;
        ev_cross = true; ev_idx = idx + 1; ev_act = act; ev_center_d = center_odo;
        if (act == 0) mode = M_CROSS_PASS;
        else {
            turn_dir = (act > 0) ? 1 : -1;
            if (ARC_ALIGN_MM <= 0.5f) enterPivot();
            else mode = M_TURN_ALIGN;
        }
    }

    void runEstimate(const SensorData& sd, float dt, float u_eff) {
        const float* s = sd.ir;
        bool crossbar = isCrossbar(s);
        bool allowed  = (handover_t <= 0.05f) && (!in_loss || crossbar) && (run_t > 0.12f) &&
                        ((dist_mm - last_cross_dist) >= getMinCrossSpacing());
        if (allowed) trackWings(s, dt);
        else { clearWings(); updateWingBase(s, dt); }
        bool crossing = detectCross(s, allowed);
        edge_pinned = false;

        if (crossbar) {
            in_loss = is_gap_loss = meas_ok = false; lost_t = lost_d = 0.0f;
            z1 *= 0.92f; z2 *= 0.85f; syncState();
        } else {
            float z = 0.0f;
            bool ok = measure(sd, z1, z);
            if (ok && in_loss && is_gap_loss && lost_t < GAP_REACQ_MAX &&
                max4f(s[0], s[1], s[2], s[3]) < GAP_REACQ_MIN) ok = false;
            meas_ok = ok;

            if (meas_ok) {
                if (in_loss) {
                    bool was_gap = is_gap_loss;
                    in_loss = is_gap_loss = false;
                    if (lost_t > 0.012f) {
                        need_reinit = true;
                        reacq_vx = (!was_gap && fabsf(lost_x) >= GAP_X_MM) ? (-(float)lost_side * REACQ_VX) : 0.0f;
                    }
                    if (!was_gap && lost_t > 0.12f) z3 *= 0.50f;
                    lost_t = lost_d = 0.0f;
                }
                if (need_reinit) {
                    z1 = x_slow = z; z2 = reacq_vx; reacq_vx = 0.0f; need_reinit = false;
                } else {
                    float acc_net = z3 - B0_GAIN * u_eff;
                    float z1_hat  = z1 + dt * z2 + 0.5f * dt * dt * acc_net;
                    float z2_hat  = z2 + dt * acc_net;
                    float z3_hat  = z3;
                    float eo      = z - z1_hat;
                    float eo_t    = E_GATE_MM * tanhf(eo / E_GATE_MM);
                    float gamma   = expf(-OMEGA_O * dt), omg = 1.0f - gamma;
                    float L1      = 1.0f - gamma * gamma * gamma;
                    float L2      = (1.5f / dt) * omg * omg * (1.0f + gamma);
                    float L3      = (1.0f / (dt * dt)) * omg * omg * omg;
                    float L3_eff  = isOuterPlateau(s) ? ((eo_t * z1 < 0.0f) ? 0.0f : (L3 * PLATEAU_L3_UP)) : L3;
                    z1 = z1_hat + L1 * eo_t;
                    z2 = z2_hat + L2 * eo_t;
                    z3 = clampf(z3_hat + L3_eff * eo_t, -Z3_MAX, Z3_MAX);
                }
                edge_pinned = ((z1 > 13.0f && s[1] < 0.12f) || (z1 < -13.0f && s[2] < 0.12f));
                if (v_est < ESO_LOW_V_MMPS) z3 *= ESO_LOW_V_DECAY;
                syncState();
                x_slow += (z1 - x_slow) * (dt / (0.030f + dt));
                if (fabsf(z1) > 2.5f) last_side = (z1 > 0.0f) ? 1 : -1;
            } else {
                if (!in_loss) {
                    in_loss = true; lost_t = lost_d = 0.0f;
                    is_gap_loss = (fabsf(x_slow) < GAP_X_MM);
                    lost_x = is_gap_loss ? x_slow : clampf(z1 + LOST_PRED_T * z2, -X_LIM_MM, X_LIM_MM);
                    lost_side = (fabsf(lost_x) > 1.5f) ? ((lost_x > 0.0f) ? 1 : -1) : last_side;
                }
                lost_t += dt; lost_d += v_est * dt;
                if (is_gap_loss && lost_d < GAP_HOLD_MM) {
                    float g_dec = expf(-dt / 0.035f);
                    z1 *= g_dec; z2 *= g_dec; z3 *= expf(-dt / GAP_Z3_DECAY_TAU);
                } else {
                    z1 = clampf(z1 + z2 * dt, -X_LIM_MM, X_LIM_MM); z2 *= 0.94f;
                    if (z3 * (float)lost_side < 0.0f) z3 *= 0.90f;
                    else z3 *= expf(-dt / 0.50f);
                }
                syncState();
                if (lost_t > LOST_GIVEUP_S) was_lifted = true;
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

        float u_ff = z3 / B0_GAIN, u_ff_abs = fabsf(u_ff);
        float tau_c = (u_ff_abs > u_curv) ? U_CURV_TAU_UP : U_CURV_TAU_DN;
        u_curv += (u_ff_abs - u_curv) * (dt / (tau_c + dt));

        float u = 0.0f, v_t = V_MAX, dn_rate = ACC_DN;
        if (on_crossbar && !in_loss) {
            u = clampf(u_ff, -min2f(45.0f, a_cap), min2f(45.0f, a_cap));
            risk_f -= risk_f * (dt / (RISK_TAU_DN + dt));
            v_t = V_MAX; dn_rate = ACC_DN_SOFT;
        } else if (!in_loss) {
            float ae = fabsf(z1), s_dir = (z1 >= 0.0f) ? 1.0f : -1.0f;
            float v_out = (ae > 2.0f) ? max2f(0.0f, s_dir * z2) : 0.0f;
            float risk_raw = clampf((ae + T_PRED * v_out - RISK_X0) / (RISK_X1 - RISK_X0), 0.0f, 1.0f);
            risk_f += (risk_raw - risk_f) * (dt / (((risk_raw > risk_f) ? RISK_TAU_UP : RISK_TAU_DN) + dt));

            if (!handover) {
                float sigma_tight = max2f(risk_f, clampf((u_curv - CURV_TIGHT_0) / CURV_TIGHT_SPAN, 0.0f, 1.0f));
                f_in = F_IN_MIN - (F_IN_MIN - F_IN_TIGHT) * sigma_tight;
                a_cap = 0.5f * (255.0f - f_in);
            }
            float th = tanhf(z1 / OMEGA_C_NL_MM);
            float wc = OMEGA_C0 * (1.0f + OMEGA_C_NL_GAIN * th * th);
            float kp = wc * wc, kd = 2.0f * ZETA_C * wc;
            float kd_term = kd * z2;
            if (!handover && edge_pinned && (z2 * z1 < 0.0f)) kd_term = 0.0f;
            u = clampf((z3 + kp * z1 + kd_term) / B0_GAIN, -a_cap, a_cap);

            float v_curv = V_MAX - K_CURV * max2f(0.0f, u_curv - U_CURV_DEAD);
            v_t = clampf(min2f(v_curv, V_MAX - K_RISK_V * risk_f), V_BRAKE_MIN, V_MAX);
            float urg = max2f(risk_f, clampf((u_curv - 40.0f) / 50.0f, 0.0f, 1.0f));
            dn_rate = ACC_DN_SOFT + (ACC_DN - ACC_DN_SOFT) * urg;
        } else {
            if (is_gap_loss && lost_d < GAP_HOLD_MM) {
                float lim_gap = min2f(GAP_U_MAX, a_cap);
                u = clampf(u_ff, -lim_gap, lim_gap);
                v_t = min2f(v_cmd, V_GAP_MAX);
                risk_f -= risk_f * (dt / (RISK_TAU_DN + dt));
                dn_rate = ACC_DN_SOFT;
            } else {
                if (!handover) { f_in = F_IN_LOST; a_cap = 0.5f * (255.0f - f_in); }
                float kp_eq = (OMEGA_C0 * OMEGA_C0) / B0_GAIN, lx = fabsf(lost_x);
                float u0 = clampf(max2f(kp_eq * max2f(lx, 18.0f), (float)lost_side * u_cmd), 65.0f, U_LOST_MAX);
                u = (float)lost_side * min2f(u0 + (U_LOST_MAX - u0) * clampf(lost_t / LOST_RAMP_S, 0.0f, 1.0f), a_cap);
                v_t = V_LOST; risk_f = 1.0f;
            }
        }
        if (handover) {
            float v_ho = (fabsf(z1) > 8.0f || fabsf(z2) > 130.0f) ? V_HANDOVER : 205.0f;
            if (v_t > v_ho) v_t = v_ho;
        }
        rampSpeed(v_t, dt, dn_rate);
        mixWheels(v_cmd, u, f_in, vl, vr);
        u_cmd = u; risk_dbg = risk_f;
        if (!handover && !in_loss && !on_crossbar && !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0)
            u_ref += (u - u_ref) * (dt / (U_REF_TAU + dt));
    }

    void outCrossPass(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt; mode_d += v_est * dt;
        trackWings(s, dt); refineCenter();
        z1 *= 0.94f; z2 *= 0.90f; syncState();
        u_hold -= u_hold * (dt / 0.10f);
        bool clear = w_fell[0] && w_fell[1] &&
                     (s[0] < clampf(max2f(0.32f, base_o[0] + 0.14f), 0.32f, 0.55f)) &&
                     (s[3] < clampf(max2f(0.32f, base_o[1] + 0.14f), 0.32f, 0.55f));
        clear_ticks = clear ? (clear_ticks + 1) : 0;
        rampSpeed(V_MAX, dt);
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;
        if ((clear_ticks >= 3 && mode_d > 15.0f) || (((w_fell[0] && w_fell[1]) || mode_d > 75.0f) && dist_mm >= end_odo) ||
            mode_d > CROSS_PASS_MAX_MM || mode_t > CROSS_PASS_MAX_S) {
            mode = M_RUN; need_reinit = true; reacq_vx = 0.0f;
            z3 *= 0.50f; u_curv = fabsf(z3 / B0_GAIN);
            in_loss = is_gap_loss = false; risk_f = risk_dbg = 0.0f;
            lost_t = lost_d = 0.0f; last_cross_dist = dist_mm; clearWings();
        }
    }

    void outAlign(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt; mode_d += v_est * dt; align_rem_mm -= v_est * dt;
        trackWings(s, dt); rampSpeed(V_ALIGN, dt);
        u_hold -= u_hold * (dt / 0.08f);
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold; ev_center_d = center_odo;
        if (align_rem_mm <= 0.0f || mode_d >= ARC_ALIGN_MM || mode_d > ALIGN_FALLBACK_MM || mode_t > ALIGN_FALLBACK_S) {
            ev_align_end = true; ev_al_t = mode_t; ev_al_d = mode_d; ev_al_rem = align_rem_mm;
            ev_al_wf = (w_fell[0] ? 1 : 0) | (w_fell[1] ? 2 : 0);
            enterPivot();
        }
    }

    void enterPivot() {
        mode = M_TURN_PIVOT; mode_t = mode_d = 0.0f; white_ticks = 0;
        pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
        pivot_prev_z = pivot_vx_est = 0.0f; clearWings();
    }

    void outPivot(const SensorData& sd, float dt, float& vl, float& vr) {
        const float* s = sd.ir;
        mode_t += dt;
        float old_out = (turn_dir > 0) ? s[0] : s[3], old_in = (turn_dir > 0) ? s[1] : s[2];
        float new_in_s = (turn_dir > 0) ? s[2] : s[1], new_out = (turn_dir > 0) ? s[3] : s[0];
        float inner_on = (line_w_est < 22.0f) ? 0.25f : 0.38f;

        if (!pivot_left_crossbar && (!isCrossbar(s) || mode_t >= 0.045f)) {
            if (old_out < 0.35f || new_out < 0.35f || mode_t >= 0.045f) pivot_left_crossbar = true;
        }
        if (pivot_left_crossbar) {
            if (old_in < PIVOT_OLD_GONE) pivot_old_in_clear = true;
            if (mode_t >= 0.035f) {
                if (old_out > PIVOT_OLD_ON) { if (++white_ticks >= 2) pivot_old_seen = true; }
                else white_ticks = 0;
            }
        }
        bool old_gone = (pivot_old_seen || mode_t > PIVOT_RELAX_T) && (old_out < PIVOT_OLD_GONE);
        bool new_in   = (new_in_s > inner_on) || (pivot_old_in_clear && old_in > inner_on);

        float z_hint = pivot_has_z ? pivot_prev_z : ((float)turn_dir * 12.0f), zp = 0.0f;
        if (old_gone && measure(sd, z_hint, zp)) {
            if (pivot_has_z) {
                float v_inst = (zp - pivot_prev_z) / dt;
                if (v_inst * (float)(-turn_dir) > 30.0f && fabsf(v_inst) < 1200.0f)
                    pivot_vx_est = (pivot_vx_est == 0.0f) ? v_inst : (0.5f * pivot_vx_est + 0.5f * v_inst);
            }
            pivot_prev_z = zp; pivot_has_z = true;
        } else pivot_has_z = false;

        if (old_gone && new_in && mode_t >= PIVOT_MIN_T) {
            float z = 0.0f;
            if (!measure(sd, pivot_has_z ? pivot_prev_z : ((float)turn_dir * 12.0f), z)) z = (float)turn_dir * 12.0f;
            z1 = x_slow = z;
            z2 = (fabsf(pivot_vx_est) > 30.0f) ? pivot_vx_est : (-(float)turn_dir * PIVOT_SWEEP_MMPS * v_scale);
            z3 = 0.0f; u_eff_f = 0.0f; edge_pinned = false; syncState();
            ev_pivot_end = true; ev_pv_t = mode_t; ev_pv_z = z1;
            need_reinit = in_loss = is_gap_loss = on_crossbar = false;
            u_curv = risk_f = risk_dbg = lost_t = lost_d = reacq_vx = 0.0f;
            handover_t = HANDOVER_T; v_cmd = V_HANDOVER;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings(); mode = M_RUN; mode_t = 0.0f;
            outRun(dt, vl, vr);
            return;
        }

        float p_outer, p_inner;
        if (!old_gone) {
            p_outer = P_ARC_OUTER;
            p_inner = (mode_t < ARC_TIGHT_T) ? P_ARC_INNER_CLEAR : P_ARC_INNER_TIGHT;
        } else if (new_out > 0.22f) {
            p_outer = P_APPROACH_OUTER; p_inner = P_APPROACH_INNER;
        } else {
            p_outer = 175.0f; p_inner = (mode_t < ARC_TIGHT_T) ? 0.0f : P_ARC_INNER_TIGHT;
        }
        if (turn_dir > 0) { vl = p_outer; vr = p_inner; } else { vl = p_inner; vr = p_outer; }
        v_cmd = 0.5f * (vl + vr); u_cmd = 0.5f * (vl - vr);

        if (mode_t > T_PIVOT_MAX) {
            ev_pivot_end = true; ev_pv_t = mode_t; ev_pv_z = 99.0f;
            z1 = z2 = z3 = x = x_slow = vx = u_eff_f = 0.0f; edge_pinned = false;
            u_curv = risk_f = risk_dbg = handover_t = lost_t = lost_d = reacq_vx = 0.0f;
            need_reinit = true; in_loss = is_gap_loss = false; v_cmd = V_LOST;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings(); mode = M_RUN; mode_t = 0.0f;
        }
    }
};

Brain brain;
int   ir_white_cal[4] = {ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM};
int   ir_black_cal[4] = {ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM};
int   adc_white_anchor = ADC_WHITE_NOM, track_span_est = 1500;
float floor_cm = 0.0f, local_peak = 1.0f;
unsigned long startMicros = 0, lastStepMicros = 0;
uint32_t loopFrameCount = 0;
float prev_hw_out_l = 0.0f, prev_hw_out_r = 0.0f, bemf_l = 0.0f, bemf_r = 0.0f;
float hw_log_out_l = 0.0f, hw_log_out_r = 0.0f, hw_u_eff = 0.0f;

void setupMotorsHardware() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ_HZ, PWM_RES_BITS, CH_L_A);
    ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ_HZ, PWM_RES_BITS, CH_L_B);
    ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ_HZ, PWM_RES_BITS, CH_R_A);
    ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ_HZ, PWM_RES_BITS, CH_R_B);
#elif defined(ESP32)
    ledcSetup(CH_L_A, PWM_FREQ_HZ, PWM_RES_BITS); ledcSetup(CH_L_B, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_R_A, PWM_FREQ_HZ, PWM_RES_BITS); ledcSetup(CH_R_B, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PWM_PIN_L_A, CH_L_A); ledcAttachPin(PWM_PIN_L_B, CH_L_B);
    ledcAttachPin(PWM_PIN_R_A, CH_R_A); ledcAttachPin(PWM_PIN_R_B, CH_R_B);
#endif
}

static inline void writeLEDC(int ch, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(ch, duty);
#elif defined(ESP32)
    ledcWrite(ch, duty);
#else
    analogWrite(ch, duty >> 2);
#endif
}

static inline float applyN20Deadzone(float p) {
    float ap = fabsf(p);
    if (ap < 1e-4f) return 0.0f;
    if (ap > 255.0f) ap = 255.0f;
    static const float slope = (255.0f - N20_DEADZONE_D0) / 255.0f;
    float out = min2f(255.0f, N20_DEADZONE_D0 * tanhf(ap / N20_DEADZONE_KNEE) + slope * ap);
    return (p >= 0.0f) ? out : -out;
}

static inline float invertN20Deadzone(float y) {
    float ay = fabsf(y);
    if (ay < 1e-4f) return 0.0f;
    if (ay >= 255.0f) return (y >= 0.0f) ? 255.0f : -255.0f;
    static const float slope = (255.0f - N20_DEADZONE_D0) / 255.0f;
    static const float d0k   = N20_DEADZONE_D0 / N20_DEADZONE_KNEE;
    float m = clampf(max2f(ay / (d0k + slope), (ay - N20_DEADZONE_D0) / slope), 0.0f, 255.0f);
    for (int i = 0; i < 2; i++) {
        float th = tanhf(m / N20_DEADZONE_KNEE);
        m = clampf(m - (N20_DEADZONE_D0 * th + slope * m - ay) / (d0k * (1.0f - th * th) + slope), 0.0f, 255.0f);
    }
    return (y >= 0.0f) ? m : -m;
}

static inline float limitSingleChannelBEMF(float target, float prev, float bemf) {
    float out = clampf(clampf(target, bemf - BEMF_SINGLE_MAX_DV, bemf + BEMF_SINGLE_MAX_DV), -255.0f, 255.0f);
    if (((prev > 0.5f && out < -0.5f) || (prev < -0.5f && out > 0.5f)) && fabsf(bemf) > BEMF_FLIP_GUARD) return 0.0f;
    return out;
}

void setMotorHardware(float pwm_l, float pwm_r, float dt) {
    if (!isfinite(pwm_l)) pwm_l = 0.0f;
    if (!isfinite(pwm_r)) pwm_r = 0.0f;
    float pl = pwm_l, pr = pwm_r;
    if (SWAP_MOTORS_LR)     { float tmp = pl; pl = pr; pr = tmp; }
    if (INVERT_LEFT_MOTOR)  pl = -pl;
    if (INVERT_RIGHT_MOTOR) pr = -pr;

    float out_l = limitSingleChannelBEMF(applyN20Deadzone(pl), prev_hw_out_l, bemf_l);
    float out_r = limitSingleChannelBEMF(applyN20Deadzone(pr * RIGHT_MOTOR_TRIM), prev_hw_out_r, bemf_r);

    float dv_l = max2f(0.0f, (out_l >= 0.0f) ? (out_l - bemf_l) : (-out_l + bemf_l));
    float dv_r = max2f(0.0f, (out_r >= 0.0f) ? (out_r - bemf_r) : (-out_r + bemf_r));
    if (dv_l + dv_r > BEMF_TOTAL_BUDGET) {
        float trim = 0.5f * ((dv_l + dv_r) - BEMF_TOTAL_BUDGET);
        if (out_l > 0.0f) out_l = max2f(0.0f, out_l - trim); else if (out_l < 0.0f) out_l = min2f(0.0f, out_l + trim);
        if (out_r > 0.0f) out_r = max2f(0.0f, out_r - trim); else if (out_r < 0.0f) out_r = min2f(0.0f, out_r + trim);
    }
    prev_hw_out_l = out_l; prev_hw_out_r = out_r;
    float alpha_m = dt / (BEMF_TAU_S + dt);
    bemf_l += (out_l - bemf_l) * alpha_m; bemf_r += (out_r - bemf_r) * alpha_m;

    const float scale10 = (float)PWM_MAX_DUTY / 255.0f;
    int dl = constrain(roundi(fabsf(out_l) * scale10), 0, PWM_MAX_DUTY);
    int dr = constrain(roundi(fabsf(out_r) * scale10), 0, PWM_MAX_DUTY);
    if (out_l >= 0.0f) { writeLEDC(CH_L_B, 0); writeLEDC(CH_L_A, dl); } else { writeLEDC(CH_L_A, 0); writeLEDC(CH_L_B, dl); }
    if (out_r >= 0.0f) { writeLEDC(CH_R_B, 0); writeLEDC(CH_R_A, dr); } else { writeLEDC(CH_R_A, 0); writeLEDC(CH_R_B, dr); }

    float lin_l = invertN20Deadzone(out_l);
    float lin_r = clampf(invertN20Deadzone(out_r) / ((RIGHT_MOTOR_TRIM > 0.1f) ? RIGHT_MOTOR_TRIM : 1.0f), -255.0f, 255.0f);
    float log_l = out_l, log_r = out_r;
    if (INVERT_LEFT_MOTOR)  { lin_l = -lin_l; log_l = -log_l; }
    if (INVERT_RIGHT_MOTOR) { lin_r = -lin_r; log_r = -log_r; }
    if (SWAP_MOTORS_LR) { float t1 = lin_l; lin_l = lin_r; lin_r = t1; float t2 = log_l; log_l = log_r; log_r = t2; }
    hw_log_out_l = log_l; hw_log_out_r = log_r; hw_u_eff = 0.5f * (lin_l - lin_r);
}

static inline int readIRFiltered(int pin) {
    (void)analogRead(pin);
    int a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
    int hi = (a > b) ? a : b, lo = (a < b) ? a : b;
    return (c > hi) ? hi : ((c < lo) ? lo : c);
}

#if ENABLE_BLACKBOX
#define BB_N 5000
struct BBRec { uint16_t t10; int16_t x10, vx, uff10, u, pl, pr; uint8_t s[4], mode, pad, cm, rk; };
struct Blackbox {
    static BBRec buf[BB_N]; static uint16_t count; static bool dumped, was_lifted_prev;
    static void clear() { count = 0; dumped = false; }
    static void record(const SensorData& sd, float pl, float pr) {
        if (count >= BB_N) return;
        BBRec& r = buf[count++];
        r.t10   = (uint16_t)clampf(sd.elapsed_sec * 100.0f, 0.0f, 65535.0f);
        r.x10   = (int16_t)clampf(brain.z1 * 10.0f, -32000.0f, 32000.0f);
        r.vx    = (int16_t)clampf(brain.z2, -32000.0f, 32000.0f);
        r.uff10 = (int16_t)clampf((brain.z3 / B0_GAIN) * 10.0f, -32000.0f, 32000.0f);
        r.u     = (int16_t)clampf(brain.u_cmd, -32000.0f, 32000.0f);
        r.pl    = (int16_t)roundi(clampf(pl, -255.0f, 255.0f));
        r.pr    = (int16_t)roundi(clampf(pr, -255.0f, 255.0f));
        for (int i = 0; i < 4; i++) r.s[i] = (uint8_t)clampf(sd.ir[i] * 255.0f, 0.0f, 255.0f);
        r.mode = (uint8_t)brain.mode; r.pad = (uint8_t)brain.cross_count;
        r.cm   = (uint8_t)clampf(floor_cm * 255.0f, 0.0f, 255.0f);
        r.rk   = (uint8_t)clampf(brain.risk_dbg * 255.0f, 0.0f, 255.0f);
    }
    static void dump() {
        if (count < 10) return;
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(50);
#endif
        Serial.println("\n=== BLACKBOX v7.2 DUMP START ===");
        Serial.println("t,x_mm,vx,uff,u,pl,pr,s0,s1,s2,s3,mode,cross_count,floor_cm,risk");
        for (uint16_t i = 0; i < count; i++) {
            const BBRec& r = buf[i];
            Serial.printf("%.2f,%.1f,%d,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n",
                          r.t10 * 0.01f, r.x10 * 0.1f, (int)r.vx, r.uff10 * 0.1f, (int)r.u, (int)r.pl, (int)r.pr,
                          (int)r.s[0], (int)r.s[1], (int)r.s[2], (int)r.s[3],
                          (int)r.mode, (int)r.pad, r.cm / 255.0f, r.rk / 255.0f);
            if ((i & 0x0F) == 0) delay(1);
        }
        Serial.println("=== BLACKBOX v7.2 DUMP END (go 'd' de xa lai) ===");
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(0);
#endif
    }
};
BBRec Blackbox::buf[BB_N]; uint16_t Blackbox::count = 0; bool Blackbox::dumped = false, Blackbox::was_lifted_prev = false;
#endif

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
    WiFi.mode(WIFI_OFF); btStop(); analogReadResolution(12);
#endif
#if ENABLE_SERIAL_TELEMETRY || ENABLE_BLACKBOX
    Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
#endif
    for (int i = 0; i < 4; i++) pinMode(PIN_IR[i], INPUT);
    setupMotorsHardware();
    setMotorHardware(0.0f, 0.0f, 0.002f);

    unsigned long t0 = millis();
    while (millis() - t0 < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]), r3 = readIRFiltered(PIN_IR[3]);
        bool ok0 = (IR_POL < 0) ? (r0 > ADC_RAW_LO_OK + 350) : (r0 < ADC_RAW_HI_OK - 350);
        bool ok3 = (IR_POL < 0) ? (r3 > ADC_RAW_LO_OK + 350) : (r3 < ADC_RAW_HI_OK - 350);
        if (ok0 || ok3) break;
        delay(8);
    }
    t0 = millis();
    int prev_c[4] = {readIRFiltered(PIN_IR[0]), readIRFiltered(PIN_IR[1]), readIRFiltered(PIN_IR[2]), readIRFiltered(PIN_IR[3])};
    while (millis() - t0 < 90) {
        delay(12); int md = 0;
        for (int i = 0; i < 4; i++) { int cur = readIRFiltered(PIN_IR[i]); md = max(md, abs(cur - prev_c[i])); prev_c[i] = cur; }
        if (md < 100) break;
    }
    int bright_seen[4], dark_seen[4];
    for (int i = 0; i < 4; i++) bright_seen[i] = dark_seen[i] = readIRFiltered(PIN_IR[i]);
    for (int s = 0; s < 20; s++) {
        for (int i = 0; i < 4; i++) {
            int v = readIRFiltered(PIN_IR[i]);
            bright_seen[i] = brighterOf(bright_seen[i], v); dark_seen[i] = darkerOf(dark_seen[i], v);
        }
        delay(3);
    }
    int g_white = bright_seen[0], g_dark = dark_seen[0];
    for (int i = 1; i < 4; i++) { g_white = brighterOf(g_white, bright_seen[i]); g_dark = darkerOf(g_dark, dark_seen[i]); }
    if ((IR_POL < 0) ? (g_white <= ADC_RAW_LO_OK + 200) : (g_white >= ADC_RAW_HI_OK - 200)) {
        g_white = ADC_WHITE_NOM; g_dark = ADC_BLACK_NOM;
        for (int i = 0; i < 4; i++) dark_seen[i] = ADC_BLACK_NOM;
    }
    adc_white_anchor = g_white;
    int init_span = darker(g_dark, g_white);
    bool has_line = (init_span >= INIT_CONTRAST_SPAN && g_dark > ADC_RAW_LO_OK && g_dark < ADC_RAW_HI_OK);
    if (!has_line) { g_dark = constrain(g_white + IR_POL * 1500, ADC_RAW_LO_OK + 50, ADC_RAW_HI_OK - 50); init_span = abs(g_dark - g_white); }
    track_span_est = constrain(init_span, INIT_CONTRAST_SPAN, 2800);
    int w_tol = has_line ? min(init_span / 3, 850) : 950;
    for (int i = 0; i < 4; i++) {
        ir_white_cal[i] = (darker(bright_seen[i], g_white) < w_tol) ? bright_seen[i] : g_white;
        int dk = dark_seen[i];
        ir_black_cal[i] = (has_line && darker(dk, ir_white_cal[i]) >= INIT_CONTRAST_SPAN && dk > ADC_RAW_LO_OK && dk < ADC_RAW_HI_OK)
                          ? dk : darkerOf(g_dark, ir_white_cal[i] + IR_POL * track_span_est);
    }
    brain.reset(); floor_cm = 0.0f; local_peak = 1.0f;
    bemf_l = bemf_r = prev_hw_out_l = prev_hw_out_r = hw_log_out_l = hw_log_out_r = hw_u_eff = 0.0f;
    startMicros = lastStepMicros = micros();
}

void loop() {
    unsigned long nowMicros = micros(), diffMicros = nowMicros - lastStepMicros;
    if (diffMicros < 2000) return;
    lastStepMicros = nowMicros;
    float dt = (float)diffMicros * 1e-6f;
    if (dt > 0.025f) dt = 0.002f;
    loopFrameCount++;

    SensorData sData;
    sData.elapsed_sec = (float)(nowMicros - startMicros) * 1e-6f;
    int frame_min_raw = 4095, frame_max_raw = 0;
    for (int i = 0; i < 4; i++) {
        int raw = readIRFiltered(PIN_IR[i]);
        sData.raw_ir[i] = raw;
        if (raw < frame_min_raw) frame_min_raw = raw;
        if (raw > frame_max_raw) frame_max_raw = raw;
    }
    int frame_span = frame_max_raw - frame_min_raw;
    sData.frame_span = frame_span;
    int frame_bright = (IR_POL > 0) ? frame_min_raw : frame_max_raw;
    bool raw_in_range = (IR_POL < 0) ? (frame_min_raw > ADC_RAW_LO_OK) : (frame_max_raw < ADC_FRAME_HI_OK);
    bool slow_tick = ((loopFrameCount & 0x1F) == 0);

    if (raw_in_range && frame_span > MIN_CONTRAST_SPAN) {
        if (darker(frame_bright, adc_white_anchor) < 0) adc_white_anchor = (adc_white_anchor * 3 + frame_bright) / 4;
        else if (slow_tick && darker(frame_bright, adc_white_anchor) > 150) {
            if ((IR_POL < 0) ? (frame_bright > ADC_RAW_LO_OK + 350) : (frame_bright < ADC_RAW_HI_OK - 350)) adc_white_anchor += IR_POL;
        }
    }
    bool valid_contrast = (frame_span > MIN_CONTRAST_SPAN) && (darker(frame_bright, adc_white_anchor) < WHITE_SEEN_MAX_DARK) && raw_in_range;
    bool wing_busy = brain.w_on[0] || brain.w_on[1] || brain.w_rose[0] || brain.w_rose[1] || brain.on_crossbar || (brain.mode != Brain::M_RUN);

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
        float n_min = min4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]), n_max = max4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
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
    for (int i = 0; i < 4; i++) sData.ir[i] = clampf((n_raw[i] - floor_cm) / denom, 0.0f, 1.0f);

    float pwm_l = 0.0f, pwm_r = 0.0f;
    brain.step(sData, dt, hw_log_out_l, hw_log_out_r, hw_u_eff, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r, dt);

#if ENABLE_SERIAL_TELEMETRY
    if ((loopFrameCount % 25) == 0 && !brain.was_lifted && Serial) {
        Serial.printf("x=%.1f vx=%.0f uff=%.1f u=%.1f v=%.0f m=%d\n",
                      brain.z1, brain.z2, brain.z3 / B0_GAIN, brain.u_cmd, brain.v_cmd, (int)brain.mode);
    }
#endif

#if ENABLE_BLACKBOX
    if (!brain.was_lifted) {
        if (Blackbox::was_lifted_prev) { Blackbox::clear(); Blackbox::was_lifted_prev = false; }
        if (brain.has_seen_line && (loopFrameCount % 10) == 0) Blackbox::record(sData, pwm_l, pwm_r);
    } else {
        Blackbox::was_lifted_prev = true;
        if (!Blackbox::dumped && Blackbox::count >= 10 && Serial) { Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros(); }
        while (Serial.available() > 0) {
            int ch = Serial.read();
            if (ch == 'd' || ch == 'D') { Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros(); }
        }
    }
#endif
}
