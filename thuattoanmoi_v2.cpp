
// CODE THUẬT TOÁN MỚI V2

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

#define TURN_RIGHT (+1)
#define TURN_LEFT  (-1)
static constexpr int8_t CROSS_PLAN[]            = { 0, 0, TURN_RIGHT };
static constexpr int    CROSS_PLAN_LEN          = (int)(sizeof(CROSS_PLAN) / sizeof(CROSS_PLAN[0]));
static constexpr float  CROSS_MIN_DIST_TABLE[4] = { 350.0f, 700.0f, 1100.0f, 500.0f };

static constexpr float L_LOOK_MM           = 60.0f;
static constexpr float SENSOR_X_MM[4]      = { -20.0f, -11.25f, +11.25f, +20.0f };
static constexpr float SENSOR_VORONOI_W[4] = { 8.75f, 15.625f, 15.625f, 8.75f };
static constexpr float DEFAULT_LINE_W_MM   = 24.0f;
static constexpr float K_V_MMPS_PER_PWM    = 2.1176f;
static constexpr float K_OMEGA_RAD_PER_PWM = 0.0314f;
static constexpr float TAU_MOTOR_S         = 0.042f;
static constexpr float X_LIM_MM            = 30.0f;

static constexpr float OMEGA_O             = 55.0f;
static constexpr float OMEGA_C0            = 13.5f;
static constexpr float B0_GAIN             = 50.0f;
static constexpr float ZETA_C              = 0.92f;
static constexpr float E_GATE_MM           = 5.5f;
static constexpr float Z3_MAX              = B0_GAIN * 127.5f;

static constexpr float V_MAX               = 255.0f;
static constexpr float V_BRAKE_MIN         = 115.0f;
static constexpr float K_CURV              = 1.20f;
static constexpr float RHO_DEAD            = 0.10f;
static constexpr float CURV_TAU_UP         = 0.012f;
static constexpr float CURV_TAU_DN         = 0.045f;
static constexpr float RISK_T_PRED         = 0.035f;
static constexpr float RISK_X0_MM          = 11.0f;
static constexpr float RISK_SPAN_MM        = 10.0f;
static constexpr float RISK_V_DROP         = 120.0f;
static constexpr float RISK_TAU_UP         = 0.012f;
static constexpr float RISK_TAU_DN         = 0.050f;
static constexpr float ACC_UP_PWM_S        = 2200.0f;
static constexpr float F_IN_BASE           = 45.0f;

static constexpr uint32_t PWM_FREQ_HZ      = 5000;
static constexpr uint8_t  PWM_RES_BITS     = 10;
static constexpr int      PWM_MAX_DUTY     = 1023;
static constexpr float D0_DZ               = 36.0f;
static constexpr float P_KNEE              = 10.0f;
static constexpr float C_LIN               = (255.0f - D0_DZ) / 255.0f;
static constexpr float BEMF_SINGLE_MAX_DV  = 150.0f;
static constexpr float BEMF_BAT_BUDGET     = 220.0f;
static constexpr float HBRIDGE_SUM_BUDGET  = 280.0f;

static constexpr float GAP_X_MM            = 11.5f;
static constexpr float GAP_XPRED_T         = 0.025f;
static constexpr float GAP_XPRED_MM        = 15.0f;
static constexpr float GAP_HOLD_MM         = 82.0f;
static constexpr float V_GAP_STRAIGHT      = 240.0f;
static constexpr float V_GAP_CURVE         = 200.0f;
static constexpr float V_LOST              = 165.0f;
static constexpr float U_LOST_MAX          = 135.0f;
static constexpr float LOST_RAMP_S         = 0.090f;
static constexpr float LOST_GIVEUP_S       = 1.25f;
static constexpr float LOST_PRED_T         = 0.028f;
static constexpr float F_IN_LOST           = -15.0f;

static constexpr float WING_ON               = 0.52f;
static constexpr float WING_OFF              = 0.34f;
static constexpr float WING_BASE_MAX         = 0.45f;
static constexpr float WING_DELTA_MIN        = 0.26f;
static constexpr int   WING_DEBOUNCE_TICKS   = 3;
static constexpr int   CROSS_CONFIRM_TICKS   = 2;
static constexpr int   CROSS_CONFIRM_SKEW    = 3;
static constexpr float WING_MIN_CHORD_MM     = 5.0f;
static constexpr float WING_LATCH_MM         = 60.0f;
static constexpr float CROSS_PASS_MAX_MM     = 190.0f;
static constexpr float CROSS_PASS_MAX_S      = 0.70f;
static constexpr float CROSS_EXIT_MARGIN_MM  = 14.0f;
static constexpr float U_REF_TAU             = 0.18f;

static constexpr float V_ALIGN               = 80.0f;
static constexpr float ARC_ALIGN_MM          = 0.0f;
static constexpr float ALIGN_FALLBACK_MM     = 25.0f;
static constexpr float ALIGN_FALLBACK_S      = 0.12f;
static constexpr float P_ARC_OUTER           = 205.0f;
static constexpr float P_ARC_INNER_CLEAR     = 0.0f;
static constexpr float P_ARC_INNER_TIGHT     = -105.0f;
static constexpr float ARC_TIGHT_T           = 0.020f;
static constexpr float P_APPROACH_OUTER      = 135.0f;
static constexpr float P_APPROACH_INNER      = -45.0f;
static constexpr float PIVOT_OLD_ON          = 0.48f;
static constexpr float PIVOT_OLD_GONE        = 0.26f;
static constexpr float PIVOT_MIN_T           = 0.110f;
static constexpr float PIVOT_RELAX_T         = 0.20f;
static constexpr float T_PIVOT_MAX           = 1.00f;
static constexpr float HANDOVER_T            = 0.22f;

static constexpr float CM_DEAD               = 0.05f;
static constexpr float CM_MAX                = 0.72f;
static constexpr float CM_TAU_UP             = 0.035f;
static constexpr float CM_TAU_DN             = 0.028f;
static constexpr float CM_ABSORB_MIN         = 0.05f;
static constexpr float CM_ABSORB_RATE        = 0.04f;
static constexpr float PEAK_TARGET           = 0.88f;
static constexpr float PEAK_MIN              = 0.30f;
static constexpr float PEAK_TAU_UP           = 0.020f;
static constexpr float PEAK_TAU_DN           = 0.120f;

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
static constexpr int IR_POL        = -1;
static constexpr int ADC_WHITE_NOM = 2500;
static constexpr int ADC_BLACK_NOM = 500;
#else
static constexpr int IR_POL        = +1;
static constexpr int ADC_WHITE_NOM = 300;
static constexpr int ADC_BLACK_NOM = 2400;
#endif

static constexpr int ADC_RAW_LO_OK        = 20;
static constexpr int ADC_RAW_HI_OK        = 3850;
static constexpr int ADC_FRAME_HI_OK      = 3950;
static constexpr int LINE_FRAME_SPAN_MIN  = 140;
static constexpr int CM_SPAN_MIN          = 240;
static constexpr int MIN_CONTRAST_SPAN    = 360;
static constexpr int INIT_CONTRAST_SPAN   = 900;
static constexpr int EFF_SPAN_MIN         = 320;
static constexpr int WHITE_SEEN_MAX_DARK  = 2000;
static constexpr int WHITE_DRIFT_MAX_DARK = 1200;
static constexpr int WHITE_CAL_MAX_DARK   = 1600;

static constexpr int   PIN_IR[4]          = { SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN };
static constexpr float RIGHT_MOTOR_TRIM   = 1.00f;
static constexpr bool  INVERT_LEFT_MOTOR  = false;
static constexpr bool  INVERT_RIGHT_MOTOR = false;
static constexpr bool  SWAP_MOTORS_LR     = false;

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }
static inline float signf(float v) { return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f); }
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

static inline void mixWheels(float v_target, float u_raw, float f_in,
                             float& p_lin_l, float& p_lin_r, float& u_cmd_out) {
    float u_max = (255.0f - f_in) * 0.5f;
    float u_cmd = clampf(u_raw, -u_max, +u_max);
    float v_mix = clampf(v_target, f_in + fabsf(u_cmd), 255.0f - fabsf(u_cmd));
    p_lin_l   = v_mix + u_cmd;
    p_lin_r   = v_mix - u_cmd;
    u_cmd_out = u_cmd;
}

static inline float applyDeadzoneCInf(float p) {
    if (fabsf(p) < 1e-6f) return 0.0f;
    float a = clampf(fabsf(p), 0.0f, 255.0f);
    float y = clampf(D0_DZ * tanhf(a / P_KNEE) + C_LIN * a, 0.0f, 255.0f);
    return (p >= 0.0f) ? y : -y;
}

static inline float invDeadzone(float y, float p_tgt, float p_lin_orig) {
    if (fabsf(y - p_tgt) < 1e-3f) return p_lin_orig;
    if (fabsf(y) < 1e-4f) return 0.0f;
    float a = fabsf(y);
    float x = max2f((a - D0_DZ) / C_LIN, a / (D0_DZ / P_KNEE + C_LIN));
    for (int k = 0; k < 3; ++k) {
        float t  = tanhf(x / P_KNEE);
        float f  = D0_DZ * t + C_LIN * x - a;
        float df = (D0_DZ / P_KNEE) * (1.0f - t * t) + C_LIN;
        x = max2f(0.0f, x - f / df);
    }
    return (y >= 0.0f) ? x : -x;
}

class Brain {
public:
    enum Mode : uint8_t { M_RUN = 0, M_CROSS_PASS, M_TURN_ALIGN, M_TURN_PIVOT };
    Mode  mode;
    float mode_t, mode_d;

    float z1, z2, z3;
    float z1_hat, z2_hat, z3_hat;
    float x, vx;
    bool  eso_initialized, cross_exit_reinit;

    float m_curv, rho_hat, risk_f, risk_dbg;
    float v_target, u_cmd;

    float u_eff, v_lin_l, v_lin_r;
    float v_est_pwm, v_est, omega_est;
    float p_prev_l, p_prev_r;

    float dist_mm, v_scale;
    bool  v_cal_valid;
    float line_w_est;
    int   last_side;

    bool  in_loss, meas_ok, is_gap_loss, on_crossbar;
    int   loss_ticks;
    float lost_t, lost_d, lost_x;
    int   lost_side;
    float handover_t;
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
    float pivot_prev_z;
    bool  ev_cross, ev_align_end, ev_pivot_end;
    int   ev_idx, ev_act;
    float ev_center_d, ev_al_t, ev_al_d, ev_al_rem, ev_pv_t, ev_pv_z;
    uint8_t ev_al_wf;

    Brain() { reset(); }

    void reset() {
        mode = M_RUN; mode_t = mode_d = 0.0f;
        z1 = z2 = z3 = 0.0f;
        z1_hat = z2_hat = z3_hat = 0.0f;
        x = vx = 0.0f;
        eso_initialized = cross_exit_reinit = false;
        m_curv = rho_hat = risk_f = risk_dbg = 0.0f;
        v_target = u_cmd = 0.0f;
        u_eff = v_lin_l = v_lin_r = 0.0f;
        v_est_pwm = v_est = omega_est = 0.0f;
        p_prev_l = p_prev_r = 0.0f;
        dist_mm = 0.0f; v_scale = 1.0f; v_cal_valid = false;
        line_w_est = DEFAULT_LINE_W_MM; last_side = 1;
        in_loss = meas_ok = is_gap_loss = on_crossbar = false;
        loss_ticks = 0; lost_t = lost_d = lost_x = 0.0f; lost_side = 1;
        handover_t = 0.0f;
        was_lifted = has_seen_line = false;
        off_track_t = all_black_t = air_t = run_t = 0.0f;
        base_init = false; base_o[0] = base_o[1] = 0.0f;
        cross_confirm = crossbar_ticks = cross_count = clear_ticks = white_ticks = 0;
        last_cross_dist = -1.0e6f; turn_dir = 1;
        pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
        pivot_prev_z = 0.0f;
        cross_sec = 1.0f; center_odo = end_odo = align_rem_mm = 0.0f; center_refined = false;
        u_ref = u_hold = 0.0f;
        ev_cross = ev_align_end = ev_pivot_end = false;
        ev_idx = ev_act = 0;
        ev_center_d = ev_al_t = ev_al_d = ev_al_rem = ev_pv_t = ev_pv_z = 0.0f;
        ev_al_wf = 0;
        clearWings();
    }

    static inline bool isCrossbar(const float* s) {
        return (min4f(s[0], s[1], s[2], s[3]) >= 0.46f) ||
               (s[0] >= 0.52f && s[3] >= 0.52f && max2f(s[1], s[2]) >= 0.42f);
    }

    static inline bool hasLineContrast(const float* s, int frame_span) {
        float s_min = min4f(s[0], s[1], s[2], s[3]);
        float s_max = max4f(s[0], s[1], s[2], s[3]);
        return ((s_max - s_min) >= 0.14f) && (frame_span >= LINE_FRAME_SPAN_MIN);
    }

    static bool measure(const SensorData& sd, float z_hint, bool tracking_valid, float& z_meas) {
        const float* s = sd.ir;
        float s_min   = min4f(s[0], s[1], s[2], s[3]);
        float s_max   = max4f(s[0], s[1], s[2], s[3]);
        float delta_s = s_max - s_min;
        if (!(delta_s >= 0.14f && sd.frame_span >= LINE_FRAME_SPAN_MIN)) return false;

        float r[4];
        float r_max = 0.0f;
        for (int i = 0; i < 4; ++i) {
            r[i] = max2f(0.0f, s[i] - s_min - 0.08f * delta_s);
            if (r[i] > r_max) r_max = r[i];
        }
        if (r_max <= 1e-6f) return false;

        int p = 0;
        if (!tracking_valid) {
            for (int i = 1; i < 4; ++i) {
                if (r[i] > r[p]) p = i;
            }
        } else {
            float best_dist = 1.0e9f;
            int   best_p    = -1;
            for (int k = 0; k < 4; ++k) {
                bool is_local_peak = (r[k] >= 0.40f * r_max) &&
                                     (k == 0 || r[k] >= r[k - 1] - 0.04f * delta_s) &&
                                     (k == 3 || r[k] >= r[k + 1] - 0.04f * delta_s);
                bool near_max      = (r[k] >= r_max - 0.05f * delta_s);
                if (is_local_peak || near_max) {
                    float d = fabsf(SENSOR_X_MM[k] - z_hint);
                    if (d < best_dist || (fabsf(d - best_dist) < 1e-4f && best_p >= 0 && r[k] > r[best_p])) {
                        best_dist = d;
                        best_p    = k;
                    }
                }
            }
            if (best_p >= 0) p = best_p;
            else {
                for (int i = 1; i < 4; ++i) {
                    if (r[i] > r[p]) p = i;
                }
            }
        }

        int c_lo = p;
        for (int i = p - 1; i >= 0; --i) {
            if (r[i] > 0.0f && r[i] <= r[i + 1] + 0.04f * delta_s) c_lo = i;
            else break;
        }
        int c_hi = p;
        for (int i = p + 1; i < 4; ++i) {
            if (r[i] > 0.0f && r[i] <= r[i - 1] + 0.04f * delta_s) c_hi = i;
            else break;
        }

        float wsum = 0.0f, xnum = 0.0f;
        for (int i = c_lo; i <= c_hi; ++i) {
            float w = SENSOR_VORONOI_W[i] * r[i];
            wsum += w;
            xnum += w * SENSOR_X_MM[i];
        }
        if (wsum <= 1e-6f) return false;

        float xc = xnum / wsum;
        if (c_lo == 0 && c_hi == 0) {
            float eta_edge = clampf((s[0] - s_min) / max2f(0.25f, 0.90f - s_min), 0.0f, 1.0f);
            float ext      = 10.0f * clampf((0.85f - eta_edge) / 0.70f, 0.0f, 1.0f);
            xc = -20.0f - ext;
        } else if (c_lo == 3 && c_hi == 3) {
            float eta_edge = clampf((s[3] - s_min) / max2f(0.25f, 0.90f - s_min), 0.0f, 1.0f);
            float ext      = 10.0f * clampf((0.85f - eta_edge) / 0.70f, 0.0f, 1.0f);
            xc = +20.0f + ext;
        }

        z_meas = clampf(xc, -X_LIM_MM, +X_LIM_MM);
        return isfinite(z_meas);
    }

    void step(const SensorData& sd, float dt, float& p_out_l, float& p_out_r) {
        p_out_l = 0.0f;
        p_out_r = 0.0f;
        dt = clampf(dt, 0.001f, 0.004f);
        const float* s = sd.ir;

        int max_raw = sd.raw_ir[0], min_raw = sd.raw_ir[0];
        for (int i = 1; i < 4; ++i) {
            if (sd.raw_ir[i] > max_raw) max_raw = sd.raw_ir[i];
            if (sd.raw_ir[i] < min_raw) min_raw = sd.raw_ir[i];
        }
        bool in_air = (IR_POL < 0) ? (max_raw <= ADC_RAW_LO_OK + 60) : (min_raw >= ADC_FRAME_HI_OK);
        air_t = in_air ? (air_t + dt) : 0.0f;

        float min_ir   = min4f(s[0], s[1], s[2], s[3]);
        bool real_line = hasLineContrast(s, sd.frame_span);
        bool crossbar  = isCrossbar(s);
        on_crossbar    = crossbar;
        bool all_black = (min_ir > 0.86f) && (sd.frame_span < 420 || min_ir > 0.93f);
        bool no_line   = !all_black && !real_line && !crossbar;
        bool spinning  = (mode == M_TURN_PIVOT);
        bool in_cross  = (mode == M_CROSS_PASS || mode == M_TURN_ALIGN);

        if (air_t > 0.10f) {
            was_lifted = true;
        } else if (was_lifted) {
            if (real_line && !crossbar && min_ir < 0.65f && !in_air) { reset(); has_seen_line = true; }
        } else if (!has_seen_line) {
            if (real_line && min_ir < 0.75f && !in_air) { has_seen_line = true; off_track_t = all_black_t = 0.0f; }
            else off_track_t += dt;
        } else {
            if (all_black && !spinning && !in_cross) {
                all_black_t += dt;
                if (all_black_t > 0.28f) was_lifted = true;
            } else all_black_t = 0.0f;
            if (no_line && !spinning && !in_cross) {
                off_track_t += dt;
                if (off_track_t > LOST_GIVEUP_S) was_lifted = true;
            } else off_track_t = 0.0f;
        }

        if (was_lifted || !has_seen_line) {
            stopOutputs(dt, p_out_l, p_out_r);
            return;
        }

        run_t   += dt;
        dist_mm += max2f(0.0f, v_est) * v_scale * dt;

        if (mode == M_RUN) runEstimate(sd, dt);
        if (was_lifted) {
            stopOutputs(dt, p_out_l, p_out_r);
            return;
        }

        float p_lin_l = 0.0f, p_lin_r = 0.0f;
        switch (mode) {
            case M_RUN:        outRun(dt, p_lin_l, p_lin_r);          break;
            case M_CROSS_PASS: outCrossPass(s, dt, p_lin_l, p_lin_r); break;
            case M_TURN_ALIGN: outAlign(s, dt, p_lin_l, p_lin_r);     break;
            case M_TURN_PIVOT: outPivot(sd, dt, p_lin_l, p_lin_r);    break;
        }
        applyActuatorAndSync(p_lin_l, p_lin_r, dt, p_out_l, p_out_r);
    }

private:
    void stopOutputs(float dt, float& p_out_l, float& p_out_r) {
        z1 = z2 = z3 = 0.0f;
        z1_hat = z2_hat = z3_hat = 0.0f;
        x = vx = 0.0f;
        eso_initialized = cross_exit_reinit = false;
        m_curv = rho_hat = risk_f = risk_dbg = 0.0f;
        v_target = u_cmd = u_eff = 0.0f;
        p_prev_l = p_prev_r = p_out_l = p_out_r = 0.0f;

        float alpha_m = expf(-dt / TAU_MOTOR_S);
        v_lin_l *= alpha_m;
        v_lin_r *= alpha_m;
        if (fabsf(v_lin_l) < 0.05f) v_lin_l = 0.0f;
        if (fabsf(v_lin_r) < 0.05f) v_lin_r = 0.0f;
        v_est_pwm = (v_lin_l + v_lin_r) * 0.5f;
        v_est     = K_V_MMPS_PER_PWM * v_est_pwm;
        omega_est = K_OMEGA_RAD_PER_PWM * (v_lin_l - v_lin_r) * 0.5f;
    }

    inline void esoPredict(float dt) {
        float acc_pred = z3 - B0_GAIN * u_eff;
        z1_hat = z1 + dt * z2 + 0.5f * dt * dt * acc_pred;
        z2_hat = z2 + dt * acc_pred;
        z3_hat = z3;
    }

    inline void esoCorrect(float z_meas, float dt) {
        float gamma = expf(-OMEGA_O * dt);
        float omg   = 1.0f - gamma;
        float L1    = 1.0f - gamma * gamma * gamma;
        float L2    = (1.5f / dt) * omg * omg * (1.0f + gamma);
        float L3    = (omg * omg * omg) / (dt * dt);

        float eo       = z_meas - z1_hat;
        float eo_tilde = E_GATE_MM * tanhf(eo / E_GATE_MM);

        z1 = z1_hat + L1 * eo_tilde;
        z2 = z2_hat + L2 * eo_tilde;
        z3 = clampf(z3_hat + L3 * eo_tilde, -Z3_MAX, +Z3_MAX);
        if (v_est < 80.0f) z3 *= 0.98f;

        if (!isfinite(z1) || !isfinite(z2) || !isfinite(z3)) {
            z1 = z_meas; z2 = 0.0f; z3 = 0.0f;
        }
        x  = z1;
        vx = z2;
    }

    void applyActuatorAndSync(float p_lin_l, float p_lin_r, float dt, float& p_out_l, float& p_out_r) {
        p_lin_l = clampf(p_lin_l, -255.0f, +255.0f);
        p_lin_r = clampf(p_lin_r, -255.0f, +255.0f);

        float p_tgt_l = applyDeadzoneCInf(p_lin_l);
        float p_tgt_r = applyDeadzoneCInf(p_lin_r);

        float bemf_l = C_LIN * v_lin_l;
        float bemf_r = C_LIN * v_lin_r;

        float dV_l = clampf(p_tgt_l - bemf_l, -BEMF_SINGLE_MAX_DV, +BEMF_SINGLE_MAX_DV);
        float dV_r = clampf(p_tgt_r - bemf_r, -BEMF_SINGLE_MAX_DV, +BEMF_SINGLE_MAX_DV);

        float i_bat_l   = (fabsf(p_tgt_l) > 1e-3f) ? max2f(0.0f, signf(p_tgt_l) * dV_l) : 0.0f;
        float i_bat_r   = (fabsf(p_tgt_r) > 1e-3f) ? max2f(0.0f, signf(p_tgt_r) * dV_r) : 0.0f;
        float i_bat_sum = i_bat_l + i_bat_r;
        if (i_bat_sum > BEMF_BAT_BUDGET) {
            float s_bat = BEMF_BAT_BUDGET / i_bat_sum;
            if (i_bat_l > 0.0f) dV_l *= s_bat;
            if (i_bat_r > 0.0f) dV_r *= s_bat;
        }

        float hb_sum = fabsf(dV_l) + fabsf(dV_r);
        if (hb_sum > HBRIDGE_SUM_BUDGET) {
            float s_hb = HBRIDGE_SUM_BUDGET / hb_sum;
            dV_l *= s_hb;
            dV_r *= s_hb;
        }

        p_out_l = clampf(bemf_l + dV_l, -255.0f, +255.0f);
        p_out_r = clampf(bemf_r + dV_r, -255.0f, +255.0f);

        if (p_prev_l * p_out_l < 0.0f) p_out_l = 0.0f;
        if (p_prev_r * p_out_r < 0.0f) p_out_r = 0.0f;
        if (!isfinite(p_out_l)) p_out_l = 0.0f;
        if (!isfinite(p_out_r)) p_out_r = 0.0f;
        p_prev_l = p_out_l;
        p_prev_r = p_out_r;

        float p_eff_lin_l = invDeadzone(p_out_l, p_tgt_l, p_lin_l);
        float p_eff_lin_r = invDeadzone(p_out_r, p_tgt_r, p_lin_r);
        u_eff = (p_eff_lin_l - p_eff_lin_r) * 0.5f;

        float alpha_m = expf(-dt / TAU_MOTOR_S);
        v_lin_l = alpha_m * v_lin_l + (1.0f - alpha_m) * p_eff_lin_l;
        v_lin_r = alpha_m * v_lin_r + (1.0f - alpha_m) * p_eff_lin_r;
        if (!isfinite(v_lin_l) || !isfinite(v_lin_r) || !isfinite(u_eff)) {
            v_lin_l = v_lin_r = u_eff = 0.0f;
        }
        v_est_pwm = (v_lin_l + v_lin_r) * 0.5f;
        v_est     = K_V_MMPS_PER_PWM * v_est_pwm;
        omega_est = K_OMEGA_RAD_PER_PWM * (v_lin_l - v_lin_r) * 0.5f;
    }

    void updateWingBase(const float* s, float dt) {
        if (!base_init) { base_o[0] = s[0]; base_o[1] = s[3]; base_init = true; }
        const float ab = dt / (0.15f + dt);
        base_o[0] += ((s[0] < 0.45f) ? ab : ab * 0.125f) * (s[0] - base_o[0]);
        base_o[1] += ((s[3] < 0.45f) ? ab : ab * 0.125f) * (s[3] - base_o[1]);
    }

    void clearWings() {
        for (int k = 0; k < 2; ++k) {
            w_on[k] = w_rose[k] = w_fell[k] = false; w_ticks[k] = 0;
            w_cand_d[k] = w_rise_d[k] = w_fall_d[k] = w_half_fall_d[k] = w_last_d[k] = dist_mm;
        }
        w_sep = false; cross_confirm = 0; crossbar_ticks = 0;
    }

    void trackWings(const float* s, float dt) {
        const int idx[2] = { 0, 3 };
        for (int k = 0; k < 2; ++k) {
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

        bool skew_ok     = fabsf(w_rise_d[1] - w_rise_d[0]) <= (v_cal_valid ? 60.0f : 75.0f);
        bool center      = (s[1] > 0.30f || s[2] > 0.30f || (s[0] >= WING_ON && s[3] >= WING_ON));
        float c0         = (w_on[0] ? dist_mm : w_half_fall_d[0]) - w_rise_d[0];
        float c1         = (w_on[1] ? dist_mm : w_half_fall_d[1]) - w_rise_d[1];
        bool both_strong = (s[0] >= WING_ON && s[3] >= WING_ON && (s[1] >= 0.38f || s[2] >= 0.38f));
        bool both_chord  = (max2f(c0, c1) >= WING_MIN_CHORD_MM) && (min2f(c0, c1) >= 3.5f);
        bool trig        = allowed && w_rose[0] && w_rose[1] && w_sep && center && skew_ok && (both_strong || both_chord);
        cross_confirm    = trig ? (cross_confirm + 1) : 0;
        return cross_confirm >= (both_strong ? CROSS_CONFIRM_TICKS : CROSS_CONFIRM_SKEW);
    }

    void planCross() {
        float t    = min2f(fabsf(w_rise_d[1] - w_rise_d[0]) / (SENSOR_X_MM[3] - SENSOR_X_MM[0]), 3.0f);
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
        if (!(L0 > L_min_ok && L1 > L_min_ok && L0 < L_max_ok && L1 < L_max_ok &&
              fabsf(L0 - L1) < 0.45f * max2f(L0, L1))) return;
        float L_meas    = (fabsf(L0 - L1) > 0.20f * max2f(L0, L1)) ? min2f(L0, L1) : (0.5f * (L0 + L1));
        center_odo      = 0.25f * (w_rise_d[0] + w_half_fall_d[0] + w_rise_d[1] + w_half_fall_d[1]);
        float skew_term = (line_w_est * fabsf(w_rise_d[1] - w_rise_d[0])) / (SENSOR_X_MM[3] - SENSOR_X_MM[0]);
        float denom_r   = sqrtf(max2f(L_meas * L_meas - skew_term * skew_term, 0.25f * L_meas * L_meas));
        float r         = clampf(line_w_est / max2f(denom_r, 1.0f), 0.70f, 1.35f);
        float v_old     = v_scale;
        v_scale         = clampf(v_old + (!v_cal_valid ? 0.65f : 0.35f) * (clampf(v_old * r, 0.70f, 1.35f) - v_old), 0.70f, 1.35f);
        v_cal_valid     = true;
    }

    void enterCross() {
        int idx = cross_count++;
        int act = 0;
#if ENABLE_SHORTCUT
        if (idx < CROSS_PLAN_LEN) act = CROSS_PLAN[idx];
#endif
        for (int k = 0; k < 2; ++k) {
            if (!w_rose[k]) {
                w_on[k] = w_rose[k] = true; w_fell[k] = false;
                w_rise_d[k] = (w_ticks[k] > 0) ? w_cand_d[k] : dist_mm;
                w_half_fall_d[k] = w_last_d[k] = dist_mm;
            }
        }
        last_cross_dist = dist_mm;
        u_hold          = clampf(u_ref, -45.0f, +45.0f);
        clear_ticks = white_ticks = cross_confirm = crossbar_ticks = 0;
        mode_t = mode_d = 0.0f;
        planCross();
        align_rem_mm = ARC_ALIGN_MM;
        ev_cross = true; ev_idx = idx + 1; ev_act = act; ev_center_d = center_odo;
        if (act == 0) {
            mode = M_CROSS_PASS;
        } else {
            turn_dir = (act > 0) ? 1 : -1;
            if (ARC_ALIGN_MM <= 0.5f) enterPivot();
            else mode = M_TURN_ALIGN;
        }
    }

    void runEstimate(const SensorData& sd, float dt) {
        const float* s = sd.ir;
        bool crossbar  = isCrossbar(s);
        bool allowed   = (handover_t <= 0.05f) && (!in_loss || crossbar) && (run_t > 0.12f) &&
                         ((dist_mm - last_cross_dist) >= getMinCrossSpacing());
        if (allowed) trackWings(s, dt);
        else { clearWings(); updateWingBase(s, dt); }
        bool crossing = detectCross(s, allowed);

        if (crossbar) {
            in_loss = is_gap_loss = meas_ok = false;
            loss_ticks = 0; lost_t = lost_d = 0.0f;
            z1 *= expf(-dt / 0.05f);
            z2 *= expf(-dt / 0.03f);
            x  = z1; vx = z2;
        } else {
            bool can_predict = eso_initialized && !in_loss && !cross_exit_reinit;
            if (can_predict) esoPredict(dt);

            float z_hint = can_predict ? z1_hat : z1;
            float z_meas = 0.0f;
            bool  ok     = measure(sd, z_hint, can_predict, z_meas);
            meas_ok      = ok;

            if (ok) {
                if (in_loss) {
                    bool was_gap         = is_gap_loss;
                    int  prev_loss_ticks = loss_ticks;
                    in_loss = is_gap_loss = false;
                    loss_ticks = 0; lost_t = lost_d = 0.0f;

                    if (prev_loss_ticks >= 3) {
                        z1 = z_meas;
                        z2 = was_gap ? 0.0f : (-L_LOOK_MM * omega_est);
                        x  = z1; vx = z2;
                    } else {
                        esoPredict(dt);
                        esoCorrect(z_meas, dt);
                    }
                    eso_initialized   = true;
                    cross_exit_reinit = false;
                } else if (cross_exit_reinit) {
                    z1 = z_meas;
                    z2 = 0.0f;
                    x  = z1; vx = z2;
                    cross_exit_reinit = false;
                    eso_initialized   = true;
                } else if (!eso_initialized) {
                    z1 = z_meas; z2 = 0.0f; z3 = 0.0f;
                    x  = z1; vx = z2;
                    eso_initialized = true;
                } else {
                    esoCorrect(z_meas, dt);
                }
                if (fabsf(x) > 2.5f) last_side = (x > 0.0f) ? 1 : -1;
            } else {
                if (!in_loss) {
                    in_loss    = true;
                    loss_ticks = 1;
                    lost_t     = 0.0f;
                    lost_d     = 0.0f;
                    is_gap_loss = (fabsf(z1) < GAP_X_MM) && (fabsf(z1 + GAP_XPRED_T * z2) < GAP_XPRED_MM);
                    float pred_loss = z1 + LOST_PRED_T * z2;
                    if (fabsf(pred_loss) > 1e-3f) lost_side = (pred_loss > 0.0f) ? 1 : -1;
                    else if (fabsf(z3) > 1e-3f)   lost_side = (z3 > 0.0f) ? 1 : -1;
                    else                          lost_side = last_side;
                    lost_x = clampf(pred_loss, -X_LIM_MM, +X_LIM_MM);
                } else {
                    loss_ticks++;
                }

                lost_t += dt;
                lost_d += max2f(0.0f, v_est) * v_scale * dt;
                if (is_gap_loss && lost_d >= GAP_HOLD_MM) is_gap_loss = false;

                if (is_gap_loss) {
                    z1 *= expf(-dt / 0.05f);
                    z2 *= expf(-dt / 0.03f);
                    z3 *= expf(-dt / 0.25f);
                } else {
                    if (z3 * (float)lost_side < 0.0f) z3 = 0.0f;
                    else                              z3 *= expf(-dt / 0.50f);
                    z2 *= expf(-dt / 0.05f);
                    z1  = clampf(z1, -X_LIM_MM, +X_LIM_MM);
                }
                x = z1; vx = z2;
                if (lost_t > LOST_GIVEUP_S) was_lifted = true;
            }
        }
        if (crossing) enterCross();
    }

    void outRun(float dt, float& p_lin_l, float& p_lin_r) {
        float m_raw = fabsf(z3 / B0_GAIN);
        float tau_m = (m_raw > m_curv) ? CURV_TAU_UP : CURV_TAU_DN;
        m_curv += (1.0f - expf(-dt / tau_m)) * (m_raw - m_curv);
        rho_hat = m_curv / max2f(v_est_pwm, 60.0f);
        float v_curv = V_MAX / (1.0f + K_CURV * max2f(0.0f, rho_hat - RHO_DEAD));

        float u_raw = 0.0f, v_des = V_MAX, f_in = F_IN_BASE;

        if (on_crossbar && !in_loss) {
            risk_f *= expf(-dt / RISK_TAU_DN);
            u_raw   = clampf(z3 / B0_GAIN, -60.0f, +60.0f);
            v_des   = clampf(v_curv, V_BRAKE_MIN, V_MAX);
            float sigma_tight = max2f(risk_f, clampf((m_curv - 75.0f) / 40.0f, 0.0f, 1.0f));
            f_in    = F_IN_BASE - F_IN_BASE * sigma_tight;
        } else if (!in_loss) {
            float tz      = tanhf(z1 / 12.0f);
            float omega_c = OMEGA_C0 * (1.0f + 0.30f * tz * tz);
            float kp      = omega_c * omega_c;
            float kd      = 2.0f * ZETA_C * omega_c;
            u_raw         = (z3 + kp * z1 + kd * z2) / B0_GAIN;

            float v_out    = max2f(0.0f, signf(z1) * z2);
            float x_pred   = fabsf(z1) + RISK_T_PRED * v_out;
            float risk_raw = clampf((x_pred - RISK_X0_MM) / RISK_SPAN_MM, 0.0f, 1.0f);
            float tau_r    = (risk_raw > risk_f) ? RISK_TAU_UP : RISK_TAU_DN;
            risk_f        += (1.0f - expf(-dt / tau_r)) * (risk_raw - risk_f);

            v_des = clampf(min2f(v_curv, V_MAX - RISK_V_DROP * risk_f), V_BRAKE_MIN, V_MAX);
            float sigma_tight = max2f(risk_f, clampf((m_curv - 75.0f) / 40.0f, 0.0f, 1.0f));
            f_in  = F_IN_BASE - F_IN_BASE * sigma_tight;
        } else if (is_gap_loss) {
            risk_f *= expf(-dt / RISK_TAU_DN);
            float u_ff      = z3 / B0_GAIN;
            u_raw           = clampf(u_ff, -60.0f, +60.0f);
            float v_gap_cap = (fabsf(u_ff) < 20.0f) ? V_GAP_STRAIGHT : V_GAP_CURVE;
            v_des           = clampf(min2f(v_curv, v_gap_cap), V_BRAKE_MIN, V_MAX);
            float sigma_tight = max2f(risk_f, clampf((m_curv - 75.0f) / 40.0f, 0.0f, 1.0f));
            f_in            = F_IN_BASE - F_IN_BASE * sigma_tight;
        } else {
            f_in             = F_IN_LOST;
            float u_max_lost = (255.0f - f_in) * 0.5f;
            float lx         = fabsf(lost_x);
            float u0         = min2f(3.2f * (1.0f + lx / 24.0f) * max2f(lx, 20.0f), U_LOST_MAX);
            u_raw            = (float)lost_side * min2f(u0 + (U_LOST_MAX - u0) * clampf(lost_t / LOST_RAMP_S, 0.0f, 1.0f), u_max_lost);
            v_des            = V_LOST;
            risk_f           = 1.0f;
        }

        if (v_des > v_target) v_target = min2f(v_des, v_target + ACC_UP_PWM_S * dt);
        else                  v_target = v_des;

        mixWheels(v_target, u_raw, f_in, p_lin_l, p_lin_r, u_cmd);
        risk_dbg = risk_f;

        if (handover_t <= 0.0f && !in_loss && !on_crossbar &&
            !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0) {
            u_ref += (u_cmd - u_ref) * (1.0f - expf(-dt / U_REF_TAU));
        }
        if (handover_t > 0.0f) handover_t = max2f(0.0f, handover_t - dt);
    }

    void outCrossPass(const float* s, float dt, float& p_lin_l, float& p_lin_r) {
        mode_t += dt;
        mode_d += max2f(0.0f, v_est) * v_scale * dt;
        z1 *= expf(-dt / 0.05f);
        z2 *= expf(-dt / 0.03f);
        x  = z1; vx = z2;

        trackWings(s, dt);
        refineCenter();
        u_hold *= expf(-dt / 0.10f);
        bool clear = w_fell[0] && w_fell[1] &&
                     (s[0] < clampf(max2f(0.32f, base_o[0] + 0.14f), 0.32f, 0.55f)) &&
                     (s[3] < clampf(max2f(0.32f, base_o[1] + 0.14f), 0.32f, 0.55f));
        clear_ticks = clear ? (clear_ticks + 1) : 0;

        if (V_MAX > v_target) v_target = min2f(V_MAX, v_target + ACC_UP_PWM_S * dt);
        else                  v_target = V_MAX;
        mixWheels(v_target, u_hold, F_IN_BASE, p_lin_l, p_lin_r, u_cmd);

        if ((clear_ticks >= 3 && mode_d > 15.0f) ||
            (((w_fell[0] && w_fell[1]) || mode_d > 75.0f) && dist_mm >= end_odo) ||
            mode_d > CROSS_PASS_MAX_MM || mode_t > CROSS_PASS_MAX_S) {
            mode = M_RUN;
            cross_exit_reinit = true;
            in_loss = is_gap_loss = false;
            loss_ticks = 0; risk_f = risk_dbg = 0.0f;
            lost_t = lost_d = 0.0f;
            last_cross_dist = dist_mm;
            clearWings();
        }
    }

    void outAlign(const float* s, float dt, float& p_lin_l, float& p_lin_r) {
        float ds = max2f(0.0f, v_est) * v_scale * dt;
        mode_t += dt; mode_d += ds; align_rem_mm -= ds;
        z1 *= expf(-dt / 0.05f);
        z2 *= expf(-dt / 0.03f);
        x  = z1; vx = z2;

        trackWings(s, dt);
        if (V_ALIGN > v_target) v_target = min2f(V_ALIGN, v_target + ACC_UP_PWM_S * dt);
        else                    v_target = V_ALIGN;
        u_hold *= expf(-dt / 0.08f);
        mixWheels(v_target, u_hold, F_IN_BASE, p_lin_l, p_lin_r, u_cmd);
        ev_center_d = center_odo;

        if (align_rem_mm <= 0.0f || mode_d >= ARC_ALIGN_MM || mode_d > ALIGN_FALLBACK_MM || mode_t > ALIGN_FALLBACK_S) {
            ev_align_end = true; ev_al_t = mode_t; ev_al_d = mode_d; ev_al_rem = align_rem_mm;
            ev_al_wf = (w_fell[0] ? 1 : 0) | (w_fell[1] ? 2 : 0);
            enterPivot();
        }
    }

    void enterPivot() {
        mode = M_TURN_PIVOT; mode_t = mode_d = 0.0f; white_ticks = 0;
        pivot_old_seen = pivot_has_z = pivot_left_crossbar = pivot_old_in_clear = false;
        pivot_prev_z = 0.0f;
        clearWings();
    }

    void outPivot(const SensorData& sd, float dt, float& p_lin_l, float& p_lin_r) {
        const float* s = sd.ir;
        mode_t += dt;
        float old_out  = (turn_dir > 0) ? s[0] : s[3];
        float old_in   = (turn_dir > 0) ? s[1] : s[2];
        float new_in_s = (turn_dir > 0) ? s[2] : s[1];
        float new_out  = (turn_dir > 0) ? s[3] : s[0];
        float inner_on = (line_w_est < 22.0f) ? 0.25f : 0.38f;

        if (!pivot_left_crossbar && (!isCrossbar(s) || mode_t >= 0.045f)) {
            if (old_out < 0.35f || new_out < 0.35f || mode_t >= 0.045f)
                pivot_left_crossbar = true;
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

        float z_hint_pv = pivot_has_z ? pivot_prev_z : ((float)turn_dir * 12.0f);
        float zp = 0.0f;
        if (old_gone && measure(sd, z_hint_pv, pivot_has_z, zp)) {
            pivot_prev_z = zp;
            pivot_has_z  = true;
        } else {
            pivot_has_z = false;
        }

        if (old_gone && new_in && mode_t >= PIVOT_MIN_T) {
            float z_meas = 0.0f;
            if (!measure(sd, pivot_has_z ? pivot_prev_z : ((float)turn_dir * 12.0f), pivot_has_z, z_meas)) {
                z_meas = (float)turn_dir * 12.0f;
            }
            float v_x_pivot = -L_LOOK_MM * omega_est;

            z1 = z_meas;
            z2 = v_x_pivot;
            z3 = 0.0f;
            x  = z1; vx = z2;
            m_curv   = 0.0f;
            rho_hat  = 0.0f;
            v_target = clampf(v_est_pwm, 0.0f, V_MAX);

            eso_initialized   = true;
            cross_exit_reinit = false;
            in_loss = is_gap_loss = on_crossbar = false;
            loss_ticks = 0; lost_t = lost_d = 0.0f;
            risk_f = risk_dbg = 0.0f;
            handover_t      = HANDOVER_T;
            last_side       = turn_dir;
            last_cross_dist = dist_mm;
            ev_pivot_end = true; ev_pv_t = mode_t; ev_pv_z = x;
            clearWings();
            mode = M_RUN; mode_t = 0.0f;

            outRun(dt, p_lin_l, p_lin_r);
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
        if (turn_dir > 0) { p_lin_l = p_outer; p_lin_r = p_inner; }
        else              { p_lin_l = p_inner; p_lin_r = p_outer; }
        v_target = clampf(0.5f * (p_lin_l + p_lin_r), 0.0f, V_MAX);
        u_cmd    = 0.5f * (p_lin_l - p_lin_r);

        if (mode_t > T_PIVOT_MAX) {
            ev_pivot_end = true; ev_pv_t = mode_t; ev_pv_z = 99.0f;
            z1 = z2 = z3 = x = vx = m_curv = rho_hat = risk_f = risk_dbg = 0.0f;
            eso_initialized = cross_exit_reinit = false;
            in_loss = is_gap_loss = false;
            loss_ticks = 0; lost_t = lost_d = handover_t = 0.0f;
            v_target = clampf(v_est_pwm, 0.0f, V_LOST);
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings(); mode = M_RUN; mode_t = 0.0f;
        }
    }
};

Brain brain;
int   ir_white_cal[4] = { ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM };
int   ir_black_cal[4] = { ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM };
int   adc_white_anchor = ADC_WHITE_NOM, track_span_est = 1500;
float floor_cm = 0.0f, local_peak = 1.0f;
unsigned long startMicros = 0, lastStepMicros = 0, lastTelemetryMillis = 0;
uint32_t loopFrameCount = 0;

void setupMotorsHardware() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ_HZ, PWM_RES_BITS, CH_L_A);
    ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ_HZ, PWM_RES_BITS, CH_L_B);
    ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ_HZ, PWM_RES_BITS, CH_R_A);
    ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ_HZ, PWM_RES_BITS, CH_R_B);
#elif defined(ESP32)
    ledcSetup(CH_L_A, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_L_B, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_R_A, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_R_B, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PWM_PIN_L_A, CH_L_A);
    ledcAttachPin(PWM_PIN_L_B, CH_L_B);
    ledcAttachPin(PWM_PIN_R_A, CH_R_A);
    ledcAttachPin(PWM_PIN_R_B, CH_R_B);
#endif
}

static inline void writeLEDC(int channel, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(channel, duty);
#elif defined(ESP32)
    ledcWrite(channel, duty);
#else
    analogWrite(channel, duty >> 2);
#endif
}

static inline int pwmToDuty10Bit(float p) {
    long d = lroundf(fabsf(p) * (1023.0f / 255.0f));
    if (d < 0L) return 0;
    if (d > PWM_MAX_DUTY) return PWM_MAX_DUTY;
    return (int)d;
}

static inline void writeMotors10Bit(float p_out_l, float p_out_r) {
    float hw_l = clampf(p_out_l, -255.0f, +255.0f);
    float hw_r = clampf(p_out_r * RIGHT_MOTOR_TRIM, -255.0f, +255.0f);
    if (SWAP_MOTORS_LR) { float tmp = hw_l; hw_l = hw_r; hw_r = tmp; }
    if (INVERT_LEFT_MOTOR)  hw_l = -hw_l;
    if (INVERT_RIGHT_MOTOR) hw_r = -hw_r;

    int duty_l = pwmToDuty10Bit(hw_l);
    int duty_r = pwmToDuty10Bit(hw_r);

    if (hw_l >= 0.0f) { writeLEDC(CH_L_A, duty_l); writeLEDC(CH_L_B, 0); }
    else              { writeLEDC(CH_L_A, 0);      writeLEDC(CH_L_B, duty_l); }
    if (hw_r >= 0.0f) { writeLEDC(CH_R_A, duty_r); writeLEDC(CH_R_B, 0); }
    else              { writeLEDC(CH_R_A, 0);      writeLEDC(CH_R_B, duty_r); }
}

static inline int readIRFiltered(int pin) {
    (void)analogRead(pin);
    int a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
    int hi = (a > b) ? a : b, lo = (a < b) ? a : b;
    return (c > hi) ? hi : ((c < lo) ? lo : c);
}

#if ENABLE_BLACKBOX
#define BB_N 5000
struct BBRec {
    uint16_t t10;
    int16_t  x10, vx10, uff10;
    uint16_t rho1000;
    int16_t  ueff10, pl, pr;
    uint8_t  s[4], mode, cross_count, cm, rk;
};
static_assert(sizeof(BBRec) <= 28, "BBRec SRAM budget exceeded");

struct Blackbox {
    static BBRec buf[BB_N];
    static uint16_t count;
    static bool dumped, was_lifted_prev;
    static void clear() { count = 0; dumped = false; }
    static void record(const SensorData& sd, float p_out_l, float p_out_r) {
        if (count >= BB_N) return;
        BBRec& r      = buf[count++];
        r.t10         = (uint16_t)clampf(sd.elapsed_sec * 100.0f, 0.0f, 65535.0f);
        r.x10         = (int16_t)clampf(brain.x * 10.0f, -32000.0f, 32000.0f);
        r.vx10        = (int16_t)clampf(brain.z2 * 10.0f, -32000.0f, 32000.0f);
        r.uff10       = (int16_t)clampf((brain.z3 / B0_GAIN) * 10.0f, -32000.0f, 32000.0f);
        r.rho1000     = (uint16_t)clampf(brain.rho_hat * 1000.0f, 0.0f, 65535.0f);
        r.ueff10      = (int16_t)clampf(brain.u_eff * 10.0f, -32000.0f, 32000.0f);
        r.pl          = (int16_t)roundi(clampf(p_out_l, -255.0f, 255.0f));
        r.pr          = (int16_t)roundi(clampf(p_out_r, -255.0f, 255.0f));
        for (int i = 0; i < 4; ++i) r.s[i] = (uint8_t)clampf(sd.ir[i] * 255.0f, 0.0f, 255.0f);
        r.mode        = (uint8_t)brain.mode;
        r.cross_count = (uint8_t)brain.cross_count;
        r.cm          = (uint8_t)clampf(floor_cm * 255.0f, 0.0f, 255.0f);
        r.rk          = (uint8_t)clampf(brain.risk_dbg * 255.0f, 0.0f, 255.0f);
    }
    static void dump() {
        if (count < 10) return;
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(50);
#endif
        Serial.println("\n=== BLACKBOX DUMP START ===");
        Serial.println("t,x_mm,vx,uff,rho,u_eff,pl,pr,s0,s1,s2,s3,mode,cross_count,floor_cm,risk");
        for (uint16_t i = 0; i < count; ++i) {
            const BBRec& r = buf[i];
            Serial.printf("%.2f,%.1f,%.1f,%.1f,%.3f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n",
                          r.t10 * 0.01f, r.x10 * 0.1f, r.vx10 * 0.1f, r.uff10 * 0.1f,
                          r.rho1000 * 0.001f, r.ueff10 * 0.1f, (int)r.pl, (int)r.pr,
                          (int)r.s[0], (int)r.s[1], (int)r.s[2], (int)r.s[3],
                          (int)r.mode, (int)r.cross_count, r.cm / 255.0f, r.rk / 255.0f);
            if ((i & 0x0F) == 0) delay(1);
        }
        Serial.println("=== BLACKBOX DUMP END ===");
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(0);
#endif
    }
};
BBRec Blackbox::buf[BB_N];
uint16_t Blackbox::count = 0;
bool Blackbox::dumped = false, Blackbox::was_lifted_prev = false;
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
    for (int i = 0; i < 4; ++i) pinMode(PIN_IR[i], INPUT);
    setupMotorsHardware();
    writeMotors10Bit(0.0f, 0.0f);

    unsigned long t0 = millis();
    while (millis() - t0 < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]), r3 = readIRFiltered(PIN_IR[3]);
        bool ok0 = (IR_POL < 0) ? (r0 > ADC_RAW_LO_OK + 350) : (r0 < ADC_RAW_HI_OK - 350);
        bool ok3 = (IR_POL < 0) ? (r3 > ADC_RAW_LO_OK + 350) : (r3 < ADC_RAW_HI_OK - 350);
        if (ok0 || ok3) break;
        delay(8);
    }
    t0 = millis();
    int prev_c[4] = {
        readIRFiltered(PIN_IR[0]), readIRFiltered(PIN_IR[1]),
        readIRFiltered(PIN_IR[2]), readIRFiltered(PIN_IR[3])
    };
    while (millis() - t0 < 90) {
        delay(12);
        int md = 0;
        for (int i = 0; i < 4; ++i) {
            int cur = readIRFiltered(PIN_IR[i]);
            md = max(md, abs(cur - prev_c[i]));
            prev_c[i] = cur;
        }
        if (md < 100) break;
    }

    int bright_seen[4], dark_seen[4];
    for (int i = 0; i < 4; ++i) bright_seen[i] = dark_seen[i] = readIRFiltered(PIN_IR[i]);
    for (int s = 0; s < 20; ++s) {
        for (int i = 0; i < 4; ++i) {
            int v = readIRFiltered(PIN_IR[i]);
            bright_seen[i] = brighterOf(bright_seen[i], v);
            dark_seen[i]   = darkerOf(dark_seen[i], v);
        }
        delay(3);
    }

    int g_white = bright_seen[0], g_dark = dark_seen[0];
    for (int i = 1; i < 4; ++i) {
        g_white = brighterOf(g_white, bright_seen[i]);
        g_dark  = darkerOf(g_dark, dark_seen[i]);
    }
    if ((IR_POL < 0) ? (g_white <= ADC_RAW_LO_OK + 200) : (g_white >= ADC_RAW_HI_OK - 200)) {
        g_white = ADC_WHITE_NOM; g_dark = ADC_BLACK_NOM;
        for (int i = 0; i < 4; ++i) dark_seen[i] = ADC_BLACK_NOM;
    }
    adc_white_anchor = g_white;
    int init_span = darker(g_dark, g_white);
    bool has_line = (init_span >= INIT_CONTRAST_SPAN && g_dark > ADC_RAW_LO_OK && g_dark < ADC_RAW_HI_OK);
    if (!has_line) {
        g_dark    = constrain(g_white + IR_POL * 1500, ADC_RAW_LO_OK + 50, ADC_RAW_HI_OK - 50);
        init_span = abs(g_dark - g_white);
    }
    track_span_est = constrain(init_span, INIT_CONTRAST_SPAN, 2800);
    int w_tol = has_line ? min(init_span / 3, 850) : 950;
    for (int i = 0; i < 4; ++i) {
        ir_white_cal[i] = (darker(bright_seen[i], g_white) < w_tol) ? bright_seen[i] : g_white;
        int dk = dark_seen[i];
        ir_black_cal[i] = (has_line && darker(dk, ir_white_cal[i]) >= INIT_CONTRAST_SPAN &&
                           dk > ADC_RAW_LO_OK && dk < ADC_RAW_HI_OK)
                          ? dk : darkerOf(g_dark, ir_white_cal[i] + IR_POL * track_span_est);
    }

    brain.reset();
    floor_cm = 0.0f; local_peak = 1.0f;
    startMicros = lastStepMicros = micros();
}

void loop() {
    unsigned long nowMicros = micros(), diffMicros = nowMicros - lastStepMicros;
    if (diffMicros < 2000UL) return;
    lastStepMicros = nowMicros;
    float dt = (diffMicros > 10000UL) ? 0.002f : clampf((float)diffMicros * 1e-6f, 0.001f, 0.004f);
    loopFrameCount++;

    SensorData sData;
    sData.elapsed_sec = (float)(nowMicros - startMicros) * 1e-6f;
    int frame_min_raw = 4095, frame_max_raw = 0;
    for (int i = 0; i < 4; ++i) {
        int raw = readIRFiltered(PIN_IR[i]);
        sData.raw_ir[i] = raw;
        if (raw < frame_min_raw) frame_min_raw = raw;
        if (raw > frame_max_raw) frame_max_raw = raw;
    }
    int  frame_span   = frame_max_raw - frame_min_raw;
    sData.frame_span  = frame_span;
    int  frame_bright = (IR_POL > 0) ? frame_min_raw : frame_max_raw;
    bool raw_in_range = (IR_POL < 0) ? (frame_min_raw > ADC_RAW_LO_OK) : (frame_max_raw < ADC_FRAME_HI_OK);
    bool slow_tick    = ((loopFrameCount & 0x1F) == 0);

    if (raw_in_range && frame_span > MIN_CONTRAST_SPAN) {
        if (darker(frame_bright, adc_white_anchor) < 0) {
            adc_white_anchor = (adc_white_anchor * 3 + frame_bright) / 4;
        } else if (slow_tick && darker(frame_bright, adc_white_anchor) > 150) {
            if ((IR_POL < 0) ? (frame_bright > ADC_RAW_LO_OK + 350) : (frame_bright < ADC_RAW_HI_OK - 350))
                adc_white_anchor += IR_POL;
        }
    }
    bool valid_contrast = (frame_span > MIN_CONTRAST_SPAN) &&
                          (darker(frame_bright, adc_white_anchor) < WHITE_SEEN_MAX_DARK) && raw_in_range;
    bool wing_busy = brain.w_on[0] || brain.w_on[1] || brain.w_rose[0] || brain.w_rose[1] ||
                     brain.on_crossbar || (brain.mode != Brain::M_RUN);

    float n_raw[4];
    int   eff_span[4], span_i_arr[4];
    int   max_ch_span = MIN_CONTRAST_SPAN, span_sum = 0;
    int   black_learn_thr = (track_span_est > MIN_CONTRAST_SPAN * 2) ? (track_span_est / 2) : MIN_CONTRAST_SPAN;

    for (int i = 0; i < 4; ++i) {
        int  raw       = sData.raw_ir[i];
        bool raw_ok    = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK) : (raw < ADC_RAW_HI_OK);
        bool bright_ok = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK + 400) : (raw < ADC_RAW_HI_OK - 400);
        if (raw_ok && bright_ok && darker(raw, ir_white_cal[i]) < 0) {
            ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            if (darker(raw, adc_white_anchor) < 0) adc_white_anchor = (adc_white_anchor * 3 + raw) / 4;
        }
        if (valid_contrast && !brain.was_lifted && !wing_busy &&
            (brain.off_track_t < 0.04f || frame_span < INIT_CONTRAST_SPAN)) {
            int dw = darker(raw, ir_white_cal[i]);
            if (slow_tick && dw < ((frame_span < INIT_CONTRAST_SPAN) ? 650 : 280) &&
                darker(ir_white_cal[i], adc_white_anchor) < WHITE_DRIFT_MAX_DARK) {
                ir_white_cal[i] += IR_POL;
            }
            dw = darker(raw, ir_white_cal[i]);
            if (brain.has_seen_line && brain.off_track_t < 0.04f && raw_ok &&
                dw > black_learn_thr && darker(raw, ir_black_cal[i]) > 0) {
                ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
            }
        }
        int min_blk = ir_white_cal[i] + IR_POL * MIN_CONTRAST_SPAN;
        if (slow_tick && valid_contrast && brain.has_seen_line && !brain.was_lifted &&
            brain.off_track_t < 0.04f && darker(ir_black_cal[i], ir_white_cal[i]) > MIN_CONTRAST_SPAN + 200) {
            ir_black_cal[i] -= 4 * IR_POL;
        }
        if (darker(ir_black_cal[i], min_blk) < 0) ir_black_cal[i] = min_blk;
        int span_i = darker(ir_black_cal[i], ir_white_cal[i]);
        span_i_arr[i] = span_i;
        if (span_i > max_ch_span) max_ch_span = span_i;
        span_sum += span_i;
    }

    int shared_floor = max((max_ch_span * 74) / 100, EFF_SPAN_MIN);
    for (int i = 0; i < 4; ++i) {
        eff_span[i] = max(span_i_arr[i], shared_floor);
        n_raw[i]    = (float)darker(sData.raw_ir[i], ir_white_cal[i]) / (float)eff_span[i];
    }
    if (slow_tick && valid_contrast && !brain.was_lifted && !wing_busy) {
        track_span_est = (track_span_est * 3 + (span_sum / 4)) / 4;
    }

#if !ENABLE_FLOOR_TRACK
    (void)eff_span; local_peak = 1.0f;
#else
    if (brain.was_lifted) {
        floor_cm = 0.0f; local_peak = 1.0f;
    } else {
        float n_min = min4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        float n_max = max4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        bool surf_line_ok = (frame_span >= CM_SPAN_MIN) && ((n_max - n_min) >= 0.16f) &&
                            raw_in_range && (brain.off_track_t < 0.04f);

        float cm_tgt = floor_cm;
        if (surf_line_ok && n_min <= CM_MAX) cm_tgt = max2f(0.0f, n_min - CM_DEAD);
        else if (raw_in_range && n_min < floor_cm) cm_tgt = max2f(0.0f, n_min - CM_DEAD);
        floor_cm = clampf(floor_cm + (cm_tgt - floor_cm) * (dt / (((cm_tgt > floor_cm) ? CM_TAU_UP : CM_TAU_DN) + dt)),
                          0.0f, CM_MAX - CM_DEAD);

        if (surf_line_ok && !brain.in_loss && !brain.on_crossbar && brain.mode == Brain::M_RUN) {
            float pk_obs = clampf((n_max - floor_cm) / PEAK_TARGET, PEAK_MIN, 1.0f);
            bool centered = (max2f(n_raw[1], n_raw[2]) >= max2f(n_raw[0], n_raw[3])) && (fabsf(brain.x) < 12.0f);
            if (pk_obs > local_peak) local_peak += (pk_obs - local_peak) * (dt / (PEAK_TAU_UP + dt));
            else if (centered)       local_peak += (pk_obs - local_peak) * (dt / (PEAK_TAU_DN + dt));
            local_peak = clampf(local_peak, PEAK_MIN, 1.0f);
        }
        if (slow_tick && surf_line_ok && !wing_busy && floor_cm > CM_ABSORB_MIN) {
            for (int i = 0; i < 4; ++i) {
                int w = ir_white_cal[i] + IR_POL * (int)(floor_cm * CM_ABSORB_RATE * (float)eff_span[i]);
                if (darker(w, adc_white_anchor) > WHITE_CAL_MAX_DARK) w = adc_white_anchor + IR_POL * WHITE_CAL_MAX_DARK;
                ir_white_cal[i] = w;
            }
        }
    }
#endif

    const float denom = clampf(local_peak, PEAK_MIN, 1.0f);
    for (int i = 0; i < 4; ++i) sData.ir[i] = clampf((n_raw[i] - floor_cm) / denom, 0.0f, 1.0f);

    float p_out_l = 0.0f, p_out_r = 0.0f;
    brain.step(sData, dt, p_out_l, p_out_r);
    writeMotors10Bit(p_out_l, p_out_r);

#if ENABLE_SERIAL_TELEMETRY
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 50UL) {
        lastTelemetryMillis = nowMs;
        Serial.printf("x=%.1f vx=%.0f uff=%.1f rho=%.3f ue=%.1f L=%d R=%d m=%d\n",
                      brain.x, brain.z2, brain.z3 / B0_GAIN, brain.rho_hat, brain.u_eff,
                      roundi(p_out_l), roundi(p_out_r), (int)brain.mode);
    }
#endif

#if ENABLE_BLACKBOX
    if (!brain.was_lifted) {
        if (Blackbox::was_lifted_prev) { Blackbox::clear(); Blackbox::was_lifted_prev = false; }
        if (brain.has_seen_line && (loopFrameCount % 10) == 0) Blackbox::record(sData, p_out_l, p_out_r);
    } else {
        Blackbox::was_lifted_prev = true;
        if (!Blackbox::dumped && Blackbox::count >= 10 && Serial) {
            Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros();
        }
        while (Serial.available() > 0) {
            int ch = Serial.read();
            if (ch == 'd' || ch == 'D') {
                Blackbox::dump(); Blackbox::dumped = true; lastStepMicros = micros();
            }
        }
    }
#endif
}
