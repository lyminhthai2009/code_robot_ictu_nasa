// ============================================================================
//  COBOT LINE FOLLOWER v4 (SLOW + 100% CROSS DETECT + STOP 5S) - ESP32-C3
//  - Đã fix lỗi bộ lọc làm mất ngã tư khi đi tốc độ chậm
//  - Sáng đèn LED trắng (chân 20) trong suốt 5 giây dừng tại ngã tư
// ============================================================================
#include <Arduino.h>
#include <math.h>
#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
// 1. CẤU HÌNH CHIẾN THUẬT
// ============================================================================
#define ENABLE_SHORTCUT          1   // 1 = rẽ theo CROSS_PLAN sau khi dừng 5s | 0 = luôn đi thẳng sau khi dừng 5s
#define ENABLE_SERIAL_TELEMETRY  0   // 1 = in log 8 Hz (chỉ để debug, tắt khi thi)
#define ENABLE_BLACKBOX          0   // 1 = ghi dữ liệu vào RAM, xả qua USB khi nhấc xe

// Thời gian dừng tại mỗi ngã tư (giây)
static const float CROSS_STOP_TIME_S = 5.0f;  // Dừng đủ 5 giây ở mọi ngã tư rồi mới đi tiếp

// Hành động ở từng lần gặp ngã tư (theo thứ tự gặp) sau khi đã dừng 5s:
//   0 = đi thẳng,  +1 = rẽ phải (về phía S4),  -1 = rẽ trái (về phía S1)
#define TURN_RIGHT  (+1)
#define TURN_LEFT   (-1)
static const int8_t CROSS_PLAN[] = { 0, 0, TURN_RIGHT };   // lần 1: thẳng | lần 2: thẳng | lần 3: rẽ phải
static const int    CROSS_PLAN_LEN = (int)(sizeof(CROSS_PLAN) / sizeof(CROSS_PLAN[0]));

// ---------------------------- Hình học xe (mm) -----------------------------
static const float SENSOR_X_MM[4]       = { -20.0f, -11.25f, 11.25f, 20.0f };
static const float AXLE_TO_OUTER_ROW_MM = 57.0f;   // trục bánh -> hàng S1/S4
static const float ASSUMED_LINE_W_MM    = 30.0f;   // bề rộng vạch giả định
static const float SPEED_MMPS_AT_255    = 500.0f;  // tốc độ thật (mm/s) khi PWM=255 trên sàn

// ---------------------------- Tốc độ (ĐÃ CHỈNH RẤT CHẬM) -------------------
static const float V_MAX         = 80.0f;    // tốc độ nền tối đa trên đường thẳng
static const float V_CORNER_MIN  = 55.0f;    // sàn tốc độ nền khi vào cua
static const float V_START       = 45.0f;    // tốc độ khởi động
static const float V_SEEK_LINE   = 55.0f;    // đi chậm khi chưa thấy vạch
static const float ACC_UP        = 300.0f;   // PWM/s, tăng tốc êm
static const float ACC_DN        = 3000.0f;  // PWM/s, giảm tốc
static const float K_SPD_U       = 0.35f;    // giảm tốc theo nỗ lực lái
static const float K_SPD_E       = 0.80f;    // giảm tốc theo sai số
static const float E_SPD_DEAD_MM = 6.0f;
static const float F_IN_MIN      = 0.0f;     // cho phép bánh trong giảm về 0 khi chạy chậm

// ---------------------------- PID (đơn vị mm) ------------------------------
static const float KP          = 4.5f;       // PWM / mm
static const float KD          = 0.18f;      // PWM*s / mm
static const float KI          = 25.0f;      // PWM / (mm*s)
static const float I_MAX       = 40.0f;
static const float I_LEAK_TAU  = 0.9f;       // s
static const float I_ACTIVE_MM = 15.0f;
static const float KP_NL_MM    = 40.0f;

// ---------------------------- Bộ lọc vị trí alpha-beta ---------------------
static const float AB_ALPHA = 0.40f;
static const float AB_BETA  = 0.08f;
static const float JUMP_MM  = 25.0f;

// ---------------------------- Ước lượng vị trí -----------------------------
static const float W_FLOOR   = 0.08f;
static const float WSUM_MIN  = 0.08f;
static const float K_EXT_MM  = 9.0f;
static const float X_LIM_MM  = 30.0f;

// ---------------------------- Mất vạch -------------------------------------
static const float GAP_X_MM       = 10.0f;
static const float GAP_HOLD_MM    = 50.0f;
static const float V_GAP_MAX      = 75.0f;
static const float V_LOST         = 60.0f;
static const float U_LOST_MAX     = 70.0f;
static const float LOST_RAMP_S    = 0.15f;
static const float LOST_GIVEUP_S  = 3.0f;

// ---------------------------- Ngã tư [ĐÃ FIX NHẠY CHO TỐC ĐỘ CHẬM] ---------
static const float WING_ON               = 0.38f;   // [FIX] hạ từ 0.50 -> 0.38 để bắt nhạy hơn
static const float WING_OFF              = 0.25f;   // [FIX] cánh < 0.25 = đã rời vạch ngang
static const float WING_BASE_MAX         = 0.55f;   // [FIX] nâng từ 0.30 -> 0.55 để đi chậm không bị khóa
static const float WING_SEP_ON           = 0.45f;   // [FIX] hạ từ 0.80 -> 0.45 để mắt ngoài hơi yếu vẫn bắt tốt
static const int   WING_DEBOUNCE_TICKS   = 2;       // [FIX] 2 tick (4 ms)
static const float WING_LATCH_MM         = 60.0f;   // cửa sổ chờ 2 cánh chạm vạch ngang (ngã tư xiên)
static const float CROSS_MIN_SPACING_MM  = 70.0f;   // [FIX] giảm quãng đường khóa để nhận được các ngã tư gần nhau
static const float CROSS_COOLDOWN_S      = 0.40f;   // [MỚI] thời gian nghỉ tối thiểu sau khi thoát hẳn ngã tư cũ
static const float CROSS_PASS_MAX_MM     = 200.0f;
static const float CROSS_PASS_MAX_S      = 2.50f;
static const float CROSS_EXIT_MARGIN_MM  = 15.0f;
static const float U_REF_TAU             = 0.20f;

// ---------------------------- Rẽ tắt (pivot) -------------------------------
static const float V_ALIGN           = 65.0f;
static const float ALIGN_FALLBACK_MM = 200.0f;
static const float ALIGN_FALLBACK_S  = 2.50f;
static const float P_CLEAR           = 75.0f;
static const float P_SEEK            = 60.0f;
static const float PIVOT_OLD_ON      = 0.45f;
static const float PIVOT_OLD_GONE    = 0.25f;
static const float PIVOT_NEW_OUTER   = 0.45f;
static const float PIVOT_RELAX_T     = 0.55f;
static const float T_PIVOT_MAX       = 2.20f;
static const float PIVOT_SWEEP_MMPS  = 150.0f;
static const float HANDOVER_T        = 0.35f;
static const float V_HANDOVER        = 55.0f;
static const float F_IN_HANDOVER     = -30.0f;
static const float U_MAX_HANDOVER    = 65.0f;

// ---------------------------- [V3] Lái chủ động khi vào cua ----------------
static const float T_PRED     = 0.045f;
static const float RISK_X0    = 9.0f;
static const float RISK_X1    = 20.0f;
static const float F_IN_EMERG = 0.0f;

// ============================================================================
// 2. CHÂN PHẦN CỨNG (GIỮ NGUYÊN)
// ============================================================================
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define CH_L_A 0
#define CH_L_B 1
#define CH_R_A 2
#define CH_R_B 3

#define SENSOR_1_PIN 4
#define SENSOR_2_PIN 3
#define SENSOR_3_PIN 1
#define SENSOR_4_PIN 0

#define W_LED_ON  20
#define IR_LED_ON 21

const int PIN_IR[4]          = {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN};
const int MIN_N20_PWM        = 36;
const float RIGHT_MOTOR_TRIM = 1.00f;

const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;
const bool SWAP_MOTORS_LR     = false;

#define SLEW_UP_PER_TICK     40
#define SLEW_DN_PER_TICK     160
#define REVERSE_STEP         140
#define REVERSE_PAUSE_TICKS  2

// ============================================================================
// 3. HÀM TOÁN
// ============================================================================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }
static inline int   roundi(float v) { return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)); }
static inline int   sgn(float v) { return (v >= 0.0f) ? 1 : -1; }

struct SensorData {
    float ir[4];      // 0.0 = trắng, 1.0 = đen
    int   raw_ir[4];
    float elapsed_sec;
};

static void mixWheels(float v, float u, float f_in, float& vl, float& vr) {
    float a_max = 0.5f * (255.0f - f_in);
    float a = fabsf(u);
    if (a > a_max) a = a_max;
    float lo = f_in + a;
    float hi = 255.0f - a;
    float ve = v;
    if (ve < lo) ve = lo;
    if (ve > hi) ve = hi;
    if (u >= 0.0f) { vl = ve + a; vr = ve - a; }
    else           { vl = ve - a; vr = ve + a; }
}

// ============================================================================
// 4. BỘ ĐIỀU KHIỂN TRUNG TÂM
// ============================================================================
class Brain {
public:
    enum Mode : uint8_t {
        M_RUN = 0,
        M_CROSS_WAIT,      // Dừng 5 giây tại ngã tư
        M_CROSS_PASS,      // Đi thẳng qua ngã tư
        M_TURN_ALIGN,      // Tiến vào tâm ngã tư để chuẩn bị rẽ
        M_TURN_PIVOT       // Xoay tại chỗ bắt nhánh mới
    };

    Mode  mode;
    float mode_t, mode_d;

    float x, vx;
    bool  need_reinit;
    float integ, u_abs_f, u_cmd;
    float v_cmd, v_est, dist_mm;
    int   last_side;

    bool  in_loss, meas_ok;
    float lost_t, lost_d, lost_x;
    int   lost_side;
    float handover_t;
    float cross_cooldown_t; // Đếm ngược thời gian nghỉ sau khi qua ngã tư

    bool  was_lifted, has_seen_line;
    float off_track_t, run_t;

    bool  base_init;
    float base_o[2];
    bool  w_on[2], w_rose[2], w_fell[2], w_sep;
    int   w_ticks[2];
    float w_cand_d[2], w_rise_d[2], w_fall_d[2], w_last_d[2];
    float cross_sec, center_odo, end_odo;
    bool  center_refined;
    float u_ref, u_hold;
    int   cross_confirm, cross_count;
    float last_cross_dist;
    int   clear_ticks, white_ticks;
    int   turn_dir;
    int   pending_cross_act;
    bool  pivot_old_seen;

    bool  ev_cross; int ev_idx, ev_act; float ev_center_d;
    int   prev_pl, prev_pr;

    Brain() { reset(); }

    void reset() {
        mode = M_RUN; mode_t = 0.0f; mode_d = 0.0f;
        x = 0.0f; vx = 0.0f; need_reinit = true;
        integ = 0.0f; u_abs_f = 0.0f; u_cmd = 0.0f;
        v_cmd = V_START; v_est = 0.0f; dist_mm = 0.0f;
        last_side = 1;
        in_loss = false; meas_ok = false;
        lost_t = 0.0f; lost_d = 0.0f; lost_x = 0.0f; lost_side = 1;
        handover_t = 0.0f;
        cross_cooldown_t = 0.0f;
        was_lifted = false; has_seen_line = false;
        off_track_t = 0.0f; run_t = 0.0f;
        base_init = false; base_o[0] = base_o[1] = 0.0f;
        cross_confirm = 0; cross_count = 0; last_cross_dist = -1.0e6f;
        clear_ticks = 0; white_ticks = 0; turn_dir = 1;
        pending_cross_act = 0;
        pivot_old_seen = false;
        cross_sec = 1.0f; center_odo = 0.0f; end_odo = 0.0f; center_refined = false;
        u_ref = 0.0f; u_hold = 0.0f;
        ev_cross = false; ev_idx = 0; ev_act = 0; ev_center_d = 0.0f;
        clearWings();
        prev_pl = 0; prev_pr = 0;
    }

    // ------------------------------------------------------------------------
    void step(const SensorData& sd, float dt, int& pl, int& pr) {
        pl = 0; pr = 0;
        if (!(dt > 0.0005f && dt < 0.02f)) dt = 0.002f;
        const float* s = sd.ir;

        // ---- Nhấc xe / đặt xe ----
        float max_ir = max4f(s[0], s[1], s[2], s[3]);
        float min_ir = min4f(s[0], s[1], s[2], s[3]);
        bool all_white = (max_ir < 0.10f);
        bool all_black = (min_ir > 0.88f);
        // Không coi là nhấc xe khi đang dừng chờ ở ngã tư, đang đi qua ngã tư, hoặc đang xoay
        bool ignore_lift = (mode == M_TURN_PIVOT || mode == M_CROSS_WAIT || mode == M_CROSS_PASS || mode == M_TURN_ALIGN);

        if (all_white && !ignore_lift) {
            off_track_t += dt;
            if (off_track_t > 0.85f) was_lifted = true;
        } else if (all_black && !ignore_lift) {
            // Lưu ý: nếu 4 mắt cùng đen trong lúc M_RUN thì đó chính là NGÃ TƯ chứ không phải nhấc xe!
            // Chỉ khi 4 mắt đen liên tục > 2.5s mà không vào ngã tư mới coi là nhấc xe.
            off_track_t += dt;
            if (off_track_t > 2.50f) was_lifted = true;
        } else {
            if (was_lifted) {
                if ((s[1] > 0.35f || s[2] > 0.35f) && min_ir < 0.45f) {
                    reset();
                    has_seen_line = true;
                }
            } else {
                if (max_ir >= 0.20f && min_ir < 0.75f) has_seen_line = true;
                off_track_t = 0.0f;
            }
        }
        if (was_lifted) { stopOutputs(); return; }

        run_t += dt;
        if (cross_cooldown_t > 0.0f) cross_cooldown_t -= dt;
        updateSpeedEstimate(dt);

        // ---- Chưa thấy vạch: đi thẳng chậm ----
        if (!has_seen_line) {
            float v = 0.0f;
            if (off_track_t < 0.45f) { rampSpeed(V_SEEK_LINE, dt); v = v_cmd; }
            pl = roundi(v); pr = roundi(v);
            prev_pl = pl; prev_pr = pr;
            return;
        }

        if (mode == M_RUN) runEstimate(s, dt);
        if (was_lifted) { stopOutputs(); return; }

        float vl = 0.0f, vr = 0.0f;
        switch (mode) {
            case M_RUN:          outRun(dt, vl, vr);          break;
            case M_CROSS_WAIT:   outCrossWait(s, dt, vl, vr); break;
            case M_CROSS_PASS:   outCrossPass(s, dt, vl, vr); break;
            case M_TURN_ALIGN:   outAlign(s, dt, vl, vr);     break;
            case M_TURN_PIVOT:   outPivot(s, dt, vl, vr);     break;
        }
        pl = roundi(clampf(vl, -255.0f, 255.0f));
        pr = roundi(clampf(vr, -255.0f, 255.0f));
        prev_pl = pl; prev_pr = pr;
    }

private:
    void stopOutputs() { v_cmd = 0.0f; v_est = 0.0f; prev_pl = 0; prev_pr = 0; }

    void rampSpeed(float target, float dt) {
        if (target > v_cmd) { v_cmd += ACC_UP * dt; if (v_cmd > target) v_cmd = target; }
        else                { v_cmd -= ACC_DN * dt; if (v_cmd < target) v_cmd = target; }
    }

    void updateSpeedEstimate(float dt) {
        if (mode == M_CROSS_WAIT) {
            v_est = 0.0f;
            return;
        }
        float c = 0.5f * (float)(prev_pl + prev_pr);
        if (c < 0.0f) c = 0.0f;
        float tgt = SPEED_MMPS_AT_255 * c / 255.0f;
        v_est += (tgt - v_est) * (dt / (0.04f + dt));
        dist_mm += v_est * dt;
    }

    bool measure(const float* s, float& z) const {
        float w[4], wsum = 0.0f;
        for (int i = 0; i < 4; i++) {
            float c = (s[i] - W_FLOOR) * (1.0f / (1.0f - W_FLOOR));
            w[i] = (c > 0.0f) ? c : 0.0f;
            wsum += w[i];
        }
        if (wsum < WSUM_MIN) return false;
        float num = 0.0f;
        for (int i = 0; i < 4; i++) num += w[i] * SENSOR_X_MM[i];
        float xc = num / wsum;

        if (w[3] >= w[0]) {
            float inner = w[0] + w[1] + w[2];
            float b = clampf(1.0f - inner / 0.30f, 0.0f, 1.0f);
            xc += b * K_EXT_MM * (1.0f - min2f(w[3], 1.0f));
        } else {
            float inner = w[1] + w[2] + w[3];
            float b = clampf(1.0f - inner / 0.30f, 0.0f, 1.0f);
            xc -= b * K_EXT_MM * (1.0f - min2f(w[0], 1.0f));
        }
        z = clampf(xc, -X_LIM_MM, X_LIM_MM);
        return true;
    }

    // [FIX] Chỉ cập nhật nền khi cảm biến thực sự ở vùng trắng (< 0.20f)
    // Tránh việc xe đi chậm làm base_o tăng theo cạnh vạch đen và tự khóa nhận diện ngã tư
    void updateWingBase(const float* s, float dt) {
        if (!base_init) { base_o[0] = min2f(s[0], 0.15f); base_o[1] = min2f(s[3], 0.15f); base_init = true; }
        const float ab = dt / (0.20f + dt);
        if (s[0] < 0.20f) base_o[0] += ab * (s[0] - base_o[0]);
        if (s[3] < 0.20f) base_o[1] += ab * (s[3] - base_o[1]);
    }

    void clearWings() {
        for (int k = 0; k < 2; k++) {
            w_on[k] = false; w_rose[k] = false; w_fell[k] = false; w_ticks[k] = 0;
            w_cand_d[k] = w_rise_d[k] = w_fall_d[k] = w_last_d[k] = dist_mm;
        }
        w_sep = false;
    }

    void trackWings(const float* s, float dt) {
        const int idx[2] = { 0, 3 };
        for (int k = 0; k < 2; k++) {
            float v = s[idx[k]];
            if (w_on[k]) {
                w_last_d[k] = dist_mm;
                if (v < WING_OFF) { w_on[k] = false; w_fell[k] = true; w_fall_d[k] = dist_mm; }
            } else if (!w_rose[k]) {
                if (v > WING_ON && base_o[k] < WING_BASE_MAX) {
                    if (w_ticks[k] == 0) w_cand_d[k] = dist_mm;
                    if (++w_ticks[k] >= WING_DEBOUNCE_TICKS) {
                        w_on[k] = true; w_rose[k] = true; w_fell[k] = false;
                        w_rise_d[k] = w_cand_d[k]; w_last_d[k] = dist_mm;
                    }
                } else {
                    w_ticks[k] = 0;
                }
            } else if (mode == M_RUN && (dist_mm - w_last_d[k]) > WING_LATCH_MM) {
                w_rose[k] = false; w_fell[k] = false; w_ticks[k] = 0;
            }
        }
        if (w_rose[0] || w_rose[1] || w_ticks[0] > 0 || w_ticks[1] > 0) {
            if ((s[0] > WING_SEP_ON && s[2] > WING_SEP_ON) ||
                (s[1] > WING_SEP_ON && s[3] > WING_SEP_ON) ||
                (s[0] > WING_ON && s[3] > WING_ON)) {
                w_sep = true;
            }
        } else {
            w_sep = false;
        }
        updateWingBase(s, dt);
    }

    // [FIX] Nhận diện ngã tư kết hợp:
    //   1. Trực tiếp: Cả 2 cánh ngoài (S1, S4) đều đen (> WING_ON) + ít nhất 1 mắt giữa đen (S2 hoặc S3 > 0.35)
    //   2. Ngã tư xiên: 2 cánh chạm vạch lệch thời gian (w_rose[0] && w_rose[1] && w_sep && center)
    bool detectCross(const float* s, bool allowed) {
        if (!allowed) {
            cross_confirm = 0;
            return false;
        }
        bool center      = (s[1] > 0.35f || s[2] > 0.35f);
        bool direct_both = (s[0] > WING_ON && s[3] > WING_ON && center);
        bool skewed_both = (w_rose[0] && w_rose[1] && w_sep && center);

        if (direct_both || skewed_both) {
            cross_confirm++;
        } else {
            cross_confirm = 0;
        }
        return (cross_confirm >= 2);
    }

    void planCross() {
        // Nếu bắt trực tiếp mà 1 cánh chưa kịp chốt w_rise_d thì gán bằng dist_mm hiện tại
        if (!w_rose[0]) w_rise_d[0] = dist_mm;
        if (!w_rose[1]) w_rise_d[1] = dist_mm;

        float span = SENSOR_X_MM[3] - SENSOR_X_MM[0];
        float t = fabsf(w_rise_d[1] - w_rise_d[0]) / span;
        t = min2f(t, 3.0f);
        cross_sec  = sqrtf(1.0f + t * t);
        float thick = ASSUMED_LINE_W_MM * cross_sec;
        center_odo = 0.5f * (w_rise_d[0] + w_rise_d[1]) + 0.5f * thick;
        end_odo    = max2f(w_rise_d[0], w_rise_d[1]) + thick + CROSS_EXIT_MARGIN_MM;
        center_refined = false;
    }

    void refineCenter() {
        if (center_refined || !(w_fell[0] && w_fell[1])) return;
        center_refined = true;
        float E = ASSUMED_LINE_W_MM * cross_sec;
        float L0 = w_fall_d[0] - w_rise_d[0], L1 = w_fall_d[1] - w_rise_d[1];
        if (L0 > 0.6f * E && L0 < 1.5f * E && L1 > 0.6f * E && L1 < 1.5f * E) {
            center_odo = 0.25f * (w_rise_d[0] + w_fall_d[0] + w_rise_d[1] + w_fall_d[1]);
        }
    }

    void enterCross() {
        int idx = cross_count;
        cross_count++;
        int act = 0;
#if ENABLE_SHORTCUT
        if (idx < CROSS_PLAN_LEN) act = CROSS_PLAN[idx];
#endif
        pending_cross_act = act;
        last_cross_dist = dist_mm;
        u_hold = u_ref;
        clear_ticks = 0; white_ticks = 0;
        mode_t = 0.0f; mode_d = 0.0f;
        cross_confirm = 0;
        planCross();
        ev_cross = true; ev_idx = idx + 1; ev_act = act; ev_center_d = center_odo;

        // Vào trạng thái dừng 5 giây ở mọi ngã tư
        mode = M_CROSS_WAIT;
    }

    // ---- Dừng 5 giây tại ngã tư ----
    void outCrossWait(const float* s, float dt, float& vl, float& vr) {
        (void)s;
        mode_t += dt;
        vl = 0.0f;
        vr = 0.0f;
        v_cmd = 0.0f;
        v_est = 0.0f;
        u_cmd = 0.0f;

        if (mode_t >= CROSS_STOP_TIME_S) {
            mode_t = 0.0f;
            mode_d = 0.0f;
            clear_ticks = 0;
            v_cmd  = V_START;
            if (pending_cross_act == 0) {
                mode = M_CROSS_PASS;
            } else {
                turn_dir = (pending_cross_act > 0) ? 1 : -1;
                mode = M_TURN_ALIGN;
            }
        }
    }

    // ---- RUN: cập nhật bộ lọc, mất vạch, ngã tư ----
    void runEstimate(const float* s, float dt) {
        bool allowed = (handover_t <= 0.0f) && (cross_cooldown_t <= 0.0f) && !in_loss &&
                       (run_t > 0.20f) && ((dist_mm - last_cross_dist) >= CROSS_MIN_SPACING_MM);
        if (allowed) trackWings(s, dt);
        else { clearWings(); updateWingBase(s, dt); }
        bool crossing = detectCross(s, allowed);

        float z = 0.0f;
        meas_ok = measure(s, z);
        if (meas_ok) {
            if (in_loss) {
                in_loss = false;
                if (lost_t > 0.012f) need_reinit = true;
                if (lost_t > 0.15f) integ *= 0.3f;
                lost_t = 0.0f; lost_d = 0.0f;
            }
            if (!need_reinit) {
                float xp = x + vx * dt;
                float r  = z - xp;
                if (fabsf(r) > JUMP_MM) {
                    need_reinit = true;
                } else {
                    x  = xp + AB_ALPHA * r;
                    vx += (AB_BETA / dt) * r;
                    vx = clampf(vx, -1500.0f, 1500.0f);
                }
            }
            if (need_reinit) { x = z; vx = 0.0f; need_reinit = false; }
            if (fabsf(x) > 3.0f) last_side = (x > 0.0f) ? 1 : -1;
        } else {
            if (!in_loss) {
                in_loss = true; lost_t = 0.0f; lost_d = 0.0f; lost_x = x;
                lost_side = (fabsf(x) > 1.5f) ? ((x > 0.0f) ? 1 : -1) : last_side;
            }
            lost_t += dt;
            lost_d += v_est * dt;
            x = clampf(x + vx * dt, -X_LIM_MM, X_LIM_MM);
            vx *= 0.96f;
            if (lost_t > LOST_GIVEUP_S) was_lifted = true;
        }
        if (crossing) enterCross();
    }

    void outRun(float dt, float& vl, float& vr) {
        float f_in  = F_IN_MIN;
        float a_cap = 0.5f * (255.0f - f_in);
        bool  handover = false;
        if (handover_t > 0.0f) {
            handover = true;
            handover_t -= dt;
            f_in  = F_IN_HANDOVER;
            a_cap = min2f(U_MAX_HANDOVER, 0.5f * (255.0f - f_in));
        }
        float u = 0.0f, v_t = V_MAX;

        if (!in_loss) {
            float ae = fabsf(x);
            float x_pred = x + T_PRED * vx;
            float risk   = clampf((max2f(fabsf(x_pred), ae) - RISK_X0) / (RISK_X1 - RISK_X0), 0.0f, 1.0f);
            if (!handover) {
                f_in  = F_IN_MIN + (F_IN_EMERG - F_IN_MIN) * risk;
                a_cap = 0.5f * (255.0f - f_in);
            }

            float kp = KP * (1.0f + ae / KP_NL_MM);
            float u_raw = kp * x + integ + KD * vx;
            bool  sat = fabsf(u_raw) > a_cap;
            if (ae < I_ACTIVE_MM && !(sat && ((u_raw > 0.0f) == (x > 0.0f)))) integ += KI * x * dt;
            integ -= integ * (dt / I_LEAK_TAU);
            integ = clampf(integ, -I_MAX, I_MAX);
            u = clampf(u_raw, -a_cap, a_cap);
            u_abs_f += (fabsf(u) - u_abs_f) * (dt / (0.06f + dt));
            v_t = V_MAX - K_SPD_U * u_abs_f - K_SPD_E * max2f(0.0f, ae - E_SPD_DEAD_MM);
            v_t = clampf(v_t, V_CORNER_MIN, V_MAX);
        } else {
            float lx = fabsf(lost_x);
            if (lx < GAP_X_MM && lost_d < GAP_HOLD_MM) {
                u = clampf(integ, -a_cap, a_cap);
                v_t = min2f(v_cmd, V_GAP_MAX);
            } else {
                if (!handover) { f_in = F_IN_EMERG; a_cap = 0.5f * (255.0f - f_in); }
                float ramp = clampf(lost_t / LOST_RAMP_S, 0.0f, 1.0f);
                float u0   = min2f(KP * (1.0f + lx / KP_NL_MM) * max2f(lx, 20.0f), U_LOST_MAX);
                float mag  = u0 + (U_LOST_MAX - u0) * ramp;
                u = (float)lost_side * min2f(mag, a_cap);
                v_t = V_LOST;
            }
        }
        rampSpeed(v_t, dt);
        mixWheels(v_cmd, u, f_in, vl, vr);
        u_cmd = u;
        if (!handover && !in_loss && !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0)
            u_ref += (u - u_ref) * (dt / (U_REF_TAU + dt));
    }

    // ---- Đi thẳng qua ngã tư (sau khi đã dừng 5s) ----
    // Đảm bảo xe tiến qua hẳn vạch ngang (2 cánh S1, S4 trở về nền trắng) rồi mới quay lại M_RUN
    void outCrossPass(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        mode_d += v_est * dt;
        trackWings(s, dt);
        rampSpeed(V_MAX, dt);
        u_hold -= u_hold * (dt / 0.15f);

        bool wings_white = (s[0] < WING_OFF && s[3] < WING_OFF);
        if (mode_t > 0.18f && wings_white) {
            clear_ticks++;
        } else {
            clear_ticks = 0;
        }

        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;

        if (clear_ticks >= 6 || mode_d > CROSS_PASS_MAX_MM || mode_t > CROSS_PASS_MAX_S) {
            mode = M_RUN;
            need_reinit = true;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f;
            last_cross_dist = dist_mm;
            cross_cooldown_t = CROSS_COOLDOWN_S;
            clearWings();
        }
    }

    // ---- Rẽ tắt: tiến vào tâm ngã tư (sau khi đã dừng 5s) ----
    void outAlign(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        mode_d += v_est * dt;
        trackWings(s, dt);
        refineCenter();
        rampSpeed(V_ALIGN, dt);
        u_hold -= u_hold * (dt / 0.15f);
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;
        ev_center_d = center_odo;
        if (dist_mm >= center_odo + AXLE_TO_OUTER_ROW_MM || mode_d > ALIGN_FALLBACK_MM || mode_t > ALIGN_FALLBACK_S) {
            enterPivot();
        }
    }

    void enterPivot() {
        mode = M_TURN_PIVOT; mode_t = 0.0f; mode_d = 0.0f; white_ticks = 0; pivot_old_seen = false;
        clearWings();
    }

    void outPivot(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        float old_out = (turn_dir > 0) ? s[0] : s[3];
        bool  new_in  = (turn_dir > 0) ? (s[1] > 0.40f || s[2] > 0.40f || s[3] > PIVOT_NEW_OUTER)
                                       : (s[2] > 0.40f || s[1] > 0.40f || s[0] > PIVOT_NEW_OUTER);
        if (old_out > PIVOT_OLD_ON) { if (++white_ticks >= 2) pivot_old_seen = true; }
        else white_ticks = 0;
        bool old_gone = (pivot_old_seen || mode_t > PIVOT_RELAX_T) && old_out < PIVOT_OLD_GONE;
        float p = old_gone ? P_SEEK : P_CLEAR;
        vl =  (float)turn_dir * p;
        vr = -(float)turn_dir * p;
        v_cmd = 0.0f;
        if (old_gone && new_in && mode_t > 0.015f) {
            float z = 0.0f;
            if (!measure(s, z)) z = (float)turn_dir * 25.0f;
            x = z; vx = -(float)turn_dir * PIVOT_SWEEP_MMPS; need_reinit = false;
            integ = 0.0f; u_abs_f = 0.0f; u_cmd = 0.0f;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f;
            handover_t = HANDOVER_T; v_cmd = V_HANDOVER;
            last_side = turn_dir; last_cross_dist = dist_mm;
            cross_cooldown_t = CROSS_COOLDOWN_S;
            clearWings();
            mode = M_RUN; mode_t = 0.0f;
            return;
        }
        if (mode_t > T_PIVOT_MAX) {
            x = 0.0f; vx = 0.0f; need_reinit = true;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f;
            integ = 0.0f; handover_t = 0.0f; v_cmd = V_LOST;
            last_side = turn_dir; last_cross_dist = dist_mm;
            cross_cooldown_t = CROSS_COOLDOWN_S;
            clearWings();
            mode = M_RUN; mode_t = 0.0f;
        }
    }
};

// ============================================================================
// 5. PHẦN CỨNG: ĐỘNG CƠ, CẢM BIẾN
// ============================================================================
Brain brain;

int ir_white_cal[4] = {300, 300, 300, 300};
int ir_black_cal[4] = {2400, 2400, 2400, 2400};

unsigned long startMicros         = 0;
unsigned long lastStepMicros      = 0;
unsigned long lastTelemetryMillis = 0;
uint32_t      loopFrameCount      = 0;

int prev_hw_pwm_l = 0, prev_hw_pwm_r = 0;
int zero_cross_hold_l = 0, zero_cross_hold_r = 0;

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

static inline int applyN20Deadzone(int pwm) {
    if (pwm == 0) return 0;
    int mag = abs(pwm);
    if (mag >= 250) return (pwm > 0) ? 255 : -255;
    int out = MIN_N20_PWM + ((mag * (255 - MIN_N20_PWM)) / 255);
    out = constrain(out, 0, 255);
    return (pwm > 0) ? out : -out;
}

static inline int slewAxis(int target, int prev, int& hold) {
    if (hold > 0) { hold--; return 0; }
    if (prev != 0 && target != 0 && ((target > 0) != (prev > 0))) {
        int mag = abs(prev);
        if (mag > REVERSE_STEP) return (prev > 0) ? (prev - REVERSE_STEP) : (prev + REVERSE_STEP);
        hold = REVERSE_PAUSE_TICKS;
        return 0;
    }
    int diff = target - prev;
    int lim  = (abs(target) > abs(prev)) ? SLEW_UP_PER_TICK : SLEW_DN_PER_TICK;
    if (diff > lim) diff = lim;
    else if (diff < -lim) diff = -lim;
    return prev + diff;
}

void setMotorHardware(int pwm_l, int pwm_r) {
    // Khi đang ở chế độ dừng 5 giây tại ngã tư: phanh đứng im ngay lập tức!
    if (brain.mode == Brain::M_CROSS_WAIT) {
        prev_hw_pwm_l = 0;
        prev_hw_pwm_r = 0;
        zero_cross_hold_l = 0;
        zero_cross_hold_r = 0;
        writeLEDC(CH_L_A, 0); writeLEDC(CH_L_B, 0);
        writeLEDC(CH_R_A, 0); writeLEDC(CH_R_B, 0);
        return;
    }

    if (SWAP_MOTORS_LR) { int tmp = pwm_l; pwm_l = pwm_r; pwm_r = tmp; }
    if (INVERT_LEFT_MOTOR)  pwm_l = -pwm_l;
    if (INVERT_RIGHT_MOTOR) pwm_r = -pwm_r;

    int target_l = applyN20Deadzone(pwm_l);
    int target_r = applyN20Deadzone((int)(pwm_r * RIGHT_MOTOR_TRIM));

    int out_l = slewAxis(target_l, prev_hw_pwm_l, zero_cross_hold_l);
    int out_r = slewAxis(target_r, prev_hw_pwm_r, zero_cross_hold_r);
    prev_hw_pwm_l = out_l;
    prev_hw_pwm_r = out_r;

    if (out_l >= 0) { writeLEDC(CH_L_A, out_l); writeLEDC(CH_L_B, 0); }
    else            { writeLEDC(CH_L_A, 0);     writeLEDC(CH_L_B, -out_l); }

    if (out_r >= 0) { writeLEDC(CH_R_A, out_r); writeLEDC(CH_R_B, 0); }
    else            { writeLEDC(CH_R_A, 0);     writeLEDC(CH_R_B, -out_r); }
}

static inline int readIRFiltered(int pin) {
    (void)analogRead(pin);
    int a = analogRead(pin);
    int b = analogRead(pin);
    int c = analogRead(pin);
    int hi = (a > b) ? a : b;
    int lo = (a < b) ? a : b;
    return (c > hi) ? hi : ((c < lo) ? lo : c);
}

// ============================================================================
// 6. BLACKBOX (tùy chọn)
// ============================================================================
#if ENABLE_BLACKBOX
#define BB_N 7500
struct BBRec { uint16_t t10; int16_t x10; int16_t u; int16_t pl; int16_t pr; uint8_t s[4]; uint8_t mode; uint8_t pad; };
static BBRec    bb_buf[BB_N];
static uint16_t bb_n = 0;
static bool     bb_dumped = false;

static void bbRecord(const SensorData& sd, int pl, int pr) {
    if (bb_n >= BB_N) return;
    BBRec& r = bb_buf[bb_n++];
    r.t10 = (uint16_t)clampf(sd.elapsed_sec * 100.0f, 0.0f, 65535.0f);
    r.x10 = (int16_t)clampf(brain.x * 10.0f, -32000.0f, 32000.0f);
    r.u   = (int16_t)brain.u_cmd;
    r.pl  = (int16_t)pl; r.pr = (int16_t)pr;
    for (int i = 0; i < 4; i++) r.s[i] = (uint8_t)(sd.ir[i] * 255.0f);
    r.mode = (uint8_t)brain.mode; r.pad = (uint8_t)brain.cross_count;
}
static void bbDump() {
    if (bb_n < 50) { bb_n = 0; return; }
    Serial.println("t,x_mm,u,pl,pr,s0,s1,s2,s3,mode,cross_count");
    for (uint16_t i = 0; i < bb_n; i++) {
        const BBRec& r = bb_buf[i];
        Serial.printf("%.2f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", r.t10 * 0.01f, r.x10 * 0.1f, (int)r.u,
                      (int)r.pl, (int)r.pr, (int)r.s[0], (int)r.s[1], (int)r.s[2], (int)r.s[3], (int)r.mode, (int)r.pad);
    }
    bb_n = 0;
}
#endif

// ============================================================================
// 7. KHỞI TẠO + AUTO-CALIB
// ============================================================================
void setup() {
#if defined(RTC_CNTL_BROWN_OUT_REG) && defined(RTC_CNTL_BROWN_OUT_ENA)
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#endif

    pinMode(PWM_PIN_L_A, OUTPUT); digitalWrite(PWM_PIN_L_A, LOW);
    pinMode(PWM_PIN_L_B, OUTPUT); digitalWrite(PWM_PIN_L_B, LOW);
    pinMode(PWM_PIN_R_A, OUTPUT); digitalWrite(PWM_PIN_R_A, LOW);
    pinMode(PWM_PIN_R_B, OUTPUT); digitalWrite(PWM_PIN_R_B, LOW);

    pinMode(IR_LED_ON, OUTPUT);
    digitalWrite(IR_LED_ON, HIGH);
    pinMode(W_LED_ON, OUTPUT);
    digitalWrite(W_LED_ON, LOW);

#if defined(ESP32)
    WiFi.mode(WIFI_OFF);
    btStop();
    analogReadResolution(12);
#endif

#if ENABLE_SERIAL_TELEMETRY || ENABLE_BLACKBOX
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
#endif

    for (int i = 0; i < 4; i++) pinMode(PIN_IR[i], INPUT);

    setupMotorsHardware();
    setMotorHardware(0, 0);

    // Bước 1: đợi xe được đặt xuống sàn
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]);
        int r3 = readIRFiltered(PIN_IR[3]);
        if (r0 < 1500 || r3 < 1500) break;
        delay(8);
    }

    // Bước 2: chờ tay rời khỏi xe
    unsigned long settleStart = millis();
    int prev_calib[4] = {readIRFiltered(PIN_IR[0]), readIRFiltered(PIN_IR[1]),
                         readIRFiltered(PIN_IR[2]), readIRFiltered(PIN_IR[3])};
    while (millis() - settleStart < 90) {
        delay(12);
        int max_diff = 0;
        for (int i = 0; i < 4; i++) {
            int cur = readIRFiltered(PIN_IR[i]);
            int d = abs(cur - prev_calib[i]);
            if (d > max_diff) max_diff = d;
            prev_calib[i] = cur;
        }
        if (max_diff < 100) break;
    }

    // Bước 3: đo mẫu nhanh tại vị trí xuất phát
    int min_seen[4] = {4095, 4095, 4095, 4095};
    int max_seen[4] = {0, 0, 0, 0};
    for (int sample = 0; sample < 20; sample++) {
        for (int i = 0; i < 4; i++) {
            int v = readIRFiltered(PIN_IR[i]);
            if (v < min_seen[i]) min_seen[i] = v;
            if (v > max_seen[i]) max_seen[i] = v;
        }
        delay(3);
    }

    int global_min = min(min(min_seen[0], min_seen[1]), min(min_seen[2], min_seen[3]));
    int global_max = max(max(max_seen[0], max_seen[1]), max(max_seen[2], max_seen[3]));

    if (global_min > 1200) global_min = 320;
    if (global_max < global_min + 650 || global_max > 3850) {
        global_max = constrain(global_min + 1500, 1400, 3200);
    }

    for (int i = 0; i < 4; i++) {
        if (min_seen[i] < global_min + 220) ir_white_cal[i] = min_seen[i];
        else                                ir_white_cal[i] = global_min;

        if (max_seen[i] > ir_white_cal[i] + 650 && max_seen[i] < 3850) ir_black_cal[i] = max_seen[i];
        else ir_black_cal[i] = max(global_max, ir_white_cal[i] + 1200);
    }

    brain.reset();
    startMicros    = micros();
    lastStepMicros = startMicros;
}

// ============================================================================
// 8. VÒNG LẶP 500 Hz
// ============================================================================
void loop() {
    unsigned long nowMicros  = micros();
    unsigned long diffMicros = nowMicros - lastStepMicros;
    if (diffMicros < 2000) return;
    lastStepMicros = nowMicros;

    float dt = (float)diffMicros * 1e-6f;
    if (dt > 0.025f) dt = 0.002f;

    loopFrameCount++;

    SensorData sData;
    sData.elapsed_sec = (float)(nowMicros - startMicros) * 1e-6f;

    int frame_min_raw = 4095;
    int frame_max_raw = 0;
    for (int i = 0; i < 4; i++) {
        int raw = readIRFiltered(PIN_IR[i]);
        sData.raw_ir[i] = raw;
        if (raw < frame_min_raw) frame_min_raw = raw;
        if (raw > frame_max_raw) frame_max_raw = raw;
    }

    int  frame_span     = frame_max_raw - frame_min_raw;
    bool valid_contrast = (frame_span > 500) && (frame_min_raw < 1300) && (frame_max_raw < 3950);
    bool slow_tick      = ((loopFrameCount & 0x1F) == 0);
    bool is_waiting     = (brain.mode == Brain::M_CROSS_WAIT);

    for (int i = 0; i < 4; i++) {
        int raw = sData.raw_ir[i];

        if (!is_waiting && valid_contrast && brain.off_track_t < 0.04f) {
            if (raw > 20 && raw < ir_white_cal[i]) {
                ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            } else if (slow_tick && raw < ir_white_cal[i] + 250 && ir_white_cal[i] < 1100) {
                ir_white_cal[i] += 1;
            }
            if (raw > ir_white_cal[i] + 600 && raw < 3850) {
                if (raw > ir_black_cal[i]) ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
            }
        }

        int min_valid_black = ir_white_cal[i] + 650;
        if (!is_waiting && slow_tick && ir_black_cal[i] > min_valid_black + 250) ir_black_cal[i] -= 3;
        if (ir_black_cal[i] < min_valid_black) ir_black_cal[i] = min_valid_black;

        int effective_span = ((ir_black_cal[i] - ir_white_cal[i]) * 86) / 100;
        if (effective_span < 500) effective_span = 500;

        float norm = (float)(raw - ir_white_cal[i]) / (float)effective_span;
        sData.ir[i] = clampf(norm, 0.0f, 1.0f);
    }

    int pwm_l = 0, pwm_r = 0;
    brain.step(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);

    // Bật đèn LED trắng (chân 20) khi xe đang dừng 5 giây ở ngã tư để báo hiệu
    digitalWrite(W_LED_ON, (brain.mode == Brain::M_CROSS_WAIT) ? HIGH : LOW);

#if ENABLE_BLACKBOX
    if ((loopFrameCount % 10) == 0 && !brain.was_lifted) bbRecord(sData, pwm_l, pwm_r);
    if (brain.was_lifted && !bb_dumped && bb_n > 0) { bbDump(); bb_dumped = true; }
    if (!brain.was_lifted) bb_dumped = false;
#endif

#if ENABLE_SERIAL_TELEMETRY
    if (brain.ev_cross) {
        brain.ev_cross = false;
        Serial.printf("[CROSS] #%d act=%+d odo=%.0f mm center_odo=%.0f skew_sec=%.2f\n",
                      brain.ev_idx, brain.ev_act, brain.dist_mm, brain.ev_center_d, brain.cross_sec);
    }
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 120 && Serial.availableForWrite() > 64) {
        lastTelemetryMillis = nowMs;
        Serial.printf("[t=%5.2f] M%d x=%+5.1f vx=%+6.0f u=%+4.0f | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, (int)brain.mode, brain.x, brain.vx, brain.u_cmd,
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3], pwm_l, pwm_r);
    }
#endif
}
