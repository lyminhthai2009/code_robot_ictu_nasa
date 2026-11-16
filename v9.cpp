// ============================================================================
//  COBOT LINE FOLLOWER v5 - ESP32-C3 - vòng lặp 500 Hz
//  v5 = v4 + Nâng cấp toàn diện "Bám cua gắt - Siêu mượt - Thích ứng đa màu nền":
//    1) [V5-MEAS]  Xóa điểm mù đạo hàm ở cảm biến rìa S1/S4 (Shoulder + Edge continuity)
//    2) [V5-BRAKE] Phanh đón đầu trước cua theo dự báo vị trí (x_pred) & tốc độ lệch (div_rate)
//    3) [V5-PID]   PD bất đối xứng theo pha (Vào cua gắt vs Trả lái êm) + Lọc đạo hàm 6ms
//    4) [V5-FLOOR] Sàn bánh trong 2 tầng: êm ở cua thường (>=0), phanh ngược chủ động ở cua gắt (-38)
//    5) [V5-SURF]  Thích ứng màu nền 2 lớp: Khử nền đồng pha tức thì (2ms) + Hiệu chuẩn liên kết 4 mắt
//  Giữ nguyên toàn bộ chân phần cứng, tốc độ V_MAX / V_CORNER_MIN và chiến thuật ngã tư [V4].
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
#define ENABLE_SHORTCUT          1   // 1 = rẽ tắt theo CROSS_PLAN | 0 = luôn đi thẳng qua mọi ngã tư
#define ENABLE_SERIAL_TELEMETRY  0   // 1 = in log 8 Hz (chỉ để debug, tắt khi thi)
#define ENABLE_BLACKBOX          0   // 1 = ghi dữ liệu vào RAM, xả qua USB khi nhấc xe (để chỉnh thông số)

// Hành động ở từng lần gặp ngã tư (theo thứ tự gặp):
//   0 = đi thẳng,  +1 = rẽ về phía S4 (phía sai số dương),  -1 = rẽ về phía S1
#define TURN_RIGHT  (+1)   // về phía S4
#define TURN_LEFT   (-1)   // về phía S1
static const int8_t CROSS_PLAN[] = { 0, 0, TURN_RIGHT };   // lần 1: thẳng | lần 2: thẳng | lần 3: rẽ phải
static const int    CROSS_PLAN_LEN = (int)(sizeof(CROSS_PLAN) / sizeof(CROSS_PLAN[0]));

// ---------------------------- Hình học xe (mm) -----------------------------
static const float SENSOR_X_MM[4]       = { -20.0f, -11.25f, 11.25f, 20.0f };
static const float AXLE_TO_OUTER_ROW_MM = 57.0f;   // trục bánh -> hàng S1/S4
static const float ASSUMED_LINE_W_MM    = 30.0f;   // bề rộng vạch giả định (dùng để canh tâm ngã tư)
static const float SPEED_MMPS_AT_255    = 500.0f;  // tốc độ thật (mm/s) khi PWM=255 trên sàn

// ---------------------------- Tốc độ (GIỮ NGUYÊN NỀN) ----------------------
static const float V_MAX         = 250.0f;   // tốc độ nền tối đa trên đường thẳng
static const float V_CORNER_MIN  = 150.0f;   // sàn tốc độ nền khi vào cua
static const float V_START       = 70.0f;    // tốc độ khởi động
static const float V_SEEK_LINE   = 100.0f;   // đi chậm khi chưa thấy vạch
static const float ACC_UP        = 650.0f;   // PWM/s, tăng tốc mượt (chống giật & sụt áp pin)
static const float ACC_DN        = 6500.0f;  // PWM/s, giảm tốc cực nhanh ngay khi phát hiện chớm vào cua
static const float K_SPD_U       = 0.85f;    // giảm tốc theo nỗ lực lái
static const float K_SPD_E       = 2.6f;     // giảm tốc theo sai số vị trí thực + dự báo
static const float K_SPD_VX      = 0.22f;    // [V5] giảm tốc sớm theo tốc độ vạch lao ra mép (mm/s)
static const float E_SPD_DEAD_MM = 4.5f;     // vùng chết sai số trên đường thẳng
static const float VX_SPD_DEAD   = 55.0f;    // mm/s: ngưỡng tốc độ lệch bắt đầu phanh đón đầu
static const float F_IN_MIN      = 30.0f;    // sàn bánh trong khi đi thẳng / cua nhẹ

// ---------------------------- PID Bất đối xứng (đơn vị mm) -----------------
static const float KP            = 7.6f;     // PWM / mm (lực giữ tâm cơ sở)
static const float KP_NL_MM      = 32.0f;    // Kp_eff tăng nhanh khi lệch lớn để bẻ mũi xe sớm
static const float KD_OUT        = 0.34f;    // [V5] PWM*s/mm khi vạch đang RỜI TÂM (vào cua: cần bẻ sớm)
static const float KD_IN         = 0.42f;    // [V5] PWM*s/mm khi vạch đang VỀ TÂM (trả lái: chống vọt lố, siêu mượt)
static const float VX_FILTER_TAU = 0.006f;   // [V5] s (6ms) lọc nhiễu lượng tử trên đạo hàm vx
static const float KI            = 52.0f;    // PWM / (mm*s) bù độ cong khi ôm cua dài
static const float I_MAX         = 75.0f;    // trần tích phân độ cong
static const float I_LEAK_TAU    = 0.85f;    // s: xả dần khi đi thẳng
static const float I_ACTIVE_MM   = 28.0f;    // [V5] tích phân trên toàn dải cảm biến để giữ lực ôm cua

// ---------------------------- Bộ lọc vị trí alpha-beta ---------------------
static const float AB_ALPHA = 0.44f;         // bám vị trí nhạy hơn ở đầu cua
static const float AB_BETA  = 0.095f;        // bắt vận tốc ngang vx nhanh hơn
static const float JUMP_MM  = 26.0f;

// ---------------------------- Ước lượng vị trí liên tục [V5] ---------------
static const float W_FLOOR       = 0.07f;
static const float WSUM_MIN      = 0.08f;
static const float K_SHOULDER_MM = 3.8f;     // [V5] bù độ dốc khi vạch lướt từ S2/S3 sang S1/S4 (xóa điểm mù 16..21mm)
static const float K_EXT_MM      = 9.5f;     // ngoại suy khi vạch trượt ra mép ngoài S1/S4
static const float X_LIM_MM      = 32.0f;

// ---------------------------- Mất vạch & Bắt lại vạch ----------------------
static const float GAP_X_MM       = 10.0f;
static const float GAP_HOLD_MM    = 50.0f;
static const float V_GAP_MAX      = 230.0f;
static const float V_LOST         = 125.0f;  // giảm tốc khi mất vạch ở cua gắt để quay nhanh hơn
static const float U_LOST_MAX     = 140.0f;  // lực bẻ tối đa khi tìm lại vạch
static const float LOST_RAMP_S    = 0.08f;   // [V5] tăng lực tìm vạch nhanh hơn (80ms)
static const float LOST_GIVEUP_S  = 1.2f;
static const float REENTRY_T      = 0.09f;   // [V5] cửa sổ giảm chấn êm khi vừa bắt lại vạch (90ms)

// ---------------------------- Ngã tư [V4 + V5 Adaptive] --------------------
static const float WING_ON               = 0.50f;   // ngưỡng chuẩn khi nền = 0
static const float WING_OFF              = 0.34f;   // ngưỡng rời vạch ngang
static const float WING_BASE_MAX         = 0.38f;   // [V5] nới trần nền cánh để chịu được lúc đang chuyển màu sàn
static const float WING_DELTA_ON         = 0.36f;   // [V5] cánh phải cao hơn nền thực tế của chính nó >= giá trị này
static const float WING_SEP_ON           = 0.76f;   // 2 cảm biến cách nhau >= 31 mm cùng đen = 2 vạch riêng biệt
static const int   WING_DEBOUNCE_TICKS   = 3;       // cánh đen liên tục >= 3 tick (6 ms) mới tính
static const float WING_LATCH_MM         = 50.0f;   // hai cánh chạm vạch ngang cách nhau tối đa 50 mm
static const float CROSS_MIN_SPACING_MM  = 250.0f;  // hai ngã tư liên tiếp cách nhau >= 250 mm
static const float CROSS_PASS_MAX_MM     = 200.0f;
static const float CROSS_PASS_MAX_S      = 0.80f;
static const float CROSS_EXIT_MARGIN_MM  = 15.0f;
static const float U_REF_TAU             = 0.20f;

// ---------------------------- Rẽ tắt (pivot) [V4] --------------------------
static const float V_ALIGN           = 150.0f;
static const float ALIGN_FALLBACK_MM = 200.0f;
static const float ALIGN_FALLBACK_S  = 0.90f;
static const float P_CLEAR           = 140.0f;
static const float P_SEEK            = 100.0f;
static const float PIVOT_OLD_ON      = 0.50f;
static const float PIVOT_OLD_GONE    = 0.25f;
static const float PIVOT_NEW_OUTER   = 0.50f;
static const float PIVOT_RELAX_T     = 0.30f;
static const float T_PIVOT_MAX       = 0.90f;
static const float PIVOT_SWEEP_MMPS  = 300.0f;
static const float HANDOVER_T        = 0.35f;
static const float V_HANDOVER        = 80.0f;
static const float F_IN_HANDOVER     = -50.0f;
static const float U_MAX_HANDOVER    = 90.0f;

// ---------------------------- [V5] Lái chủ động 2 tầng khi vào cua ---------
// Tầng 1 (0.0 < risk <= 0.55): hạ sàn bánh trong êm ái từ F_IN_MIN (+30) về 0.0 (không đảo chiều -> siêu mượt).
// Tầng 2 (0.55 < risk <= 1.0): mở khóa phanh ngược chủ động từ 0.0 xuống F_IN_EMERG (-38) để cứu các góc cua gắt.
static const float T_PRED        = 0.055f;  // s   nhìn trước 55ms để dự báo vị trí vạch
static const float RISK_X0       = 7.5f;    // mm  |x_dự báo| < giá trị này: nguy cơ = 0 (đi thẳng/cua nhẹ)
static const float RISK_X1       = 19.0f;   // mm  |x_dự báo| >= giá trị này: nguy cơ = 1 (cua gắt)
static const float RISK_BRAKE_TH = 0.55f;   // ngưỡng nguy cơ bắt đầu cho phép bánh trong phanh ngược (< 0)
static const float F_IN_EMERG    = -38.0f;  // sàn bánh trong khi nguy cơ = 1 (phanh chủ động giữ chặt line)

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

// Bộ giới hạn gia tốc đầu ra (đơn vị: duty sau deadzone, mỗi tick 2 ms)
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
    float ir[4];      // 0.0 = màu nền hiện tại, 1.0 = vạch đen (đã khử trôi nền)
    int   raw_ir[4];
    float elapsed_sec;
};

// Trộn bánh: ưu tiên GIỮ chênh lệch (lái), điều chỉnh tốc độ nền tối thiểu.
// Đảm bảo: bánh ngoài <= 255, bánh trong >= f_in.
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
        M_CROSS_PASS,      // đi thẳng qua ngã tư: đóng băng lái
        M_TURN_ALIGN,      // rẽ tắt: giảm tốc, tiến thẳng tới khi trục bánh trùng tâm ngã tư
        M_TURN_PIVOT       // pivot tại chỗ: quay tới khi cánh ngoài chạm nhánh mới
    };

    Mode  mode;
    float mode_t, mode_d;

    float x, vx, vx_f;
    bool  need_reinit;
    float integ, u_abs_f, u_cmd;
    float v_cmd, v_est, dist_mm;
    int   last_side;

    bool  in_loss, meas_ok;
    float lost_t, lost_d, lost_x;
    int   lost_side;
    float handover_t, reentry_t;

    bool  was_lifted, has_seen_line;
    float off_track_t, run_t;

    // [V4] theo dõi hai cánh S1 (k=0) và S4 (k=1); mọi khoảng cách là odometer (mm)
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
    bool  pivot_old_seen;
    // sự kiện cho telemetry
    bool  ev_cross; int ev_idx, ev_act; float ev_center_d;

    int   prev_pl, prev_pr;

    Brain() { reset(); }

    void reset() {
        mode = M_RUN; mode_t = 0.0f; mode_d = 0.0f;
        x = 0.0f; vx = 0.0f; vx_f = 0.0f; need_reinit = true;
        integ = 0.0f; u_abs_f = 0.0f; u_cmd = 0.0f;
        v_cmd = V_START; v_est = 0.0f; dist_mm = 0.0f;
        last_side = 1;
        in_loss = false; meas_ok = false;
        lost_t = 0.0f; lost_d = 0.0f; lost_x = 0.0f; lost_side = 1;
        handover_t = 0.0f; reentry_t = 0.0f;
        was_lifted = false; has_seen_line = false;
        off_track_t = 0.0f; run_t = 0.0f;
        base_init = false; base_o[0] = base_o[1] = 0.0f;
        cross_confirm = 0; cross_count = 0; last_cross_dist = -1.0e6f;
        clear_ticks = 0; white_ticks = 0; turn_dir = 1; pivot_old_seen = false;
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
        bool spinning  = (mode == M_TURN_PIVOT);

        if ((all_white || all_black) && !spinning) {
            off_track_t += dt;
            if (off_track_t > 0.85f) was_lifted = true;
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
        float c = 0.5f * (float)(prev_pl + prev_pr);
        if (c < 0.0f) c = 0.0f;
        float tgt = SPEED_MMPS_AT_255 * c / 255.0f;
        v_est += (tgt - v_est) * (dt / (0.04f + dt));
        dist_mm += v_est * dt;
    }

    // ---- [V5-MEAS] Ước lượng vị trí vạch liên tục (xóa điểm mù đạo hàm 16..22mm) ----
    bool measure(const float* s, float& z) const {
        // Khử sàn nền vi sai cục bộ trong khung hình (bảo vệ thêm khi nền vừa đổi tức thì)
        float s_min = min4f(s[0], s[1], s[2], s[3]);
        float s_max = max4f(s[0], s[1], s[2], s[3]);
        float bg = (s_max - s_min > 0.28f && s_min < 0.45f) ? s_min : 0.0f;
        float inv_span = 1.0f / (1.0f - bg - W_FLOOR);

        float w[4], wsum = 0.0f;
        for (int i = 0; i < 4; i++) {
            float c = (s[i] - bg - W_FLOOR) * inv_span;
            w[i] = (c > 0.0f) ? min2f(c, 1.0f) : 0.0f;
            wsum += w[i];
        }
        if (wsum < WSUM_MIN) return false;

        float num = 0.0f;
        for (int i = 0; i < 4; i++) num += w[i] * SENSOR_X_MM[i];
        float xc = num / wsum;

        // [V5] Bù độ dốc vai (Shoulder) + Ngoại suy mép ngoài (Outer Edge)
        // Khi vạch dịch từ S3(11.25) ra S4(20.0), w[2] giảm dần về 0 trong khi w[3] vẫn ~1.0.
        // Shoulder boost giữ cho đạo hàm dz/dx luôn dương và liên tục suốt từ 15mm -> 32mm.
        if (w[3] >= w[0]) {
            float opp = w[0] + w[1];
            if (w[3] > w[2] && opp < 0.20f) {
                float sh = (w[3] - w[2]) / (w[3] + w[2] + 0.05f);
                xc += K_SHOULDER_MM * sh * clampf(1.0f - opp / 0.20f, 0.0f, 1.0f);
            }
            float inner = w[0] + w[1] + w[2];
            float b = clampf(1.0f - inner / 0.32f, 0.0f, 1.0f);
            xc += b * K_EXT_MM * (1.0f - min2f(w[3], 1.0f));
        } else {
            float opp = w[2] + w[3];
            if (w[0] > w[1] && opp < 0.20f) {
                float sh = (w[0] - w[1]) / (w[0] + w[1] + 0.05f);
                xc -= K_SHOULDER_MM * sh * clampf(1.0f - opp / 0.20f, 0.0f, 1.0f);
            }
            float inner = w[1] + w[2] + w[3];
            float b = clampf(1.0f - inner / 0.32f, 0.0f, 1.0f);
            xc -= b * K_EXT_MM * (1.0f - min2f(w[0], 1.0f));
        }
        z = clampf(xc, -X_LIM_MM, X_LIM_MM);
        return true;
    }

    // ---- [V4 + V5] Theo dõi hai cánh S1, S4 thích ứng nền ---------------------
    void updateWingBase(const float* s, float dt) {
        if (!base_init) { base_o[0] = s[0]; base_o[1] = s[3]; base_init = true; }
        const float ab = dt / (0.10f + dt);   // [V5] bám nền cánh nhanh hơn khi đổi màu sàn
        base_o[0] += ((s[0] < 0.45f) ? ab : ab * 0.10f) * (s[0] - base_o[0]);
        base_o[1] += ((s[3] < 0.45f) ? ab : ab * 0.10f) * (s[3] - base_o[1]);
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
            float th_on  = max2f(WING_ON,  base_o[k] + WING_DELTA_ON);
            float th_off = max2f(WING_OFF, base_o[k] + 0.18f);
            if (w_on[k]) {
                w_last_d[k] = dist_mm;
                if (v < th_off) { w_on[k] = false; w_fell[k] = true; w_fall_d[k] = dist_mm; }
            } else if (!w_rose[k]) {
                if (v > th_on && base_o[k] < WING_BASE_MAX) {
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
            if ((s[0] > WING_SEP_ON && s[2] > WING_SEP_ON) || (s[1] > WING_SEP_ON && s[3] > WING_SEP_ON)) w_sep = true;
        } else {
            w_sep = false;
        }
        updateWingBase(s, dt);
    }

    bool detectCross(const float* s, bool allowed) {
        bool center = (s[1] > 0.45f || s[2] > 0.45f);
        bool trig = allowed && w_rose[0] && w_rose[1] && w_sep && center;
        cross_confirm = trig ? (cross_confirm + 1) : 0;
        return cross_confirm >= 2;
    }

    void planCross() {
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
        last_cross_dist = dist_mm;
        u_hold = u_ref;
        clear_ticks = 0; white_ticks = 0;
        mode_t = 0.0f; mode_d = 0.0f;
        cross_confirm = 0;
        planCross();
        ev_cross = true; ev_idx = idx + 1; ev_act = act; ev_center_d = center_odo;
        if (act == 0) {
            mode = M_CROSS_PASS;
        } else {
            turn_dir = (act > 0) ? 1 : -1;
            mode = M_TURN_ALIGN;
        }
    }

    // ---- RUN: cập nhật bộ lọc, mất vạch, ngã tư ----
    void runEstimate(const float* s, float dt) {
        bool allowed = (handover_t <= 0.0f) && !in_loss && (run_t > 0.15f) &&
                       ((dist_mm - last_cross_dist) >= CROSS_MIN_SPACING_MM);
        if (allowed) trackWings(s, dt);
        else { clearWings(); updateWingBase(s, dt); }
        bool crossing = detectCross(s, allowed);

        float z = 0.0f;
        meas_ok = measure(s, z);
        if (meas_ok) {
            if (in_loss) {
                in_loss = false;
                // [V5] Khi vừa bắt lại vạch từ ngoài rìa, gán vận tốc quét vào tâm để KD_IN phanh mượt, chống vọt lố!
                if (lost_t > 0.012f) {
                    x = z;
                    vx = -(float)lost_side * clampf(160.0f + 250.0f * lost_t, 140.0f, 320.0f);
                    vx_f = vx;
                    need_reinit = false;
                    reentry_t = REENTRY_T;
                }
                if (lost_t > 0.12f) integ *= 0.35f;
                lost_t = 0.0f; lost_d = 0.0f;
            }
            if (!need_reinit) {
                float xp = x + vx * dt;
                float r  = z - xp;
                if (fabsf(r) > JUMP_MM) {
                    x = z; vx = 0.0f; vx_f = 0.0f; need_reinit = false;
                } else {
                    x  = xp + AB_ALPHA * r;
                    vx += (AB_BETA / dt) * r;
                    vx = clampf(vx, -1600.0f, 1600.0f);
                    vx_f += (vx - vx_f) * (dt / (VX_FILTER_TAU + dt));
                }
            } else {
                x = z; vx = 0.0f; vx_f = 0.0f; need_reinit = false;
            }
            if (fabsf(x) > 2.5f) last_side = (x > 0.0f) ? 1 : -1;
        } else {
            if (!in_loss) {
                in_loss = true; lost_t = 0.0f; lost_d = 0.0f; lost_x = x;
                lost_side = (fabsf(x) > 1.5f) ? ((x > 0.0f) ? 1 : -1) : last_side;
            }
            lost_t += dt;
            lost_d += v_est * dt;
            x = clampf(x + vx * dt, -X_LIM_MM, X_LIM_MM);
            vx *= 0.95f;
            vx_f = vx;
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
        if (reentry_t > 0.0f) reentry_t -= dt;

        float u = 0.0f, v_t = V_MAX;

        if (!in_loss) {
            float ae = fabsf(x);
            bool  diverging = (x * vx_f >= 0.0f);   // true = vạch đang trôi ra xa tâm (vào cua)

            // [V5] Dự báo vị trí vạch sau T_PRED (55ms)
            float x_pred  = x + T_PRED * vx_f;
            float ae_pred = fabsf(x_pred);
            float risk_x  = max2f(ae, diverging ? ae_pred : (0.75f * ae_pred));
            float risk    = clampf((risk_x - RISK_X0) / (RISK_X1 - RISK_X0), 0.0f, 1.0f);

            // [V5-FLOOR] Sàn bánh trong 2 tầng:
            //   - risk <= 0.55: hạ êm từ F_IN_MIN (+30) về 0 (không đảo chiều -> xe lướt siêu mượt)
            //   - risk >  0.55: phanh ngược chủ động từ 0 xuống F_IN_EMERG (-38) khi vào cua gắt
            if (!handover) {
                if (risk <= RISK_BRAKE_TH) {
                    float r1 = risk / RISK_BRAKE_TH;
                    f_in = F_IN_MIN * (1.0f - r1);
                } else {
                    float r2 = (risk - RISK_BRAKE_TH) / (1.0f - RISK_BRAKE_TH);
                    f_in = F_IN_EMERG * r2;
                }
                a_cap = 0.5f * (255.0f - f_in);
            }

            // [V5-PID] Bất đối xứng theo pha:
            //   - Đang vào cua (diverging): KP phi tuyến mạnh + KD_OUT để bẻ mũi xe sớm.
            //   - Đang trả lái về tâm (!diverging): làm dịu KP + tăng giảm chấn KD_IN để triệt tiêu vọt lố.
            float nl_scale = diverging ? (ae / KP_NL_MM) : (0.60f * ae / KP_NL_MM);
            float kp = KP * (1.0f + nl_scale);
            float kd = (diverging && reentry_t <= 0.0f) ? KD_OUT : KD_IN;
            if (reentry_t > 0.0f) kd *= 1.15f;   // tăng giảm chấn khi vừa bắt lại line

            // Tích phân độ cong toàn dải + xả nhanh khi đổi dấu sai số (thoát cua)
            float u_pd = kp * x + kd * vx_f;
            float u_raw = u_pd + integ;
            bool  sat = fabsf(u_raw) > a_cap;
            if (ae < I_ACTIVE_MM && !(sat && ((u_raw > 0.0f) == (x > 0.0f)))) {
                integ += KI * x * dt;
            }
            float leak_tau = ((x * integ) < 0.0f) ? (0.25f * I_LEAK_TAU) : I_LEAK_TAU;
            integ -= integ * (dt / leak_tau);
            integ = clampf(integ, -I_MAX, I_MAX);

            u = clampf(u_pd + integ, -a_cap, a_cap);

            // [V5-BRAKE] Lọc nỗ lực lái bất đối xứng: tăng nhanh (12ms) để phanh sớm đầu cua, giảm êm (85ms)
            float u_abs = fabsf(u);
            float tau_u = (u_abs > u_abs_f) ? 0.012f : 0.085f;
            u_abs_f += (u_abs - u_abs_f) * (dt / (tau_u + dt));

            // Tốc độ lệch ra mép dùng để phanh đón đầu ngay khi chớm vào cua
            float div_rate = diverging ? fabsf(vx_f) : (0.25f * fabsf(vx_f));
            float e_spd    = max2f(ae, 0.85f * ae_pred);

            v_t = V_MAX
                - K_SPD_U  * u_abs_f
                - K_SPD_E  * max2f(0.0f, e_spd - E_SPD_DEAD_MM)
                - K_SPD_VX * max2f(0.0f, div_rate - VX_SPD_DEAD);
            v_t = clampf(v_t, V_CORNER_MIN, V_MAX);
        } else {
            float lx = fabsf(lost_x);
            if (lx < GAP_X_MM && lost_d < GAP_HOLD_MM) {
                // Khe hở trên vạch thẳng: giữ hướng, giữ I
                u = clampf(integ, -a_cap, a_cap);
                v_t = min2f(v_cmd, V_GAP_MAX);
            } else {
                // Vạch rời khỏi dải cảm biến ở góc cua gắt: dùng sàn F_IN_EMERG để quay mũi xe bắt lại vạch ngay
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

        // [V4] lái "sạch": chỉ cập nhật khi chưa có cánh nào chạm vạch ngang
        if (!handover && !in_loss && !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0)
            u_ref += (u - u_ref) * (dt / (U_REF_TAU + dt));
    }

    // ---- Đi thẳng qua ngã tư ----
    void outCrossPass(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        mode_d += v_est * dt;
        trackWings(s, dt);
        u_hold -= u_hold * (dt / 0.10f);
        bool clear = w_fell[0] && w_fell[1] && (s[0] < 0.30f && s[3] < 0.30f);
        clear_ticks = clear ? (clear_ticks + 1) : 0;
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;
        if ((clear_ticks >= 3 && mode_d > 15.0f) || dist_mm >= end_odo ||
            mode_d > CROSS_PASS_MAX_MM || mode_t > CROSS_PASS_MAX_S) {
            mode = M_RUN;
            need_reinit = true;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f;
            last_cross_dist = dist_mm;
            clearWings();
        }
    }

    // ---- Rẽ tắt ----
    void outAlign(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        mode_d += v_est * dt;
        trackWings(s, dt);
        refineCenter();
        rampSpeed(V_ALIGN, dt);
        u_hold -= u_hold * (dt / 0.10f);
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
            x = z; vx = -(float)turn_dir * PIVOT_SWEEP_MMPS; vx_f = vx; need_reinit = false;
            integ = 0.0f; u_abs_f = 0.0f; u_cmd = 0.0f;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f;
            handover_t = HANDOVER_T; v_cmd = V_HANDOVER;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings();
            mode = M_RUN; mode_t = 0.0f;
            return;
        }
        if (mode_t > T_PIVOT_MAX) {
            x = 0.0f; vx = 0.0f; vx_f = 0.0f; need_reinit = true;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f;
            integ = 0.0f; handover_t = 0.0f; v_cmd = V_LOST;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings();
            mode = M_RUN; mode_t = 0.0f;
        }
    }
};

// ============================================================================
// 5. PHẦN CỨNG: ĐỘNG CƠ, CẢM BIẾN & THÍCH ỨNG MÀU NỀN [V5]
// ============================================================================
Brain brain;

int   ir_white_cal[4] = {300, 300, 300, 300};
int   ir_black_cal[4] = {2400, 2400, 2400, 2400};
int   ir_white_ofs[4] = {0, 0, 0, 0};   // [V5] độ lệch phần cứng giữa 4 mắt cảm biến trên cùng màu nền
float cm_bg_floor     = 0.0f;           // [V5] sàn nền đồng pha tức thời (khử đổi màu nền trong 2ms)

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

// Giới hạn gia tốc từng bánh. Đảo chiều: giảm về 0, nghỉ 2 tick (xả Back-EMF), rồi mới tăng ngược.
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

    if (global_min > 1600) global_min = 320;
    if (global_max < global_min + 600 || global_max > 3880) {
        global_max = constrain(global_min + 1500, 1400, 3200);
    }

    for (int i = 0; i < 4; i++) {
        if (min_seen[i] < global_min + 220) ir_white_cal[i] = min_seen[i];
        else                                ir_white_cal[i] = global_min;

        ir_white_ofs[i] = constrain(ir_white_cal[i] - global_min, -120, 120);

        if (max_seen[i] > ir_white_cal[i] + 600 && max_seen[i] < 3880) ir_black_cal[i] = max_seen[i];
        else ir_black_cal[i] = max(global_max, ir_white_cal[i] + 1200);
    }

    cm_bg_floor = 0.0f;
    brain.reset();
    startMicros    = micros();
    lastStepMicros = startMicros;
}

// ============================================================================
// 8. VÒNG LẶP 500 Hz + THÍCH ỨNG MÀU NỀN 2 LỚP [V5-SURF]
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

    int  frame_span = frame_max_raw - frame_min_raw;
    int  bg_thresh  = frame_min_raw + (frame_span >> 2);   // nhóm 25% thấp nhất trong khung hình = đang ở trên nền
    int  bg_cnt     = 0;
    int  bg_sum     = 0;
    for (int i = 0; i < 4; i++) {
        if (sData.raw_ir[i] <= bg_thresh) {
            bg_cnt++;
            bg_sum += sData.raw_ir[i];
        }
    }

    // [V5-SURF] Lớp 1: Hiệu chuẩn thích ứng nhanh & đồng bộ 4 cảm biến
    // Cho phép nền thay đổi rộng (từ trắng 200 tới nền màu/gỗ/xám 2200) miễn là vẫn có tương phản với vạch đen
    bool valid_contrast = (frame_span > 420) && (frame_min_raw < 2200) &&
                          (frame_max_raw < 3960) && (brain.off_track_t < 0.04f) &&
                          (brain.mode == Brain::M_RUN);
    bool slow_tick      = ((loopFrameCount & 0x0F) == 0);   // mỗi 16 tick (32 ms)
    int  bg_ref         = (bg_cnt > 0) ? (bg_sum / bg_cnt) : frame_min_raw;

    float raw_norm[4];
    for (int i = 0; i < 4; i++) {
        int raw = sData.raw_ir[i];

        if (valid_contrast) {
            if (raw <= bg_thresh) {
                // Cảm biến này đang nhìn trực tiếp màu nền hiện tại
                if (raw > 20 && raw < ir_white_cal[i]) {
                    // Nền sáng hơn: kéo xuống cực nhanh
                    ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) >> 2;
                } else if (bg_cnt >= 2 && raw > ir_white_cal[i]) {
                    // Nền tối hơn (đổi màu sàn): khi có >= 2 cảm biến cùng xác nhận nền, kéo lên nhanh (~30ms)
                    ir_white_cal[i] += (raw - ir_white_cal[i] + 7) >> 4;
                }
            } else if (bg_cnt >= 2) {
                // Cảm biến này đang nằm trên vạch đen: kéo đồng bộ ngưỡng trắng theo các mắt đang ở trên nền
                int target_w = constrain(bg_ref + ir_white_ofs[i], 40, 2100);
                ir_white_cal[i] += (target_w - ir_white_cal[i]) >> 5;
            }

            // Cập nhật mức đen khi cảm biến nằm ở đỉnh vạch đen
            if (raw >= frame_max_raw - (frame_span >> 3) && raw > ir_white_cal[i] + 500 && raw < 3900) {
                if (raw > ir_black_cal[i]) {
                    ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) >> 2;
                } else if (slow_tick) {
                    ir_black_cal[i] += (raw - ir_black_cal[i]) >> 4;
                }
            }
        }

        int min_valid_black = ir_white_cal[i] + 550;
        if (slow_tick && ir_black_cal[i] > min_valid_black + 300) ir_black_cal[i] -= 2;
        if (ir_black_cal[i] < min_valid_black) ir_black_cal[i] = min_valid_black;

        int effective_span = ((ir_black_cal[i] - ir_white_cal[i]) * 86) / 100;
        if (effective_span < 450) effective_span = 450;

        float norm = (float)(raw - ir_white_cal[i]) / (float)effective_span;
        raw_norm[i] = clampf(norm, 0.0f, 1.0f);
    }

    // [V5-SURF] Lớp 2: Khử nền đồng pha tức thì (2ms Common-Mode Floor Suppression)
    // Ngay khoảnh khắc xe bước qua ranh giới 2 màu nền (chưa cần đợi bộ lọc hiệu chuẩn hội tụ),
    // ít nhất 2 cảm biến ngoài vạch đen sẽ báo mức sàn mới (n_min). Trừ thẳng sàn này để giữ nguyên độ nhạy PID!
    float n_min = min4f(raw_norm[0], raw_norm[1], raw_norm[2], raw_norm[3]);
    float n_max = max4f(raw_norm[0], raw_norm[1], raw_norm[2], raw_norm[3]);
    int   n_bg_cnt = 0;
    for (int i = 0; i < 4; i++) {
        if (raw_norm[i] <= n_min + 0.18f) n_bg_cnt++;
    }

    if (brain.mode == Brain::M_RUN && (n_max - n_min) > 0.28f && n_min < 0.52f && n_bg_cnt >= 2) {
        // Có cả vạch đen lẫn >= 2 cảm biến nền -> cập nhật sàn nền tức thời
        float alpha_cm = (n_min > cm_bg_floor) ? 0.45f : 0.60f;
        cm_bg_floor += alpha_cm * (n_min - cm_bg_floor);
    } else if (n_min < cm_bg_floor) {
        cm_bg_floor = n_min;
    } else {
        cm_bg_floor *= 0.96f;
    }
    cm_bg_floor = clampf(cm_bg_floor, 0.0f, 0.48f);

    float inv_cm = 1.0f / (1.0f - cm_bg_floor);
    for (int i = 0; i < 4; i++) {
        sData.ir[i] = clampf((raw_norm[i] - cm_bg_floor) * inv_cm, 0.0f, 1.0f);
    }

    int pwm_l = 0, pwm_r = 0;
    brain.step(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);

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
        Serial.printf("[t=%5.2f] M%d x=%+5.1f vx=%+6.0f u=%+4.0f bg=%.2f | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, (int)brain.mode, brain.x, brain.vx_f, brain.u_cmd, cm_bg_floor,
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3], pwm_l, pwm_r);
    }
#endif
}
