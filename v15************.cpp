// ============================================================================
//  COBOT LINE FOLLOWER v6.0 - ESP32-C3 - 500 Hz (THÁI NGUYÊN ROBOTICS 2026)
//  NÂNG CẤP TOÀN DIỆN TỪ v5.1 LÊN v6.0 [MAX SPEED + ROLLING ARC + ZERO SAG]:
//
//  1) [V6-VORONOI] KHỬ MÉO BẬC THANG MẢNG HÌNH THANG S1..S4 (-20, -11.25, +11.25, +20):
//     - Khoảng cách S2-S3 (22.5mm) rộng gấp 2.57 lần S1-S2 và S3-S4 (8.75mm).
//     - Nhân trọng số ô Voronoi [0.70, 1.25, 1.25, 0.70] theo quy tắc tích phân hình thang,
//       triệt tiêu hoàn toàn sai số bậc thang 3.44mm tại S2/S3, giúp vx mượt gấp 3 lần.
//
//  2) [V6-DIRECTIONAL-RISK & OUTER-PINNED] CUA SIÊU NHANH KHÔNG PHẠT TỐC ĐỘ KÉP:
//     - Sửa lỗi toán học risk_v: chỉ tính nguy cơ theo vận tốc THOÁT LY khỏi tâm (v_out = sgn(x)*vx).
//       Ngay khi xe bắt đầu kéo vạch quay về tâm (v_out < 0), risk_v hạ về 0 tức thì để thốc ga thoát cua!
//     - Khử "phạt tốc độ kép": khi vào cua bình thường, giữ v_cmd sát 255 để mixWheels() tự động
//       ghim bánh ngoài ở 255 (tốc độ tối đa vật lý) và chỉ giảm bánh trong theo lực lái u.
//
//  3) [V6-ARC-TURN] RẼ NGÃ TƯ VÒNG TRÒN 2 BẰNG "ROLLING ARC TURN" (KHÔNG ĐẢO CHIỀU BÁNH):
//     - Thay vì phanh về 62, đợi 43mm rồi dừng hẳn quay ngược bánh (-140) gây sụt áp pin AA,
//       xe chỉ tiến 16mm rồi khóa bánh trong về 0, phóng bánh ngoài +215 để quét vòng cung tốc độ cao
//       quanh tâm bánh trong! Chỉ dùng lực đảo chiều nhẹ (-75) nếu góc nhánh quá gắt sau 0.16s.
//
//  4) [V6-SPEEDBUMP-GATE] CHỐNG NHẬN NHẦM GỜ GIẢM TỐC 1 & 2 (10 VẠCH XANH) THÀNH NGÃ TƯ:
//     - Cửa sổ khóa hành trình động riêng cho từng đoạn (Cross #2 -> Cross #3 khóa 900mm để băng qua
//       Gờ giảm tốc 1 tuyệt đối an toàn) + Kiểm định bề dày xung tối thiểu (WING_MIN_CHORD_MM).
//
//  5) [V6-BEMF-LIMITER] CHỐNG SỤT ÁP TUYỆT ĐỐI TRÊN 4 PIN AA KHÔNG SẠC:
//     - Mô hình hóa sức phản điện động (Back-EMF) thời gian thực của từng động cơ N20 (tau = 42ms).
//     - Giới hạn tổng dòng tức thời (PWM - BEMF) của cả 2 động cơ cộng lại, ưu tiên giữ 100% vi sai lái.
//     - Khi xe đang chạy nhanh (BEMF cao), PWM đáp ứng tức thời trong 1 tick (2ms) không bị trễ slew-rate!
//
//  6) [V6-BUGFIX] Sửa lỗi off_track_t > 0.65s vô hiệu hóa LOST_GIVEUP_S (1.20s) và cập nhật v_est từ PWM thực.
// ============================================================================
#include <Arduino.h>
#include <math.h>
#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
// 1. CẤU HÌNH CHIẾN THUẬT & SA HÌNH THÁI NGUYÊN 2026
// ============================================================================
#define ENABLE_SHORTCUT          1   // 1 = rẽ tắt ở Vòng tròn 2 theo CROSS_PLAN | 0 = đi thẳng mọi ngã tư
#define ENABLE_FLOOR_TRACK       1   // 1 = tự thích ứng khi màu nền đổi / bóng đổ
#define ENABLE_SERIAL_TELEMETRY  0   // 1 = in log 8 Hz (tắt khi thi đấu để tối ưu 100% CPU)
#define ENABLE_BLACKBOX          0   // 1 = ghi dữ liệu vào RAM, xả qua USB khi nhấc xe

// Kế hoạch qua các ngã tư trên sa hình (Vòng bảng & Vòng chung kết có cùng cấu trúc):
//   Lần 1 (Vào Vòng tròn 1): Đi thẳng (0)
//   Lần 2 (Ra Vòng tròn 1) : Đi thẳng (0)
//   Lần 3 (Vòng tròn 2)    : Rẽ phải đi tắt (+1)
#define TURN_RIGHT  (+1)   // về phía S4
#define TURN_LEFT   (-1)   // về phía S1
static const int8_t CROSS_PLAN[]   = { 0, 0, TURN_RIGHT };
static const int    CROSS_PLAN_LEN = (int)(sizeof(CROSS_PLAN) / sizeof(CROSS_PLAN[0]));

// Khoảng cách tối thiểu (mm) trước khi cho phép bắt ngã tư tiếp theo [V6-SPEEDBUMP-GATE]:
//   - Trước Ngã tư 1 (từ vạch START): >= 220 mm
//   - Từ Ngã tư 1 -> Ngã tư 2 (vòng trong Vòng tròn 1, chu vi thật ~1250mm): >= 420 mm
//   - Từ Ngã tư 2 -> Ngã tư 3 (phải đi qua Gờ giảm tốc 1 + Cua C + Gờ giảm tốc 2 ~2500mm): >= 900 mm
//   - Sau Ngã tư 3 (về đích): >= 500 mm
static const float CROSS_MIN_DIST_TABLE[4] = { 220.0f, 420.0f, 900.0f, 500.0f };

// ---------------------------- Hình học xe & Cảm biến Hình thang (mm) -------
static const float SENSOR_X_MM[4]       = { -20.0f, -11.25f, 11.25f, 20.0f }; // S1 (trái) ... S4 (phải)
// [V6-VORONOI] Trọng số bề rộng không gian Voronoi bù khoảng cách không đều (8.75mm - 22.5mm - 8.75mm)
static const float SENSOR_VORONOI_W[4]  = { 0.70f, 1.25f, 1.25f, 0.70f };
static const float SENSOR_Y_OFFSET_MM   = 4.84f;   // S2, S3 nhô về phía trước 4.84mm so với S1, S4
static const float AXLE_TO_OUTER_ROW_MM = 43.0f;   // trục bánh xe -> hàng S1/S4
static const float DEFAULT_LINE_W_MM    = 24.0f;   // bề rộng vạch chuẩn trên bạt in sa hình (mm)
static const float SPEED_MMPS_AT_255    = 540.0f;  // tốc độ danh định ở PWM 255 (tự hiệu chỉnh qua v_scale)

// ---------------------------- Tốc độ & Quy hoạch vận tốc [V6] --------------
static const float V_MAX         = 255.0f;   // tốc độ tối đa trên đường thẳng & bánh ngoài khi ôm cua
static const float V_CORNER_MIN  = 210.0f;   // [V6] giữ nền cao để mixWheels ghim bánh ngoài ở 255 khi cua thường!
static const float V_BRAKE_MIN   = 135.0f;   // sàn tốc độ khi vạch thực sự có nguy cơ văng khỏi S1/S4 (risk_v = 1)
static const float V_START       = 130.0f;   // tốc độ khởi hành ban đầu
static const float ACC_UP        = 1600.0f;  // PWM/s: thoát cua cực bốc (đã có BEMF limiter bảo vệ sụt áp)
static const float ACC_DN_SOFT   = 1800.0f;  // PWM/s: giảm tốc êm khi risk_v nhỏ
static const float ACC_DN        = 6000.0f;  // PWM/s: phanh khẩn cấp khi risk_v -> 1
static const float K_SPD_U       = 0.18f;    // [V6] giảm nhẹ vì mixWheels() đã tự giảm tâm xe theo |u|
static const float K_SPD_E       = 1.10f;    // chỉ trừ nhẹ khi lệch lớn
static const float E_SPD_DEAD_MM = 8.0f;
static const float K_SPD_R       = 105.0f;   // phanh chủ động theo nguy cơ thoát ly thực sự (risk_v)
static const float F_IN_MIN      = 28.0f;    // sàn bánh trong khi cua thường -> xe lướt cua cực mượt
static const float U_ABS_TAU_UP  = 0.045f;
static const float U_ABS_TAU_DN  = 0.070f;   // nhả nhanh hơn để thốc ga ngay khi vừa qua đỉnh cua

// ---------------------------- PID Phi Tuyến + Feedforward [V6] -------------
static const float KP            = 6.0f;     // PWM / mm tại tâm vạch
static const float KP_NL_MM      = 15.5f;    // Kp_eff = KP * (1 + |e|/KP_NL_MM + 0.25*(|e|/KP_NL_MM)^2)
static const float KD            = 0.35f;    // PWM*s / mm
static const float KD_NL         = 0.45f;    // tăng dập dao động khi vạch quét ngang nhanh
static const float KD_NL_VX      = 280.0f;   // mm/s
static const float KI            = 42.0f;    // PWM / (mm*s)
static const float I_MAX         = 70.0f;
static const float I_LEAK_TAU    = 0.80f;
static const float I_LEAK_DYN    = 3.5f;
static const float I_VX_REF      = 150.0f;
static const float I_LEAK_WRONG  = 7.5f;     // xả cực nhanh tích phân cũ khi đảo chiều cua S-curve
static const float I_ACTIVE_MM   = 15.0f;

// ---------------------------- Bộ lọc vị trí alpha-beta có hướng ------------
static const float AB_ALPHA      = 0.32f;
static const float AB_BETA       = 0.038f;
static const float JUMP_MM       = 26.0f;

// ---------------------------- Ước lượng vị trí -----------------------------
static const float W_FLOOR       = 0.035f;
static const float WSUM_MIN      = 0.035f;
static const float K_EXT_MM      = 9.5f;     // mở rộng tầm đo rìa ngoài tới +/- 29.5 mm
static const float X_LIM_MM      = 30.0f;

// ---------------------------- Mất vạch & Bắt lại vạch [V6] -----------------
static const float GAP_X_MM      = 10.0f;
static const float GAP_HOLD_MM   = 55.0f;
static const float V_GAP_MAX     = 245.0f;
static const float V_LOST        = 160.0f;   // giữ tốc độ quét hợp lý để bắt lại vạch nhanh
static const float U_LOST_MAX    = 135.0f;
static const float LOST_RAMP_S   = 0.09f;    // bẻ gắt nhanh hơn khi thực sự văng cua
static const float LOST_GIVEUP_S = 1.25f;    // [V6-BUGFIX] thời gian tối đa tìm lại vạch trước khi dừng an toàn
static const float LOST_PRED_T   = 0.035f;
static const float REACQ_VX      = 145.0f;
static const float F_IN_EMERG    = 0.0f;     // khóa bánh trong về 0 khi nguy cơ = 1 (không đảo chiều -> 0 sụt áp)
static const float F_IN_LOST     = -15.0f;   // khi mất vạch hẳn mới cho phép ghìm nhẹ bánh trong

// ---------------------------- [V6] Dự báo nguy cơ CÓ HƯỚNG (Directional) ---
static const float T_PRED        = 0.055f;   // tầm nhìn dự báo cho LÁI (s)
static const float RISK_X0       = 7.5f;     // mm
static const float RISK_X1       = 19.5f;    // mm
static const float T_PRED_V      = 0.080f;   // tầm nhìn dự báo cho PHANH (s)
static const float RISK_V_X0     = 9.5f;     // mm
static const float RISK_V_X1     = 22.0f;    // mm
static const float U_SLEW_CALM   = 3200.0f;  // PWM/s khi đường êm
static const float U_SLEW_FAST   = 35000.0f; // PWM/s khi nguy cơ cao (phản ứng tức thì)

// ---------------------------- Ngã tư & Chống nhiễu Gờ giảm tốc [V6] --------
static const float WING_ON               = 0.50f;
static const float WING_OFF              = 0.35f;
static const float WING_BASE_MAX         = 0.42f;
static const int   WING_DEBOUNCE_TICKS   = 3;       // 6 ms
static const float WING_MIN_CHORD_MM     = 6.5f;    // [V6] xung cánh phải dày >= 6.5mm để loại bỏ gờ giảm tốc/nảy gầm
static const float WING_LATCH_MM         = 65.0f;
static const float CROSS_PASS_MAX_MM     = 190.0f;
static const float CROSS_PASS_MAX_S      = 0.70f;
static const float CROSS_EXIT_MARGIN_MM  = 14.0f;
static const float U_REF_TAU             = 0.18f;

// ---------------------------- [V6] Rẽ tắt Vòng Cung (Rolling Arc Turn) -----
// Thay vì chạy lố 43mm rồi đứng yên quay ngược bánh, xe chỉ tiến ARC_ALIGN_MM (16mm),
// sau đó ghìm bánh trong về 0 và phóng bánh ngoài P_ARC_OUTER (+215) để ôm vòng cung tốc độ cao!
static const float V_ALIGN            = 125.0f;  // [V6] không phanh khựng về 62 nữa, giữ đà 125 để lướt vào cung rẽ!
static const float ARC_ALIGN_MM       = 16.0f;   // quãng đường căn trước khi bẻ vòng cung (tâm quay tại bánh trong)
static const float ALIGN_FALLBACK_MM  = 52.0f;
static const float ALIGN_FALLBACK_S   = 0.25f;
static const float P_ARC_OUTER        = 215.0f;  // tốc độ bánh ngoài khi quét vòng cung
static const float P_ARC_INNER_CLEAR  = 0.0f;    // bánh trong = 0 (KHÔNG ĐẢO CHIỀU -> KHÔNG SỤT ÁP PIN!)
static const float P_ARC_INNER_TIGHT  = -70.0f;  // chỉ hỗ trợ đảo nhẹ nếu sau ARC_TIGHT_T chưa bắt xong nhánh mới
static const float ARC_TIGHT_T        = 0.15f;   // s
static const float P_APPROACH_OUTER   = 140.0f;  // giảm mượt khi nhánh mới đã chạm mắt rìa
static const float P_APPROACH_INNER   = -35.0f;
static const float PIVOT_OLD_ON       = 0.50f;
static const float PIVOT_OLD_GONE     = 0.26f;
static const float PIVOT_NEW_OUTER    = 0.72f;
static const float PIVOT_RELAX_T      = 0.22f;
static const float T_PIVOT_MAX        = 1.00f;
static const float PIVOT_SWEEP_MMPS   = 280.0f;
static const float HANDOVER_T         = 0.20f;   // bàn giao ngắn và mượt
static const float V_HANDOVER         = 145.0f;  // [V6] thoát ngã tư ở tốc độ cao (vì xe vẫn đang có đà tiến)
static const float F_IN_HANDOVER      = -20.0f;
static const float U_MAX_HANDOVER     = 130.0f;

// ---------------------------- Thích ứng màu nền ----------------------------
static const float CM_DEAD            = 0.06f;
static const float CM_MAX             = 0.40f;
static const float CM_TAU_UP          = 0.08f;
static const float CM_TAU_DN          = 0.015f;
static const float CM_ABSORB_MIN      = 0.05f;
static const float CM_ABSORB_RATE     = 0.04f;

// ============================================================================
// 2. CHÂN PHẦN CỨNG, CỰC TÍNH ADC & BẢO VỆ NGUỒN BEMF
// ============================================================================
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
static const int MIN_CONTRAST_SPAN    = 400;
static const int INIT_CONTRAST_SPAN   = 1000;
static const int EFF_SPAN_MIN         = 350;
static const int WHITE_SEEN_MAX_DARK  = 1600;
static const int WHITE_DRIFT_MAX_DARK = 800;
static const int WHITE_CAL_MAX_DARK   = 1200;

const int   PIN_IR[4]          = {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN};
const int   MIN_N20_PWM        = 36;
const float RIGHT_MOTOR_TRIM   = 1.00f;

const bool INVERT_LEFT_MOTOR   = false;
const bool INVERT_RIGHT_MOTOR  = false;
const bool SWAP_MOTORS_LR      = false;

// [V6-BEMF-LIMITER] Thông số mô hình sức phản điện động & Ngân sách dòng 4 pin AA
static const float BEMF_TAU_S         = 0.042f;  // hằng số thời gian cơ học của động cơ N20 + bánh xe
static const float BEMF_SINGLE_MAX_DV = 155.0f;  // độ chênh (|PWM| - BEMF) tối đa trên 1 động cơ
static const float BEMF_TOTAL_BUDGET  = 245.0f;  // tổng (|PWM_L - BEMF_L| + |PWM_R - BEMF_R|) tối đa trên cả 2 kênh
static const int   REVERSE_PAUSE_TICKS = 2;      // 4 ms ngắt cầu H khi đảo chiều qua 0

// ============================================================================
// 3. HÀM TOÁN HỌC CƠ BẢN
// ============================================================================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }
static inline int   roundi(float v) { return (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)); }

static inline int darker(int a, int ref)   { return IR_POL * (a - ref); }
static inline int darkerOf(int a, int b)   { return (darker(a, b) >= 0) ? a : b; }
static inline int brighterOf(int a, int b) { return (darker(a, b) <= 0) ? a : b; }

struct SensorData {
    float ir[4];      // 0.0 = trắng, 1.0 = đen
    int   raw_ir[4];
    float elapsed_sec;
};

// Trộn bánh: ưu tiên tuyệt đối giữ chênh lệch lái (2*a), tự động ghim bánh ngoài ở 255 khi v_cmd cao
static void mixWheels(float v, float u, float f_in, float& vl, float& vr) {
    float a_max = 0.5f * (255.0f - f_in);
    float a = fabsf(u);
    if (a > a_max) a = a_max;
    float lo = f_in + a;
    float hi = 255.0f - a;
    float ve = clampf(v, lo, hi);
    if (u >= 0.0f) { vl = ve + a; vr = ve - a; }
    else           { vl = ve - a; vr = ve + a; }
}

// ============================================================================
// 4. BỘ ĐIỀU KHIỂN TRUNG TÂM (BRAIN v6.0)
// ============================================================================
class Brain {
public:
    enum Mode : uint8_t {
        M_RUN = 0,
        M_CROSS_PASS,      // đi thẳng qua ngã tư (Vòng tròn 1)
        M_TURN_ALIGN,      // tiến ngắn 16mm vào điểm bẻ vòng cung (Vòng tròn 2)
        M_TURN_PIVOT       // quét vòng cung tốc độ cao (Rolling Arc Turn)
    };

    Mode  mode;
    float mode_t, mode_d;

    float x, vx;
    bool  need_reinit;
    float integ, u_abs_f, u_cmd;
    float v_cmd, v_est, dist_mm, v_scale;
    bool  v_cal_valid;
    float line_w_est;
    int   last_side;

    bool  in_loss, meas_ok;
    float lost_t, lost_d, lost_x;
    int   lost_side;
    float handover_t;
    float reacq_vx;
    float risk_dbg;

    bool  was_lifted, has_seen_line;
    float off_track_t, all_black_t, air_t, run_t;

    // Theo dõi hai cánh S1 (k=0) và S4 (k=1) theo odometer (mm)
    bool  base_init;
    float base_o[2];
    bool  w_on[2], w_rose[2], w_fell[2], w_sep;
    int   w_ticks[2];
    float w_cand_d[2], w_rise_d[2], w_fall_d[2], w_half_fall_d[2], w_last_d[2];
    float cross_sec, center_odo, end_odo, align_rem_mm;
    bool  center_refined;
    float u_ref, u_hold;
    int   cross_confirm, cross_count;
    float last_cross_dist;
    int   clear_ticks, white_ticks;
    int   turn_dir;
    bool  pivot_old_seen;
    bool  pivot_has_z;
    float pivot_prev_z, pivot_vx_est;

    // Sự kiện telemetry
    bool  ev_cross; int ev_idx, ev_act; float ev_center_d;
    bool  ev_align_end; float ev_al_t, ev_al_d, ev_al_rem; uint8_t ev_al_wf;
    bool  ev_pivot_end; float ev_pv_t, ev_pv_z;

    int   prev_pl, prev_pr;

    Brain() { reset(); }

    void reset() {
        mode = M_RUN; mode_t = 0.0f; mode_d = 0.0f;
        x = 0.0f; vx = 0.0f; need_reinit = true;
        integ = 0.0f; u_abs_f = 0.0f; u_cmd = 0.0f;
        v_cmd = V_START; v_est = 0.0f; dist_mm = 0.0f; v_scale = 1.0f; v_cal_valid = false;
        line_w_est = DEFAULT_LINE_W_MM;
        last_side = 1;
        in_loss = false; meas_ok = false;
        lost_t = 0.0f; lost_d = 0.0f; lost_x = 0.0f; lost_side = 1;
        handover_t = 0.0f; reacq_vx = 0.0f; risk_dbg = 0.0f;
        was_lifted = false; has_seen_line = false;
        off_track_t = 0.0f; all_black_t = 0.0f; air_t = 0.0f; run_t = 0.0f;
        base_init = false; base_o[0] = base_o[1] = 0.0f;
        cross_confirm = 0; cross_count = 0; last_cross_dist = -1.0e6f;
        clear_ticks = 0; white_ticks = 0; turn_dir = 1; pivot_old_seen = false;
        pivot_has_z = false; pivot_prev_z = 0.0f; pivot_vx_est = 0.0f;
        cross_sec = 1.0f; center_odo = 0.0f; end_odo = 0.0f; align_rem_mm = 0.0f; center_refined = false;
        u_ref = 0.0f; u_hold = 0.0f;
        ev_cross = false; ev_idx = 0; ev_act = 0; ev_center_d = 0.0f;
        ev_align_end = false; ev_al_t = 0.0f; ev_al_d = 0.0f; ev_al_rem = 0.0f; ev_al_wf = 0;
        ev_pivot_end = false; ev_pv_t = 0.0f; ev_pv_z = 0.0f;
        clearWings();
        prev_pl = 0; prev_pr = 0;
    }

    static bool hasRealLine(const float* s) {
        float s_max = max4f(s[0], s[1], s[2], s[3]);
        float s_min = min4f(s[0], s[1], s[2], s[3]);
        if (s_min > 0.85f) return false;
        float in_max  = max2f(s[1], s[2]);
        float out_max = max2f(s[0], s[3]);
        float out_min = min2f(s[0], s[3]);
        float min_contrast = (s_min > 0.22f) ? 0.38f : 0.26f;
        bool peak_in_center = (in_max > out_max + 0.05f);
        if (s_max >= 0.35f && ((s_max - s_min) >= min_contrast || (s_max >= 0.80f && peak_in_center))) return true;
        return (in_max >= 0.055f && out_min < 0.05f && (in_max - out_max) >= 0.04f);
    }

    void step(const SensorData& sd, float dt, int hw_pl, int hw_pr, int& pl, int& pr) {
        pl = 0; pr = 0;
        if (!(dt > 0.0005f && dt < 0.02f)) dt = 0.002f;
        const float* s = sd.ir;

        // ---- Phát hiện nhấc xe / toàn đen / lệch khỏi đường đua [V6-BUGFIX] ----
        int max_raw = sd.raw_ir[0], min_raw = sd.raw_ir[0];
        for (int i = 1; i < 4; i++) {
            if (sd.raw_ir[i] > max_raw) max_raw = sd.raw_ir[i];
            if (sd.raw_ir[i] < min_raw) min_raw = sd.raw_ir[i];
        }
        bool in_air = (IR_POL < 0) ? (max_raw <= ADC_RAW_LO_OK + 60) : (min_raw >= ADC_FRAME_HI_OK);
        if (in_air) air_t += dt;
        else        air_t = 0.0f;

        float min_ir = min4f(s[0], s[1], s[2], s[3]);
        bool real_line = hasRealLine(s);
        bool all_black = (min_ir > 0.88f);
        bool no_line   = !all_black && !real_line;
        bool spinning  = (mode == M_TURN_PIVOT);
        bool in_cross  = (mode == M_CROSS_PASS || mode == M_TURN_ALIGN);

        if (air_t > 0.10f) {
            was_lifted = true;
        } else if (was_lifted) {
            if (real_line && min_ir < 0.65f && !in_air) {
                reset();
                has_seen_line = true;
            }
        } else if (!has_seen_line) {
            if (real_line && min_ir < 0.75f && !in_air) {
                has_seen_line = true;
                off_track_t = 0.0f;
                all_black_t = 0.0f;
            } else {
                off_track_t += dt;
            }
        } else {
            if (all_black && !spinning && !in_cross) {
                all_black_t += dt;
                if (all_black_t > 0.28f) was_lifted = true;
            } else {
                all_black_t = 0.0f;
            }
            // [V6-BUGFIX] Cho phép xe tìm lại vạch đủ LOST_GIVEUP_S (1.25s) thay vì bị cắt sớm ở 0.65s!
            if (no_line && !spinning && !in_cross) {
                off_track_t += dt;
                if (off_track_t > LOST_GIVEUP_S) was_lifted = true;
            } else {
                off_track_t = 0.0f;
            }
        }
        if (was_lifted) { stopOutputs(); return; }

        run_t += dt;
        updateSpeedEstimate(dt, hw_pl, hw_pr);

        if (!has_seen_line) {
            stopOutputs();
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

    void rampSpeed(float target, float dt, float dn = ACC_DN) {
        if (target > v_cmd) { v_cmd += ACC_UP * dt; if (v_cmd > target) v_cmd = target; }
        else                { v_cmd -= dn * dt;     if (v_cmd < target) v_cmd = target; }
    }

    // [V6] Cập nhật tốc độ từ PWM thực tế đang cấp ra cầu H (hw_pl, hw_pr)
    void updateSpeedEstimate(float dt, int hw_pl, int hw_pr) {
        float c_hw = 0.5f * (float)(hw_pl + hw_pr);
        if (c_hw < 0.0f) c_hw = 0.0f;
        float tgt = v_scale * SPEED_MMPS_AT_255 * (c_hw / 255.0f);
        float tau = (tgt < v_est) ? 0.095f : 0.042f;
        v_est += (tgt - v_est) * (dt / (tau + dt));
        dist_mm += v_est * dt;
    }

    // ---- [V6-VORONOI] Ước lượng vị trí vạch (mm) đã khử méo bậc thang hình thang ----
    bool measure(const float* s, float& z) const {
        if (!hasRealLine(s) && min4f(s[0], s[1], s[2], s[3]) <= 0.85f) return false;
        float w_raw[4], w_vor[4], wsum = 0.0f;
        for (int i = 0; i < 4; i++) {
            float c = (s[i] - W_FLOOR) * (1.0f / (1.0f - W_FLOOR));
            w_raw[i] = (c > 0.0f) ? c : 0.0f;
            // Nhân trọng số bề rộng ô Voronoi để tích phân trọng tâm chính xác trên lưới không đều
            w_vor[i] = w_raw[i] * SENSOR_VORONOI_W[i];
            wsum += w_vor[i];
        }
        if (wsum < WSUM_MIN) return false;
        float num = 0.0f;
        for (int i = 0; i < 4; i++) num += w_vor[i] * SENSOR_X_MM[i];
        float xc = num / wsum;

        // Ngoại suy mượt khi vạch chỉ còn bám ở mắt ngoài cùng S1 hoặc S4
        if (w_raw[3] >= w_raw[0]) {
            float inner = w_raw[0] + w_raw[1] + w_raw[2];
            float b = clampf(1.0f - inner / 0.32f, 0.0f, 1.0f);
            xc += b * K_EXT_MM * (1.0f - min2f(w_raw[3], 1.0f));
        } else {
            float inner = w_raw[1] + w_raw[2] + w_raw[3];
            float b = clampf(1.0f - inner / 0.32f, 0.0f, 1.0f);
            xc -= b * K_EXT_MM * (1.0f - min2f(w_raw[0], 1.0f));
        }
        z = clampf(xc, -X_LIM_MM, X_LIM_MM);
        return true;
    }

    // ---- Theo dõi hai cánh S1, S4 & Chống nhiễu Gờ giảm tốc [V6] --------------
    void updateWingBase(const float* s, float dt) {
        if (!base_init) { base_o[0] = s[0]; base_o[1] = s[3]; base_init = true; }
        const float ab = dt / (0.15f + dt);
        base_o[0] += ((s[0] < 0.45f) ? ab : ab * 0.125f) * (s[0] - base_o[0]);
        base_o[1] += ((s[3] < 0.45f) ? ab : ab * 0.125f) * (s[3] - base_o[1]);
    }

    void clearWings() {
        for (int k = 0; k < 2; k++) {
            w_on[k] = false; w_rose[k] = false; w_fell[k] = false; w_ticks[k] = 0;
            w_cand_d[k] = w_rise_d[k] = w_fall_d[k] = w_half_fall_d[k] = w_last_d[k] = dist_mm;
        }
        w_sep = false;
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
                    float chord = w_half_fall_d[k] - w_rise_d[k];
                    // [V6-SPEEDBUMP-GATE] Nếu xung cánh quá mỏng (< 6.5mm) và cánh kia chưa lên -> nhiễu gờ giảm tốc!
                    if (mode == M_RUN && chord < WING_MIN_CHORD_MM && !w_rose[1 - k]) {
                        w_rose[k] = false; w_fell[k] = false; w_ticks[k] = 0;
                    } else {
                        w_fell[k] = true; w_fall_d[k] = dist_mm;
                    }
                }
            } else if (!w_rose[k]) {
                if (v > WING_ON && (base_o[k] < WING_BASE_MAX || (v - base_o[k]) > 0.25f)) {
                    if (w_ticks[k] == 0) w_cand_d[k] = dist_mm;
                    if (++w_ticks[k] >= WING_DEBOUNCE_TICKS) {
                        w_on[k] = true; w_rose[k] = true; w_fell[k] = false;
                        w_rise_d[k] = w_cand_d[k]; w_half_fall_d[k] = dist_mm; w_last_d[k] = dist_mm;
                    }
                } else {
                    w_ticks[k] = 0;
                }
            } else if (mode == M_RUN && (((dist_mm - w_last_d[k]) > WING_LATCH_MM) ||
                       (w_fell[k] && !w_rose[1 - k] && !w_sep && (dist_mm - w_fall_d[k]) > 18.0f))) {
                w_rose[k] = false; w_fell[k] = false; w_ticks[k] = 0;
            }
        }
        if (w_rose[0] || w_rose[1] || w_ticks[0] > 0 || w_ticks[1] > 0) {
            bool dip_sep    = (s[0] > 0.48f && s[1] < s[0] - 0.05f && s[2] > 0.16f) ||
                              (s[3] > 0.48f && s[2] < s[3] - 0.05f && s[1] > 0.16f);
            bool both_outer = (s[0] > 0.52f && s[3] > 0.52f && (s[1] > 0.20f || s[2] > 0.20f));
            bool edge_skew  = (s[0] > 0.80f && s[0] >= s[1] - 0.22f && s[2] > 0.35f) ||
                              (s[3] > 0.80f && s[3] >= s[2] - 0.22f && s[1] > 0.35f);
            if (dip_sep || both_outer || edge_skew) w_sep = true;
        } else {
            w_sep = false;
        }
        updateWingBase(s, dt);
    }

    // Khoảng cách tối thiểu giữa 2 ngã tư liên tiếp theo địa hình thực tế
    float getMinCrossSpacing() const {
        int idx = (cross_count < 3) ? cross_count : 3;
        float scale_guard = v_cal_valid ? clampf(v_scale, 0.70f, 1.30f) : 0.85f;
        return CROSS_MIN_DIST_TABLE[idx] * scale_guard;
    }

    bool detectCross(const float* s, bool allowed) {
        float skew_max = v_cal_valid ? 55.0f : 70.0f;
        bool skew_ok = fabsf(w_rise_d[1] - w_rise_d[0]) <= skew_max;
        bool center  = (s[1] > 0.18f || s[2] > 0.18f || (s[0] > 0.50f && s[3] > 0.50f));
        // Cánh vào trước phải đi được ít nhất WING_MIN_CHORD_MM hoặc cả 2 cánh đang đậm cùng lúc
        float lead_chord = dist_mm - min2f(w_rise_d[0], w_rise_d[1]);
        bool chord_ok = (lead_chord >= WING_MIN_CHORD_MM) || (s[0] > 0.62f && s[3] > 0.62f);
        bool trig = allowed && w_rose[0] && w_rose[1] && w_sep && center && skew_ok && chord_ok;
        cross_confirm = trig ? (cross_confirm + 1) : 0;
        return cross_confirm >= 1;
    }

    void planCross() {
        float span = SENSOR_X_MM[3] - SENSOR_X_MM[0];
        float t = fabsf(w_rise_d[1] - w_rise_d[0]) / span;
        t = min2f(t, 3.0f);
        cross_sec  = sqrtf(1.0f + t * t);
        float thick = line_w_est * cross_sec;
        center_odo = 0.5f * (w_rise_d[0] + w_rise_d[1]) + 0.5f * thick;
        end_odo    = max2f(w_rise_d[0], w_rise_d[1]) + thick + CROSS_EXIT_MARGIN_MM;
        center_refined = false;
    }

    void refineCenter() {
        if (center_refined) return;
        float L_meas = -1.0f;
        float fall_mid = dist_mm;
        float L_min_ok = 0.45f * line_w_est;
        float L_max_ok = 3.20f * line_w_est * cross_sec;

        if (w_fell[0] && w_fell[1]) {
            center_refined = true;
            float L0 = w_half_fall_d[0] - w_rise_d[0], L1 = w_half_fall_d[1] - w_rise_d[1];
            if (L0 > L_min_ok && L1 > L_min_ok && L0 < L_max_ok && L1 < L_max_ok &&
                fabsf(L0 - L1) < 0.45f * max2f(L0, L1)) {
                L_meas = (fabsf(L0 - L1) > 0.20f * max2f(L0, L1)) ? min2f(L0, L1) : (0.5f * (L0 + L1));
                fall_mid = 0.5f * (w_half_fall_d[0] + w_half_fall_d[1]);
                center_odo = 0.25f * (w_rise_d[0] + w_half_fall_d[0] + w_rise_d[1] + w_half_fall_d[1]);
            }
        }
        if (L_meas <= 0.0f) return;

        float d_skew = fabsf(w_rise_d[1] - w_rise_d[0]);
        float span   = SENSOR_X_MM[3] - SENSOR_X_MM[0];
        float skew_term = (line_w_est * d_skew) / span;
        float denom_sq  = max2f(L_meas * L_meas - skew_term * skew_term, 0.25f * L_meas * L_meas);
        float r = clampf(line_w_est / sqrtf(denom_sq), 0.50f, 1.80f);

        float v_scale_old = v_scale;
        float v_scale_obs = clampf(v_scale_old * r, 0.50f, 1.80f);
        float blend = !v_cal_valid ? 0.85f : 0.40f;
        v_scale = clampf(v_scale_old + blend * (v_scale_obs - v_scale_old), 0.50f, 1.80f);
        v_cal_valid = true;
        float r_eff = v_scale / v_scale_old;
        v_est *= r_eff;
        (void)fall_mid;
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
        // [V6-ARC-TURN] Với Rolling Arc Turn, tâm quay nằm ở bánh trong nên chỉ cần tiến thêm ARC_ALIGN_MM (16mm)
        align_rem_mm = ARC_ALIGN_MM;
        ev_cross = true; ev_idx = idx + 1; ev_act = act; ev_center_d = center_odo;
        if (act == 0) {
            mode = M_CROSS_PASS;
        } else {
            turn_dir = (act > 0) ? 1 : -1;
            mode = M_TURN_ALIGN;
        }
    }

    // ---- RUN: Cập nhật bộ lọc alpha-beta, phát hiện mất vạch & ngã tư ----
    void runEstimate(const float* s, float dt) {
        bool allowed = (handover_t <= 0.05f) && !in_loss && (run_t > 0.12f) &&
                       ((dist_mm - last_cross_dist) >= getMinCrossSpacing());
        if (allowed) trackWings(s, dt);
        else { clearWings(); updateWingBase(s, dt); }
        bool crossing = detectCross(s, allowed);

        float z = 0.0f;
        meas_ok = measure(s, z);
        if (meas_ok) {
            if (in_loss) {
                in_loss = false;
                if (lost_t > 0.012f) {
                    need_reinit = true;
                    reacq_vx = (fabsf(lost_x) >= GAP_X_MM) ? -(float)lost_side * REACQ_VX : 0.0f;
                }
                if (lost_t > 0.12f) integ *= 0.25f;
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
                    vx = clampf(vx, -1600.0f, 1600.0f);
                }
            }
            if (need_reinit) { x = z; vx = reacq_vx; reacq_vx = 0.0f; need_reinit = false; }
            if (fabsf(x) > 2.5f) last_side = (x > 0.0f) ? 1 : -1;
        } else {
            if (!in_loss) {
                in_loss = true; lost_t = 0.0f; lost_d = 0.0f;
                float xs = clampf(x + LOST_PRED_T * vx, -X_LIM_MM, X_LIM_MM);
                lost_x = xs;
                lost_side = (fabsf(xs) > 1.5f) ? ((xs > 0.0f) ? 1 : -1) : last_side;
            }
            lost_t += dt;
            lost_d += v_est * dt;
            x = clampf(x + vx * dt, -X_LIM_MM, X_LIM_MM);
            vx *= 0.96f;
            if (lost_t > LOST_GIVEUP_S) was_lifted = true;
        }
        if (crossing) enterCross();
    }

    // ---- [V6] Điều khiển bám vạch: Directional Risk + Outer-Wheel-Pinned ----
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
        float risk = 0.0f, risk_v = 0.0f;
        float dn_rate = ACC_DN;

        if (!in_loss) {
            float ae = fabsf(x);
            float s_dir = (x >= 0.0f) ? 1.0f : -1.0f;
            // [V6-DIRECTIONAL-RISK] Vận tốc thoát ly khỏi tâm:
            //   v_out > 0: vạch đang trôi RA XA tâm (nguy hiểm -> cần tăng lái & phanh sớm)
            //   v_out < 0: vạch đang quay VỀ TÂM (an toàn -> nhả phanh ngay tại đỉnh cua để bứt tốc!)
            float v_out = (ae > 1.5f) ? (s_dir * vx) : fabsf(vx);

            float x_esc_steer = ae + T_PRED * v_out;
            risk = clampf((x_esc_steer - RISK_X0) / (RISK_X1 - RISK_X0), 0.0f, 1.0f);

            float x_esc_brake = ae + T_PRED_V * v_out;
            risk_v = clampf((x_esc_brake - RISK_V_X0) / (RISK_V_X1 - RISK_V_X0), 0.0f, 1.0f);

            if (!handover) {
                f_in  = F_IN_MIN + (F_IN_EMERG - F_IN_MIN) * risk;
                a_cap = 0.5f * (255.0f - f_in);
            }

            // Kp phi tuyến bậc 2 nhẹ ở rìa: cực êm ở tâm, cực gắt khi chớm ra mép S1/S4
            float norm_e = ae / KP_NL_MM;
            float kp = KP * (1.0f + norm_e + 0.25f * norm_e * norm_e);
            float kd = KD * (1.0f + KD_NL * min2f(fabsf(vx) / KD_NL_VX, 1.0f));
            float u_raw = kp * x + integ + kd * vx;
            bool  sat = fabsf(u_raw) > a_cap;

            float dyn = clampf(fabsf(vx) / I_VX_REF, 0.0f, 1.0f);
            if (ae < I_ACTIVE_MM && !(sat && ((u_raw > 0.0f) == (x > 0.0f)))) {
                integ += KI * x * dt * (1.0f - dyn);
            }
            float leak = 1.0f / I_LEAK_TAU + I_LEAK_DYN * dyn;
            if (ae > 2.5f && integ * x < 0.0f) leak *= I_LEAK_WRONG;
            integ -= integ * (leak * dt);
            integ = clampf(integ, -I_MAX, I_MAX);

            u = clampf(u_raw, -a_cap, a_cap);
            if (!handover) {
                float lim = (U_SLEW_CALM + (U_SLEW_FAST - U_SLEW_CALM) * max2f(risk, risk_v)) * dt;
                u = clampf(u, u_cmd - lim, u_cmd + lim);
                u = clampf(u, -a_cap, a_cap);
            }

            float uf_tau = (fabsf(u) > u_abs_f) ? U_ABS_TAU_UP : U_ABS_TAU_DN;
            u_abs_f += (fabsf(u) - u_abs_f) * (dt / (uf_tau + dt));

            // [V6-OUTER-PINNED] Không phạt tốc độ kép: giữ v_t cao khi risk_v == 0 để bánh ngoài luôn đạt ~255
            v_t = V_MAX - K_SPD_U * u_abs_f - K_SPD_E * max2f(0.0f, ae - E_SPD_DEAD_MM) - K_SPD_R * risk_v;
            float v_floor = V_CORNER_MIN - (V_CORNER_MIN - V_BRAKE_MIN) * risk_v;
            v_t = clampf(v_t, v_floor, V_MAX);
            dn_rate = ACC_DN_SOFT + (ACC_DN - ACC_DN_SOFT) * risk_v;
        } else {
            float lx = fabsf(lost_x);
            if (lx < GAP_X_MM && lost_d < GAP_HOLD_MM) {
                u = clampf(integ, -a_cap, a_cap);
                v_t = min2f(v_cmd, V_GAP_MAX);
            } else {
                if (!handover) { f_in = F_IN_LOST; a_cap = 0.5f * (255.0f - f_in); }
                float ramp = clampf(lost_t / LOST_RAMP_S, 0.0f, 1.0f);
                float u0   = min2f(KP * (1.0f + lx / KP_NL_MM) * max2f(lx, 20.0f), U_LOST_MAX);
                float mag  = u0 + (U_LOST_MAX - u0) * ramp;
                u = (float)lost_side * min2f(mag, a_cap);
                v_t = V_LOST;
            }
            risk = 1.0f; risk_v = 1.0f;
        }
        if (handover) {
            float v_ho = (fabsf(x) > 8.0f || fabsf(vx) > 130.0f) ? V_HANDOVER : 185.0f;
            if (v_t > v_ho) v_t = v_ho;
        }
        rampSpeed(v_t, dt, dn_rate);
        mixWheels(v_cmd, u, f_in, vl, vr);
        u_cmd = u;
        risk_dbg = max2f(risk, risk_v);

        if (!handover && !in_loss && !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0)
            u_ref += (u - u_ref) * (dt / (U_REF_TAU + dt));
    }

    // ---- Đi thẳng qua ngã tư (Vòng tròn 1) ----
    void outCrossPass(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        mode_d += v_est * dt;
        trackWings(s, dt);
        refineCenter();
        u_hold -= u_hold * (dt / 0.10f);
        float clr_lim0 = clampf(max2f(0.30f, base_o[0] + 0.12f), 0.30f, 0.55f);
        float clr_lim1 = clampf(max2f(0.30f, base_o[1] + 0.12f), 0.30f, 0.55f);
        bool clear = w_fell[0] && w_fell[1] && (s[0] < clr_lim0 && s[3] < clr_lim1);
        clear_ticks = clear ? (clear_ticks + 1) : 0;
        rampSpeed(V_MAX, dt);   // giữ tốc độ tối đa băng qua ngã tư đi thẳng
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;
        bool wings_done = (w_fell[0] && w_fell[1]);
        if ((clear_ticks >= 3 && mode_d > 15.0f) || ((wings_done || mode_d > 75.0f) && dist_mm >= end_odo) ||
            mode_d > CROSS_PASS_MAX_MM || mode_t > CROSS_PASS_MAX_S) {
            mode = M_RUN;
            need_reinit = true;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f;
            last_cross_dist = dist_mm;
            clearWings();
        }
    }

    // ---- [V6-ARC-TURN] Pha 1: Tiến ngắn 16mm để đặt bánh trong vào tâm cung rẽ ----
    void outAlign(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        mode_d += v_est * dt;
        align_rem_mm -= v_est * dt;
        trackWings(s, dt);
        rampSpeed(V_ALIGN, dt);
        u_hold -= u_hold * (dt / 0.08f);
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;
        ev_center_d = center_odo;
        if (align_rem_mm <= 0.0f || mode_d >= ARC_ALIGN_MM || mode_d > ALIGN_FALLBACK_MM || mode_t > ALIGN_FALLBACK_S) {
            ev_align_end = true; ev_al_t = mode_t; ev_al_d = mode_d; ev_al_rem = align_rem_mm;
            ev_al_wf = (w_fell[0] ? 1 : 0) | (w_fell[1] ? 2 : 0);
            enterPivot();
        }
    }

    void enterPivot() {
        mode = M_TURN_PIVOT; mode_t = 0.0f; mode_d = 0.0f; white_ticks = 0; pivot_old_seen = false;
        pivot_has_z = false; pivot_prev_z = 0.0f; pivot_vx_est = 0.0f;
        clearWings();
    }

    // ---- [V6-ARC-TURN] Pha 2: Quét Vòng Cung Tốc Độ Cao (Rolling Arc Turn) ----
    // Bánh ngoài chạy nhanh (+215), bánh trong ghìm về 0 (không đảo chiều động cơ -> 0% sụt áp pin).
    // Nếu sau ARC_TIGHT_T (0.15s) góc nhánh quá gắt chưa vào hết thì mới hỗ trợ đảo nhẹ bánh trong (-70).
    void outPivot(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        float old_out = (turn_dir > 0) ? s[0] : s[3];
        float inner_on = (line_w_est < 22.0f) ? 0.25f : 0.38f;
        bool  new_in  = (turn_dir > 0) ? (s[1] > inner_on || s[2] > inner_on || s[3] > PIVOT_NEW_OUTER)
                                       : (s[2] > inner_on || s[1] > inner_on || s[0] > PIVOT_NEW_OUTER);
        if (old_out > PIVOT_OLD_ON) { if (++white_ticks >= 2) pivot_old_seen = true; }
        else white_ticks = 0;
        bool old_gone = (pivot_old_seen || mode_t > PIVOT_RELAX_T) && old_out < PIVOT_OLD_GONE;

        float zp = 0.0f;
        if (old_gone && measure(s, zp)) {
            if (pivot_has_z) {
                float v_inst = (zp - pivot_prev_z) / dt;
                if (v_inst * (float)(-turn_dir) > 30.0f && fabsf(v_inst) < 1200.0f) {
                    pivot_vx_est = (pivot_vx_est == 0.0f) ? v_inst : (0.5f * pivot_vx_est + 0.5f * v_inst);
                }
            }
            pivot_prev_z = zp;
            pivot_has_z = true;
        } else {
            pivot_has_z = false;
        }

        float new_out = (turn_dir > 0) ? s[3] : s[0];
        float p_outer, p_inner;
        if (!old_gone) {
            p_outer = P_ARC_OUTER;
            p_inner = (mode_t < ARC_TIGHT_T) ? P_ARC_INNER_CLEAR : P_ARC_INNER_TIGHT;
        } else if (new_out > 0.22f) {
            p_outer = P_APPROACH_OUTER;
            p_inner = P_APPROACH_INNER;
        } else {
            p_outer = 175.0f;
            p_inner = (mode_t < ARC_TIGHT_T) ? 0.0f : P_ARC_INNER_TIGHT;
        }

        if (turn_dir > 0) { vl = p_outer; vr = p_inner; }
        else              { vl = p_inner; vr = p_outer; }
        v_cmd = 0.5f * (vl + vr);

        if (old_gone && new_in && mode_t > 0.045f) {
            float z = 0.0f;
            if (!measure(s, z)) z = (float)turn_dir * 22.0f;
            x = z;
            vx = (fabsf(pivot_vx_est) > 30.0f) ? pivot_vx_est : (-(float)turn_dir * PIVOT_SWEEP_MMPS * v_scale);
            ev_pivot_end = true; ev_pv_t = mode_t; ev_pv_z = x;
            need_reinit = false;
            integ = 0.0f; u_abs_f = 0.0f; u_cmd = 0.0f;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f; reacq_vx = 0.0f;
            handover_t = HANDOVER_T; v_cmd = V_HANDOVER;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings();
            mode = M_RUN; mode_t = 0.0f;
            return;
        }
        if (mode_t > T_PIVOT_MAX) {
            ev_pivot_end = true; ev_pv_t = mode_t; ev_pv_z = 99.0f;
            x = 0.0f; vx = 0.0f; need_reinit = true;
            in_loss = false; lost_t = 0.0f; lost_d = 0.0f; reacq_vx = 0.0f;
            integ = 0.0f; handover_t = 0.0f; v_cmd = V_LOST;
            last_side = turn_dir; last_cross_dist = dist_mm;
            clearWings();
            mode = M_RUN; mode_t = 0.0f;
        }
    }
};

// ============================================================================
// 5. PHẦN CỨNG: ĐỘNG CƠ, BẢO VỆ SỤT ÁP BEMF & ĐỌC CẢM BIẾN
// ============================================================================
Brain brain;

int   ir_white_cal[4]  = {ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM};
int   ir_black_cal[4]  = {ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM};
int   adc_white_anchor = ADC_WHITE_NOM;
int   track_span_est   = 1500;
float floor_cm         = 0.0f;

unsigned long startMicros         = 0;
unsigned long lastStepMicros      = 0;
unsigned long lastTelemetryMillis = 0;
uint32_t      loopFrameCount      = 0;

int   prev_hw_pwm_l = 0, prev_hw_pwm_r = 0;
int   zero_cross_hold_l = 0, zero_cross_hold_r = 0;
float bemf_l = 0.0f, bemf_r = 0.0f;   // [V6-BEMF-LIMITER] Sức phản điện động ước lượng (-255..+255)

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

// [V6-BEMF-LIMITER] Giới hạn dòng đỉnh thông minh dựa trên Back-EMF thực tế của từng động cơ
static inline int limitSingleChannelBEMF(int target, int prev, float bemf, int& hold) {
    if (hold > 0) { hold--; return 0; }
    // Bảo vệ đảo chiều qua 0: nếu rô-to đang quay nhanh ngược chiều target, cắt PWM về 0 xả BEMF trước
    if (prev != 0 && target != 0 && ((target > 0) != (prev > 0))) {
        hold = REVERSE_PAUSE_TICKS;
        return 0;
    }
    if (target != 0 && ((target > 0 && bemf < -45.0f) || (target < 0 && bemf > 45.0f))) {
        return 0;
    }
    // Giới hạn độ chênh điện áp rơi trên nội trở cuộn dây |target - bemf| <= BEMF_SINGLE_MAX_DV
    if (target > 0 && (float)target > bemf + BEMF_SINGLE_MAX_DV) {
        target = roundi(bemf + BEMF_SINGLE_MAX_DV);
    } else if (target < 0 && (float)target < bemf - BEMF_SINGLE_MAX_DV) {
        target = roundi(bemf - BEMF_SINGLE_MAX_DV);
    }
    return constrain(target, -255, 255);
}

void setMotorHardware(int pwm_l, int pwm_r, float dt) {
    if (SWAP_MOTORS_LR) { int tmp = pwm_l; pwm_l = pwm_r; pwm_r = tmp; }
    if (INVERT_LEFT_MOTOR)  pwm_l = -pwm_l;
    if (INVERT_RIGHT_MOTOR) pwm_r = -pwm_r;

    int target_l = applyN20Deadzone(pwm_l);
    int target_r = applyN20Deadzone((int)(pwm_r * RIGHT_MOTOR_TRIM));

    int out_l = limitSingleChannelBEMF(target_l, prev_hw_pwm_l, bemf_l, zero_cross_hold_l);
    int out_r = limitSingleChannelBEMF(target_r, prev_hw_pwm_r, bemf_r, zero_cross_hold_r);

    // [V6-BEMF-LIMITER] Kiểm soát tổng ngân sách dòng của cả 2 động cơ cộng lại (tránh sụt áp khi đề-pa 2 bánh)
    float dv_l = max2f(0.0f, (out_l >= 0) ? ((float)out_l - bemf_l) : ((float)(-out_l) + bemf_l));
    float dv_r = max2f(0.0f, (out_r >= 0) ? ((float)out_r - bemf_r) : ((float)(-out_r) + bemf_r));
    if (dv_l + dv_r > BEMF_TOTAL_BUDGET) {
        float excess = (dv_l + dv_r) - BEMF_TOTAL_BUDGET;
        int trim = roundi(0.5f * excess);
        if (out_l > 0) out_l = max(0, out_l - trim);
        else if (out_l < 0) out_l = min(0, out_l + trim);
        if (out_r > 0) out_r = max(0, out_r - trim);
        else if (out_r < 0) out_r = min(0, out_r + trim);
    }

    prev_hw_pwm_l = out_l;
    prev_hw_pwm_r = out_r;

    // Cập nhật mô hình Back-EMF cho chu kỳ tiếp theo
    float alpha_m = dt / (BEMF_TAU_S + dt);
    bemf_l += ((float)out_l - bemf_l) * alpha_m;
    bemf_r += ((float)out_r - bemf_r) * alpha_m;

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
// 6. BLACKBOX (TÙY CHỌN)
// ============================================================================
#if ENABLE_BLACKBOX
#define BB_N 7000
struct BBRec { uint16_t t10; int16_t x10; int16_t u; int16_t pl; int16_t pr; uint8_t s[4]; uint8_t mode; uint8_t pad; uint8_t cm; uint8_t rk; };
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
    r.cm = (uint8_t)clampf(floor_cm * 255.0f, 0.0f, 255.0f);
    r.rk = (uint8_t)clampf(brain.risk_dbg * 255.0f, 0.0f, 255.0f);
}
static void bbDump() {
    if (bb_n < 50) { bb_n = 0; return; }
    Serial.println("t,x_mm,u,pl,pr,s0,s1,s2,s3,mode,cross_count,floor_cm,risk");
    for (uint16_t i = 0; i < bb_n; i++) {
        const BBRec& r = bb_buf[i];
        Serial.printf("%.2f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n", r.t10 * 0.01f, r.x10 * 0.1f, (int)r.u,
                      (int)r.pl, (int)r.pr, (int)r.s[0], (int)r.s[1], (int)r.s[2], (int)r.s[3], (int)r.mode, (int)r.pad,
                      r.cm / 255.0f, r.rk / 255.0f);
    }
    bb_n = 0;
}
#endif

// ============================================================================
// 7. KHỞI TẠO & HIỆU CHUẨN TỰ ĐỘNG
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
    setMotorHardware(0, 0, 0.002f);

    // Bước 1: Đợi xe được đặt xuống mặt sân
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]);
        int r3 = readIRFiltered(PIN_IR[3]);
        bool r0_ok = (IR_POL < 0) ? (r0 > ADC_RAW_LO_OK + 350) : (r0 < ADC_RAW_HI_OK - 350);
        bool r3_ok = (IR_POL < 0) ? (r3 > ADC_RAW_LO_OK + 350) : (r3 < ADC_RAW_HI_OK - 350);
        if (r0_ok || r3_ok) break;
        delay(8);
    }

    // Bước 2: Chờ tay rời khỏi xe ổn định
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

    // Bước 3: Lấy mẫu nhanh tại vị trí xuất phát
    int bright_seen[4], dark_seen[4];
    for (int i = 0; i < 4; i++) {
        int v0 = readIRFiltered(PIN_IR[i]);
        bright_seen[i] = v0;
        dark_seen[i]   = v0;
    }
    for (int sample = 0; sample < 20; sample++) {
        for (int i = 0; i < 4; i++) {
            int v = readIRFiltered(PIN_IR[i]);
            bright_seen[i] = brighterOf(bright_seen[i], v);
            dark_seen[i]   = darkerOf(dark_seen[i], v);
        }
        delay(3);
    }

    int g_white = bright_seen[0];
    int g_dark  = dark_seen[0];
    for (int i = 1; i < 4; i++) {
        g_white = brighterOf(g_white, bright_seen[i]);
        g_dark  = darkerOf(g_dark, dark_seen[i]);
    }

    bool white_saturated = (IR_POL < 0) ? (g_white <= ADC_RAW_LO_OK + 200) : (g_white >= ADC_RAW_HI_OK - 200);
    if (white_saturated) {
        g_white = ADC_WHITE_NOM;
        g_dark  = ADC_BLACK_NOM;
        for (int i = 0; i < 4; i++) dark_seen[i] = ADC_BLACK_NOM;
    }
    adc_white_anchor = g_white;

    int init_span = darker(g_dark, g_white);
    bool startup_has_line = (init_span >= INIT_CONTRAST_SPAN && g_dark > ADC_RAW_LO_OK && g_dark < ADC_RAW_HI_OK);
    if (!startup_has_line) {
        g_dark = constrain(g_white + IR_POL * 1600, ADC_RAW_LO_OK + 50, ADC_RAW_HI_OK - 50);
        init_span = abs(g_dark - g_white);
    }
    track_span_est = constrain(init_span, INIT_CONTRAST_SPAN, 3000);

    int white_tol = startup_has_line ? ((init_span / 3 < 850) ? (init_span / 3) : 850) : 950;
    for (int i = 0; i < 4; i++) {
        if (darker(bright_seen[i], g_white) < white_tol) ir_white_cal[i] = bright_seen[i];
        else                                             ir_white_cal[i] = g_white;

        int dk = dark_seen[i];
        if (startup_has_line && darker(dk, ir_white_cal[i]) >= INIT_CONTRAST_SPAN && dk > ADC_RAW_LO_OK && dk < ADC_RAW_HI_OK)
            ir_black_cal[i] = dk;
        else
            ir_black_cal[i] = darkerOf(g_dark, ir_white_cal[i] + IR_POL * track_span_est);
    }

    brain.reset();
    floor_cm = 0.0f;
    bemf_l = 0.0f; bemf_r = 0.0f;
    startMicros    = micros();
    lastStepMicros = startMicros;
}

// ============================================================================
// 8. VÒNG LẶP THỜI GIAN THỰC 500 Hz
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

    int  frame_span   = frame_max_raw - frame_min_raw;
    int  frame_bright = (IR_POL > 0) ? frame_min_raw : frame_max_raw;
    bool raw_in_range = (IR_POL < 0) ? (frame_min_raw > ADC_RAW_LO_OK) : (frame_max_raw < ADC_FRAME_HI_OK);
    bool slow_tick    = ((loopFrameCount & 0x1F) == 0);

    if (raw_in_range && frame_span > MIN_CONTRAST_SPAN) {
        if (darker(frame_bright, adc_white_anchor) < 0) {
            adc_white_anchor = (adc_white_anchor * 3 + frame_bright) / 4;
        } else if (slow_tick && darker(frame_bright, adc_white_anchor) > 150) {
            bool not_in_air = (IR_POL < 0) ? (frame_bright > ADC_RAW_LO_OK + 350) : (frame_bright < ADC_RAW_HI_OK - 350);
            if (not_in_air) adc_white_anchor += IR_POL;
        }
    }
    bool valid_contrast = (frame_span > MIN_CONTRAST_SPAN) && (darker(frame_bright, adc_white_anchor) < WHITE_SEEN_MAX_DARK) && raw_in_range;

    float n_raw[4];
    int   eff_span[4];
    int   span_i_arr[4];
    int   max_ch_span = MIN_CONTRAST_SPAN;
    int   span_sum = 0;
    int   black_learn_thr = (track_span_est > MIN_CONTRAST_SPAN * 2) ? (track_span_est / 2) : MIN_CONTRAST_SPAN;

    for (int i = 0; i < 4; i++) {
        int  raw    = sData.raw_ir[i];
        bool raw_ok = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK) : (raw < ADC_RAW_HI_OK);
        bool floor_bright_ok = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK + 400) : (raw < ADC_RAW_HI_OK - 400);

        if (raw_ok && floor_bright_ok && darker(raw, ir_white_cal[i]) < 0) {
            ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            if (darker(raw, adc_white_anchor) < 0) adc_white_anchor = (adc_white_anchor * 3 + raw) / 4;
        }

        if (valid_contrast && !brain.was_lifted && (brain.off_track_t < 0.04f || frame_span < INIT_CONTRAST_SPAN)) {
            int dw = darker(raw, ir_white_cal[i]);
            int white_drift_lim = (frame_span < INIT_CONTRAST_SPAN) ? 600 : 250;
            if (slow_tick && dw < white_drift_lim && darker(ir_white_cal[i], adc_white_anchor) < WHITE_DRIFT_MAX_DARK) {
                ir_white_cal[i] += IR_POL;
            }
            dw = darker(raw, ir_white_cal[i]);
            if (brain.has_seen_line && brain.off_track_t < 0.04f && raw_ok && dw > black_learn_thr && darker(raw, ir_black_cal[i]) > 0) {
                ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
            }
        }

        int min_valid_black = ir_white_cal[i] + IR_POL * MIN_CONTRAST_SPAN;
        int cur_span = darker(ir_black_cal[i], ir_white_cal[i]);
        if (slow_tick && valid_contrast && brain.has_seen_line && !brain.was_lifted && brain.off_track_t < 0.04f && cur_span > MIN_CONTRAST_SPAN + 200) {
            ir_black_cal[i] -= 2 * IR_POL;
        }
        if (darker(ir_black_cal[i], min_valid_black) < 0) ir_black_cal[i] = min_valid_black;

        int span_i = darker(ir_black_cal[i], ir_white_cal[i]);
        span_i_arr[i] = span_i;
        if (span_i > max_ch_span) max_ch_span = span_i;
        span_sum += span_i;
    }

    int shared_span_floor = (max_ch_span * 74) / 100;
    if (shared_span_floor < EFF_SPAN_MIN) shared_span_floor = EFF_SPAN_MIN;
    for (int i = 0; i < 4; i++) {
        int effective_span = (span_i_arr[i] * 90) / 100;   // [V6] 90% giúp giảm bão hòa sớm đỉnh chuông
        if (effective_span < shared_span_floor) effective_span = shared_span_floor;
        eff_span[i] = effective_span;
        n_raw[i] = (float)darker(sData.raw_ir[i], ir_white_cal[i]) / (float)effective_span;
    }
    if (slow_tick && valid_contrast && !brain.was_lifted) {
        track_span_est = (track_span_est * 3 + (span_sum / 4)) / 4;
    }

#if !ENABLE_FLOOR_TRACK
    (void)eff_span;
#endif
#if ENABLE_FLOOR_TRACK
    if (brain.was_lifted) {
        floor_cm = 0.0f;
    } else {
        float n_min = min4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        bool  cm_ok = (frame_span > EFF_SPAN_MIN) && raw_in_range && (brain.off_track_t < 0.04f);
        float cm_tgt = floor_cm;
        if (n_min <= CM_MAX) {
            cm_tgt = max2f(0.0f, n_min - CM_DEAD);
            if (cm_tgt > floor_cm && !cm_ok) cm_tgt = floor_cm;
        }
        float cm_tau = (cm_tgt > floor_cm) ? CM_TAU_UP : CM_TAU_DN;
        floor_cm += (cm_tgt - floor_cm) * (dt / (cm_tau + dt));
        floor_cm = clampf(floor_cm, 0.0f, CM_MAX - CM_DEAD);

        // [V6] Không hấp thụ floor_cm vào ir_white_cal khi cánh đang có dấu hiệu chạm vạch/gờ giảm tốc
        bool wing_busy = brain.w_on[0] || brain.w_on[1] || brain.w_rose[0] || brain.w_rose[1];
        if (slow_tick && cm_ok && !wing_busy && floor_cm > CM_ABSORB_MIN) {
            for (int i = 0; i < 4; i++) {
                int d = (int)(floor_cm * CM_ABSORB_RATE * (float)eff_span[i]);
                int w = ir_white_cal[i] + IR_POL * d;
                if (darker(w, adc_white_anchor) > WHITE_CAL_MAX_DARK) w = adc_white_anchor + IR_POL * WHITE_CAL_MAX_DARK;
                ir_white_cal[i] = w;
            }
        }
    }
#endif

    const float cm_inv = 1.0f / (1.0f - floor_cm);
    for (int i = 0; i < 4; i++) sData.ir[i] = clampf((n_raw[i] - floor_cm) * cm_inv, 0.0f, 1.0f);

    int pwm_l = 0, pwm_r = 0;
    brain.step(sData, dt, prev_hw_pwm_l, prev_hw_pwm_r, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r, dt);

#if ENABLE_BLACKBOX
    if ((loopFrameCount % 10) == 0 && !brain.was_lifted) bbRecord(sData, pwm_l, pwm_r);
    if (brain.was_lifted && !bb_dumped && bb_n > 0) { bbDump(); bb_dumped = true; }
    if (!brain.was_lifted) bb_dumped = false;
#endif

#if ENABLE_SERIAL_TELEMETRY
    if (brain.ev_cross) {
        brain.ev_cross = false;
        Serial.printf("[CROSS] #%d act=%+d odo=%.0f center=%.0f sec=%.2f W=%.1f vs=%.2f\n",
                      brain.ev_idx, brain.ev_act, brain.dist_mm, brain.ev_center_d, brain.cross_sec,
                      brain.line_w_est, brain.v_scale);
    }
    if (brain.ev_align_end) {
        brain.ev_align_end = false;
        Serial.printf("[ALIGN->ARC] t=%.3fs d=%.1fmm rem=%.1fmm wf=%d W=%.1f vs=%.2f\n",
                      brain.ev_al_t, brain.ev_al_d, brain.ev_al_rem, (int)brain.ev_al_wf,
                      brain.line_w_est, brain.v_scale);
    }
    if (brain.ev_pivot_end) {
        brain.ev_pivot_end = false;
        Serial.printf("[ARC->RUN] t=%.3fs z=%.1fmm\n", brain.ev_pv_t, brain.ev_pv_z);
    }
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 120 && Serial.availableForWrite() > 64) {
        lastTelemetryMillis = nowMs;
        Serial.printf("[t=%5.2f] M%d x=%+5.1f vx=%+6.0f u=%+4.0f rk=%.2f | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d) BEMF=(%+4.0f,%+4.0f)\n",
                      sData.elapsed_sec, (int)brain.mode, brain.x, brain.vx, brain.u_cmd, brain.risk_dbg,
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      prev_hw_pwm_l, prev_hw_pwm_r, bemf_l, bemf_r);
    }
#endif
}
