// ============================================================================
//  COBOT LINE FOLLOWER v5.1 - ESP32-C3 - vòng lặp 500 Hz
//  v5.1 = v5 + SỬA LỖI "NGƯỢC KÉP" CẢM BIẾN (các mục [V5.1]):
//    A) THỨ TỰ CHÂN: S1..S4 = GPIO 0,1,3,4 (trái -> phải). v5 khai báo {4,3,1,0} = ngược trái/phải.
//    B) CỰC TÍNH ADC: phần cứng thực tế NỀN TRẮNG = ADC CAO (~2500), VẠCH ĐEN = ADC THẤP (~500).
//       v5 giả định ngược lại (trắng 300 / đen 2400) nên cảm biến "thấy" nền là đen, vạch là trắng.
//       Hai lỗi nhân với nhau -> bám 1 vạch đơn vẫn lái đúng chiều, nhưng ngã tư (4 mắt cùng trên vạch đen)
//       bị hiểu là "4 mắt cùng trên nền trắng" = mất vạch -> xe lao thẳng, không bao giờ phát hiện ngã tư.
//    C) Toàn bộ chỗ phụ thuộc cực tính được viết lại theo một hàm duy nhất darker() (dương = tối hơn):
//       khởi tạo mặc định, đợi đặt xe, hiệu chuẩn lúc khởi động, hiệu chuẩn liên tục, kiểm tra tương phản,
//       chống bão hòa ADC, bộ bám nền (floor tracker), trần mức trắng. Đổi IR_WHITE_IS_HIGH là đủ để đổi cực tính.
//    Lõi điều khiển Brain GIỮ NGUYÊN 100% (nó chỉ nhìn giá trị chuẩn hóa 0.0 = trắng, 1.0 = đen).
//
//  v5 = v4 + các mục [V5]:
//    1) THÍCH ỨNG MÀU NỀN: bộ bám nền chung (floor tracker) - nền đổi tông (xám, bóng đổ, miếng thảm khác màu)
//       thì mức "trắng" của cả 4 cảm biến được nâng/hạ theo trong ~0.1 s, không cần đợi hiệu chuẩn chậm.
//    2) VÀO CUA SỚM HƠN: nguy cơ rời vạch tính cả từ VẬN TỐC trôi của vạch (nhìn xa 80 ms) -> phanh trước khi sai số
//       kịp lớn; KD tăng nhẹ khi vạch trôi nhanh; nới sàn bánh trong sớm hơn.
//    3) MƯỢT HƠN: giảm tốc mềm khi chưa nguy hiểm, giới hạn tốc độ đổi của lái khi đường êm, tích phân (I) không
//       còn "kéo ngược" từ cua trước sang cua sau.
//    4) MẤT VẠCH: chọn phía bẻ theo xu hướng trôi của vạch; bắt lại vạch thì có sẵn vận tốc ban đầu -> ít vọt lố.
//  Tốc độ nền V_MAX / V_CORNER_MIN GIỮ NGUYÊN (V_BRAKE_MIN chỉ dùng thoáng qua khi vạch sắp rời dải cảm biến).
//  Đơn vị: sai số vị trí = mm (dương = vạch lệch về phía S4), PWM = 0..255
//  Quy ước lái (giữ nguyên code gốc): u > 0 -> bánh trái nhanh hơn -> xe quay về phía S4
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
#define ENABLE_FLOOR_TRACK       1   // [V5] 1 = tự thích ứng khi màu nền đổi | 0 = tắt (cảm biến chuẩn hóa như v4)
#define ENABLE_SERIAL_TELEMETRY  0   // 1 = in log 8 Hz (chỉ để debug, tắt khi thi)
#define ENABLE_BLACKBOX          0   // 1 = ghi dữ liệu vào RAM, xả qua USB khi nhấc xe (để chỉnh thông số)

// Hành động ở từng lần gặp ngã tư (theo thứ tự gặp):
//   0 = đi thẳng,  +1 = rẽ về phía S4 (phía sai số dương, như pivot 235/-135 của code gốc),  -1 = rẽ về phía S1
// Sa hình vòng bảng: ngã tư 1 đi thẳng 2 lần (lần vào, lần ra), ngã tư 2 lần đầu rẽ tắt.
#define TURN_RIGHT  (+1)   // về phía S4
#define TURN_LEFT   (-1)   // về phía S1
static const int8_t CROSS_PLAN[] = { 0, 0, TURN_RIGHT };   // lần 1: thẳng | lần 2: thẳng | lần 3: rẽ phải
static const int    CROSS_PLAN_LEN = (int)(sizeof(CROSS_PLAN) / sizeof(CROSS_PLAN[0]));

// ---------------------------- Hình học xe (mm) -----------------------------
static const float SENSOR_X_MM[4]       = { -20.0f, -11.25f, 11.25f, 20.0f };   // S1 (trái) ... S4 (phải)
static const float AXLE_TO_OUTER_ROW_MM = 43.0f;   // trục bánh -> hàng S1/S4
static const float DEFAULT_LINE_W_MM    = 25.0f;   // điểm xuất phát cho bộ tự đo bề rộng vạch (mm); Brain tự đo lại từ S1..S4 khi chạy!
static const float SPEED_MMPS_AT_255    = 500.0f;  // tốc độ danh định ban đầu (mm/s); Brain tự hiệu chỉnh online qua v_scale!

// ---------------------------- Tốc độ ---------------------------------------
static const float V_MAX         = 255.0f;   // tốc độ nền tối đa trên đường thẳng (nhanh nhất)
static const float V_CORNER_MIN  = 165.0f;   // sàn tốc độ nền khi vào cua bình thường (mượt + nhanh)
static const float V_START       = 110.0f;   // tốc độ khởi động
static const float V_SEEK_LINE   = 0.0f;     // [V5.3] đứng yên chờ vạch nếu đặt ở nền trắng (không tự chạy khi chưa thấy vạch)
static const float ACC_UP        = 1150.0f;  // PWM/s, tăng tốc nhanh ra khỏi cua (vẫn bảo vệ pin qua slewAxis)
static const float ACC_DN        = 5500.0f;  // PWM/s, giảm tốc KHẨN (khi nguy cơ rời vạch cao)
static const float K_SPD_U       = 0.65f;    // giảm tốc theo nỗ lực lái (độ cong)
static const float K_SPD_E       = 1.8f;     // giảm tốc theo sai số
static const float E_SPD_DEAD_MM = 6.0f;
static const float F_IN_MIN      = 35.0f;    // bánh trong khi cua thường luôn tiến đều -> cua cực mượt
// [V5] phanh theo nguy cơ rời vạch (nhìn xa), chỉ tác dụng khi vạch trôi nhanh / sắp chạm mép dải cảm biến
static const float V_BRAKE_MIN   = 118.0f;   // sàn tốc độ nền khi cua gắt nguy hiểm (risk_v = 1)
static const float K_SPD_R       = 95.0f;    // PWM trừ thêm khi nguy cơ phanh = 1
static const float ACC_DN_SOFT   = 1800.0f;  // PWM/s, giảm tốc MỀM khi nguy cơ = 0 (nội suy tới ACC_DN khi nguy cơ = 1)
static const float U_ABS_TAU_UP  = 0.05f;    // s: bộ lọc nỗ lực lái (dùng cho giảm tốc) - tăng nhanh
static const float U_ABS_TAU_DN  = 0.09f;    // s: ...giảm chậm vừa -> ra cua tăng tốc êm, không "giật gas"

// ---------------------------- PID (đơn vị mm) ------------------------------
static const float KP          = 5.8f;       // PWM / mm (mềm ở tâm để đi thẳng không rung lắc)
static const float KD          = 0.34f;      // PWM*s / mm (tăng dập dao động khi thoát cua)
static const float KD_NL       = 0.40f;      // [V5] KD_eff = KD * (1 + KD_NL * min(|vx|/KD_NL_VX, 1)): vạch trôi nhanh -> dập mạnh hơn
static const float KD_NL_VX    = 300.0f;     // mm/s
static const float KI          = 40.0f;      // PWM / (mm*s)
static const float I_MAX       = 65.0f;
static const float I_LEAK_TAU  = 0.80f;      // s
static const float I_LEAK_DYN  = 3.5f;       // [V5] 1/s: rò thêm khi vạch đang trôi nhanh (đang chuyển cua)
static const float I_VX_REF    = 150.0f;     // [V5] mm/s: |vx| tại đó rò thêm đạt tối đa và ngừng tích phân
static const float I_LEAK_WRONG = 6.0f;      // [V5] hệ số rò mạnh khi I đang ngược dấu với sai số (tàn dư của cua trước)
static const float I_ACTIVE_MM = 15.0f;      // chỉ tích phân khi |e| < giá trị này
static const float KP_NL_MM    = 18.0f;      // Kp_eff = Kp * (1 + |e| / KP_NL_MM): ở tâm êm, lệch mép bám cực gắt!

// ---------------------------- Bộ lọc vị trí alpha-beta ---------------------
static const float AB_ALPHA = 0.30f;         // lọc mượt hơn -> giảm nhiễu D khi đi thẳng/vào cua
static const float AB_BETA  = 0.035f;
static const float JUMP_MM  = 25.0f;

// ---------------------------- Ước lượng vị trí -----------------------------
static const float W_FLOOR   = 0.035f;  // ngưỡng thấp để vạch mảnh (12-18 mm) nằm giữa S2-S3 không bị mất dấu
static const float WSUM_MIN  = 0.035f;
static const float K_EXT_MM  = 9.0f;
static const float X_LIM_MM  = 30.0f;

// ---------------------------- Mất vạch -------------------------------------
static const float GAP_X_MM       = 10.0f;
static const float GAP_HOLD_MM    = 50.0f;
static const float V_GAP_MAX      = 240.0f;
static const float V_LOST         = 145.0f;
static const float U_LOST_MAX     = 135.0f;   // quay gắt hơn khi mất vạch ở cua gấp
static const float LOST_RAMP_S    = 0.12f;
static const float LOST_GIVEUP_S  = 1.2f;
static const float LOST_PRED_T    = 0.035f;   // [V5] s: phía mất vạch / quyết định "khe hở hay rời vạch" dùng x + T*vx
static const float REACQ_VX       = 150.0f;   // [V5] mm/s: vận tốc ban đầu của vạch khi bắt lại sau lúc rời hẳn (hướng về tâm)

// ---------------------------- Ngã tư [V4] ----------------------------------
static const float WING_ON               = 0.50f;   // cánh > ngưỡng này = đang nằm trên vạch ngang
static const float WING_OFF              = 0.35f;   // cánh < ngưỡng này = đã rời vạch ngang (có trễ)
static const float WING_BASE_MAX         = 0.42f;   // nền cánh phải < giá trị này mới tính là cạnh lên (chịu được lệch nhẹ trên vạch rộng)
static const float WING_SEP_ON           = 0.80f;   // 2 cảm biến cách nhau >= 31 mm cùng > ngưỡng này = có 2 vạch riêng biệt
                                                    //   (chỉ đúng khi bề rộng vạch <= ~30 mm; vạch rộng hơn -> tăng khoảng cách/bỏ điều kiện này)
static const int   WING_DEBOUNCE_TICKS   = 3;       // cánh đen liên tục >= 3 tick (6 ms) mới tính
static const float WING_LATCH_MM         = 65.0f;   // hai cánh chạm vạch ngang cách nhau tối đa bấy nhiêu mm xe tiến
                                                    //   (nới nhẹ để chịu được pin yếu trước lần tự hiệu chỉnh tốc độ đầu tiên)
static const float CROSS_MIN_SPACING_MM  = 180.0f;  // hai ngã tư liên tiếp phải cách nhau >= quãng này (tránh lỡ vòng ngắn nếu v_est thấp)
static const float CROSS_PASS_MAX_MM     = 200.0f;  // chốt chặn khi đi thẳng qua ngã tư
static const float CROSS_PASS_MAX_S      = 0.80f;
static const float CROSS_EXIT_MARGIN_MM  = 15.0f;   // đi thêm quãng này sau khi cánh cuối (ước lượng) rời vạch ngang
static const float U_REF_TAU             = 0.20f;   // s: lái "sạch" (chưa bị vạch ngang làm nhiễu) để giữ hướng qua ngã tư

// ---------------------------- Rẽ tắt (pivot) [V4] --------------------------
static const float V_ALIGN           = 62.0f;    // [V5.3] hãm thấp hơn để không trôi quá tâm ngã tư trước khi pivot
static const float ALIGN_FALLBACK_MM = 88.0f;    // trần cứng tránh lố xa tâm ngã tư
static const float ALIGN_FALLBACK_S  = 0.42f;
static const float P_CLEAR           = 140.0f;   // pivot nhanh: cho tới khi vạch cũ rời hẳn dải cảm biến
static const float P_SEEK            = 100.0f;   // pivot chậm: từ lúc vạch cũ đã rời tới khi bắt được nhánh mới
static const float P_APPROACH        = 85.0f;    // [V5.3] giảm nhẹ khi nhánh mới vừa chạm rìa để tránh quét lố
static const float PIVOT_OLD_ON      = 0.50f;    // cánh phía ngược chiều quay > giá trị này (2 tick) = vạch cũ đang quét qua
static const float PIVOT_OLD_GONE    = 0.25f;    // ...rồi < giá trị này = vạch cũ đã rời khỏi dải cảm biến
static const float PIVOT_NEW_OUTER   = 0.76f;    // [V5.3] cánh phía chiều quay phải bám rõ (hoặc mắt giữa chạm nhánh mới) mới bàn giao
static const float PIVOT_RELAX_T     = 0.30f;    // s: quá lâu mà chưa thấy vạch cũ quét qua -> bỏ điều kiện đó
static const float T_PIVOT_MAX       = 1.25f;
static const float PIVOT_SWEEP_MMPS  = 300.0f;
static const float HANDOVER_T        = 0.28f;
static const float V_HANDOVER        = 90.0f;
static const float F_IN_HANDOVER     = -50.0f;
static const float U_MAX_HANDOVER    = 125.0f;


// ---------------------------- [V3] Lái chủ động khi vào cua ----------------
// Ý tưởng: khi vạch SẮP rời dải cảm biến (dự báo x sau T_PRED giây), cho phép bánh trong chậm hơn sàn F_IN_MIN
// để có thêm lực quay. Bình thường (đường thẳng, cua nhẹ) mọi thứ y hệt v2.
static const float T_PRED     = 0.055f;  // s   nhìn trước bao lâu để dự báo vị trí vạch   [V5: 0.045 -> 0.055, dead-time động cơ ~40-50 ms]
static const float RISK_X0    = 8.0f;    // mm  |x_dự báo| < giá trị này: nguy cơ = 0 (hành vi như v2)   [V5: 9 -> 8]
static const float RISK_X1    = 20.0f;   // mm  |x_dự báo| >= giá trị này: nguy cơ = 1 (nới sàn tối đa)
static const float F_IN_EMERG = 0.0f;    // sàn bánh trong khi bám vạch ở cua gắt (0 = dừng hẳn bánh trong, không đảo chiều -> không bị khựng zero-cross)
static const float F_IN_LOST  = -35.0f;  // khi thực sự mất vạch ở cua siêu gắt: cho phép bánh trong phanh ngược nhẹ để quay gắt bắt lại line

// ---------------------------- [V5] Nguy cơ phanh + độ mượt của lái ---------
static const float T_PRED_V    = 0.085f;  // s   nhìn XA hơn cho phanh (phanh cần nhiều thời gian hơn quay)
static const float RISK_V_X0   = 8.0f;    // mm  |x dự báo xa| < giá trị này: không phanh thêm
static const float RISK_V_X1   = 22.0f;   // mm  >= giá trị này: phanh tối đa
static const float U_SLEW_CALM = 2800.0f; // PWM/s: lái siêu êm khi đi thẳng / cua đều (triệt tiêu rung lắc)
static const float U_SLEW_FAST = 30000.0f;// PWM/s: khi nguy cơ = 1 (mở hết giới hạn để phản ứng tức thì)

// ---------------------------- [V5] Thích ứng màu nền -----------------------
// n_i = (raw_i - white_i) / span_i. Trên đường bám bình thường luôn có >= 1 cảm biến nằm trên nền trắng, nên
// cảm biến "tối nhất" (n_min) chính là mức nâng của nền. Nền đổi tông -> cả 4 cảm biến nâng cùng lúc -> n_min tăng.
// Ngã tư / vạch xiên rộng phủ cả 4 cảm biến -> n_min vọt > CM_MAX -> coi là vạch, ĐÓNG BĂNG (không học nhầm).
static const float CM_DEAD        = 0.06f;  // phần nâng < giá trị này = nhiễu / quầng vạch -> bỏ qua
static const float CM_MAX         = 0.40f;  // nền nâng tối đa có thể bám; n_min cao hơn = vạch, không phải nền
static const float CM_TAU_UP      = 0.08f;  // s: nền sáng -> tối (n_min tăng): học chậm hơn để vạch lướt qua không bị nhầm là nền
static const float CM_TAU_DN      = 0.015f; // s: nền tối -> sáng: nhả rất nhanh
static const float CM_ABSORB_MIN  = 0.04f;  // nền nâng > mức này lâu thì "chuyển" dần vào mức trắng hiệu chuẩn
static const float CM_ABSORB_RATE = 0.05f;  // phần của (cm*span) chuyển vào ir_white_cal mỗi slow tick (64 ms)

// ============================================================================
// 2. CHÂN PHẦN CỨNG + CỰC TÍNH CẢM BIẾN
// ============================================================================
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define CH_L_A 0
#define CH_L_B 1
#define CH_R_A 2
#define CH_R_B 3

// [V5.1] S1 = trái ngoài cùng ... S4 = phải ngoài cùng, KHỚP với SENSOR_X_MM[] = {-20, -11.25, +11.25, +20}
#define SENSOR_1_PIN 0
#define SENSOR_2_PIN 1
#define SENSOR_3_PIN 3
#define SENSOR_4_PIN 4

#define W_LED_ON  20
#define IR_LED_ON 21

// [V5.1] CỰC TÍNH ADC của mạch phản xạ IR.
//   1 = NỀN TRẮNG phản xạ mạnh -> ADC CAO (~2500), VẠCH ĐEN -> ADC THẤP (~500)   <- phần cứng thực tế của xe này
//   0 = ngược lại (đen = ADC cao). Chỉ dùng nếu đổi mạch cảm biến.
// Mọi chỗ cần biết "bên nào là tối hơn" đều đi qua IR_POL và darker(), KHÔNG so sánh > / < trực tiếp trên ADC.
#ifndef IR_WHITE_IS_HIGH
#define IR_WHITE_IS_HIGH 1
#endif
#if IR_WHITE_IS_HIGH
static const int IR_POL        = -1;     // dấu của (đen - trắng) trên thang ADC
static const int ADC_WHITE_NOM = 2500;   // mức trắng danh định (chỉ là điểm xuất phát / dự phòng, sẽ tự hiệu chuẩn)
static const int ADC_BLACK_NOM = 500;    // mức đen danh định
#else
static const int IR_POL        = +1;
static const int ADC_WHITE_NOM = 300;
static const int ADC_BLACK_NOM = 2400;
#endif

// Ngưỡng ADC (tương thích cả mắt trắng đọc tới 4095 trên phần cứng thực tế)
static const int ADC_RAW_LO_OK       = 20;     // raw <= giá trị này = chạm đáy ADC/mạch hở  -> không dùng để học
static const int ADC_RAW_HI_OK       = 3850;   // ngưỡng trần chỉ dùng khi IR_WHITE_IS_HIGH == 0
static const int ADC_FRAME_HI_OK     = 3950;
static const int MIN_CONTRAST_SPAN   = 400;    // tương phản tối thiểu giữa trắng-đen (chịu được bạt in bóng / pin yếu)
static const int INIT_CONTRAST_SPAN  = 1000;   // [V5.3] tương phản tối thiểu khi khởi động để coi là đang đè lên vạch đen thật
static const int EFF_SPAN_MIN        = 350;    // biên độ chuẩn hóa tối thiểu (tránh kẹp IR = 1.00 khi gặp nhiễu)
static const int WHITE_SEEN_MAX_DARK = 1600;   // cho phép nền sẫm màu hơn vẫn học được
static const int WHITE_DRIFT_MAX_DARK= 800;    // mức trắng được phép trôi chậm về phía tối tối đa bấy nhiêu so với mốc nền
static const int WHITE_CAL_MAX_DARK  = 1200;   // trần khi "hấp thụ" nền tối vào mức trắng (so với mốc nền)

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

// [V5.1] Hàm DUY NHẤT biết cực tính ADC.
//   darker(a, ref) > 0  <=>  mẫu a TỐI hơn mẫu ref  (không phụ thuộc đen là ADC cao hay thấp)
static inline int darker(int a, int ref)    { return IR_POL * (a - ref); }
static inline int darkerOf(int a, int b)    { return (darker(a, b) >= 0) ? a : b; }   // mẫu tối hơn
static inline int brighterOf(int a, int b)  { return (darker(a, b) <= 0) ? a : b; }   // mẫu sáng hơn

struct SensorData {
    float ir[4];      // 0.0 = trắng, 1.0 = đen
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
// 4. BỘ ĐIỀU KHIỂN TRUNG TÂM  (KHÔNG THAY ĐỔI so với v5)
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

    float x, vx;
    bool  need_reinit;
    float integ, u_abs_f, u_cmd;
    float v_cmd, v_est, dist_mm, v_scale;
    bool  v_cal_valid;
    float line_w_est, edge_lambda;
    bool  line_w_valid;
    int   last_side;

    bool  in_loss, meas_ok;
    float lost_t, lost_d, lost_x;
    int   lost_side;
    float handover_t;
    float reacq_vx;      // [V5] vận tốc ban đầu cho bộ lọc khi bắt lại vạch sau lúc rời hẳn
    float risk_dbg;      // [V5] nguy cơ (0..1) của tick gần nhất, chỉ để telemetry

    bool  was_lifted, has_seen_line;
    float off_track_t, air_t, run_t;

    // [V4] theo dõi hai cánh S1 (k=0) và S4 (k=1); mọi khoảng cách là odometer (mm)
    bool  base_init;
    float base_o[2];
    bool  w_on[2], w_rose[2], w_fell[2], w_sep;
    int   w_ticks[2];
    float w_cand_d[2], w_rise_d[2], w_fall_d[2], w_half_fall_d[2], w_last_d[2];
    float cross_sec, center_odo, end_odo, align_rem_mm;   // 1/cos(góc xiên), tâm ngã tư, điểm rời, quãng còn lại tới tâm trục bánh
    bool  center_refined;
    float u_ref, u_hold;
    int   cross_confirm, cross_count;
    float last_cross_dist;
    int   clear_ticks, white_ticks;
    int   turn_dir;
    bool  pivot_old_seen;
    bool  pivot_has_z;
    float pivot_prev_z, pivot_vx_est;
    // sự kiện cho telemetry
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
        line_w_est = DEFAULT_LINE_W_MM; edge_lambda = 2.8f; line_w_valid = false;
        last_side = 1;
        in_loss = false; meas_ok = false;
        lost_t = 0.0f; lost_d = 0.0f; lost_x = 0.0f; lost_side = 1;
        handover_t = 0.0f; reacq_vx = 0.0f; risk_dbg = 0.0f;
        was_lifted = false; has_seen_line = false;
        off_track_t = 0.0f; air_t = 0.0f; run_t = 0.0f;
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

    // [V5.3] Phân biệt vạch đen thật với nền trắng/xám phẳng bị lệch độ nhạy giữa các mắt
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
        // Vạch mảnh (12-18 mm) nằm giữa S2-S3
        return (in_max >= 0.055f && out_min < 0.05f && (in_max - out_max) >= 0.04f);
    }

    // ------------------------------------------------------------------------
    void step(const SensorData& sd, float dt, int& pl, int& pr) {
        pl = 0; pr = 0;
        if (!(dt > 0.0005f && dt < 0.02f)) dt = 0.002f;
        const float* s = sd.ir;

        // ---- Nhấc xe / nền trắng không vạch / đặt xe ----
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
            } else {
                off_track_t += dt;
            }
        } else if ((no_line || all_black) && !spinning && !(in_cross && no_line)) {
            off_track_t += dt;
            if (all_black && mode == M_RUN && off_track_t > 0.25f) was_lifted = true;
            else if (off_track_t > 0.65f)                          was_lifted = true;
        } else {
            off_track_t = 0.0f;
        }
        if (was_lifted) { stopOutputs(); return; }

        run_t += dt;
        updateSpeedEstimate(dt);

        // ---- Chưa thấy vạch: đứng yên chờ đặt lên vạch (tránh tự chạy trên nền trắng) ----
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

    // dn = tốc độ giảm tối đa (PWM/s); mặc định giảm khẩn ACC_DN như v4
    void rampSpeed(float target, float dt, float dn = ACC_DN) {
        if (target > v_cmd) { v_cmd += ACC_UP * dt; if (v_cmd > target) v_cmd = target; }
        else                { v_cmd -= dn * dt;     if (v_cmd < target) v_cmd = target; }
    }

    void updateSpeedEstimate(float dt) {
        float c = 0.5f * (float)(prev_pl + prev_pr);
        if (c < 0.0f) c = 0.0f;
        // Tính đến deadzone MIN_N20_PWM để khi phanh về V_ALIGN = 85 không đánh giá thấp tốc độ thực
        float c_hw  = (c > 1.0f) ? ((float)MIN_N20_PWM + c * (255.0f - (float)MIN_N20_PWM) / 255.0f) : 0.0f;
        float c_eff = 0.5f * (c + c_hw);
        float tgt   = v_scale * SPEED_MMPS_AT_255 * (c_eff / 255.0f);
        // Quán tính trôi khi giảm tốc chậm hơn lúc kéo ga -> bám sát đà xe khi vào ngã tư
        float tau = (tgt < v_est) ? 0.105f : 0.040f;
        v_est += (tgt - v_est) * (dt / (tau + dt));
        dist_mm += v_est * dt;
    }

    // ---- Ước lượng vị trí vạch (mm) từ 4 cảm biến; false nếu không thấy vạch ----
    bool measure(const float* s, float& z) const {
        if (!hasRealLine(s) && min4f(s[0], s[1], s[2], s[3]) <= 0.85f) return false;
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

        // Ngoại suy khi vạch chỉ còn chạm cảm biến rìa
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

    // Nghịch đảo CDF chuẩn (Abramowitz & Stegun 26.2.23, sai số < 4.5e-4): khớp chính xác hồ sơ vệt sáng IR
    static inline float normInv(float p) {
        p = clampf(p, 0.005f, 0.995f);
        float a = (p < 0.5f) ? p : (1.0f - p);
        float t = sqrtf(-2.0f * logf(a));
        float z = t - (2.515517f + 0.802853f * t + 0.010328f * t * t) /
                      (1.0f + 1.432788f * t + 0.189269f * t * t + 0.001308f * t * t * t);
        return (p < 0.5f) ? -z : z;
    }

    // Tự đo bề rộng vạch (mm) và độ mờ quang học edge_lambda (sigma, mm) trực tiếp từ tọa độ vật lý SENSOR_X_MM.
    // Không phụ thuộc tốc độ, điện áp pin, hay loại nền; tự thích ứng nhanh nếu bề rộng vạch biến thiên dọc sân.
    void updateLineWidth(const float* s, float dt) {
        if (in_loss || handover_t > 0.0f || w_sep || (w_rose[0] && w_rose[1])) return;
        if (fabsf(vx) > 350.0f) return;
        if ((s[0] - base_o[0]) > 0.25f && (s[3] - base_o[1]) > 0.25f) return;
        // Nếu cả 2 cánh ngoài đều chớm thấy vạch trong khi 2 mắt giữa không bão hòa -> chắc chắn là ngã tư đang tới, đóng băng W
        if (s[0] > 0.03f && s[3] > 0.03f && (s[1] < 0.75f || s[2] < 0.75f)) return;

        bool clean_edge_lam = !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0 &&
                              (s[0] - base_o[0]) < 0.12f && (s[3] - base_o[1]) < 0.12f;
        if (clean_edge_lam && s[0] > 0.03f && s[0] < 0.97f && s[1] > 0.03f && s[1] < 0.97f && (s[1] - s[0]) > 0.15f) {
            float dl = normInv(s[1]) - normInv(s[0]);
            if (dl > 0.8f && dl < 7.0f) {
                float lam_obs = clampf(8.75f / dl, 1.2f, 6.0f);
                edge_lambda += (lam_obs - edge_lambda) * (dt / (0.10f + dt));
            }
        }
        if (clean_edge_lam && s[3] > 0.03f && s[3] < 0.97f && s[2] > 0.03f && s[2] < 0.97f && (s[2] - s[3]) > 0.15f) {
            float dl = normInv(s[2]) - normInv(s[3]);
            if (dl > 0.8f && dl < 7.0f) {
                float lam_obs = clampf(8.75f / dl, 1.2f, 6.0f);
                edge_lambda += (lam_obs - edge_lambda) * (dt / (0.10f + dt));
            }
        }

        float w_obs = -1.0f;
        if (s[0] <= s[1] + 0.02f && s[3] <= s[2] + 0.02f && s[1] > 0.04f && s[2] > 0.04f) {
            bool has_L = false, has_R = false;
            float xL = 0.0f, xR = 0.0f;
            if (s[0] >= 0.03f && s[0] <= 0.95f)      { xL = SENSOR_X_MM[0] - edge_lambda * normInv(s[0]); has_L = true; }
            else if (s[1] >= 0.03f && s[1] <= 0.95f) { xL = SENSOR_X_MM[1] - edge_lambda * normInv(s[1]); has_L = true; }

            if (s[3] >= 0.03f && s[3] <= 0.95f)      { xR = SENSOR_X_MM[3] + edge_lambda * normInv(s[3]); has_R = true; }
            else if (s[2] >= 0.03f && s[2] <= 0.95f) { xR = SENSOR_X_MM[2] + edge_lambda * normInv(s[2]); has_R = true; }

            if (has_L && has_R) w_obs = xR - xL;
        } else if (fabsf(s[1] - s[2]) < 0.04f && s[0] < 0.04f && s[3] < 0.04f && s[1] > 0.008f && s[2] > 0.008f) {
            float sc = clampf(0.5f * (s[1] + s[2]), 0.01f, 0.95f);
            w_obs = 2.0f * (SENSOR_X_MM[2] + edge_lambda * normInv(sc));
        }
        if (w_obs < 0.0f && s[1] > 0.95f && s[2] > 0.95f && s[0] < 0.03f && s[3] < 0.03f &&
            fabsf(s[1] - s[2]) < 0.04f && fabsf(s[0] - s[3]) < 0.04f) {
            if (line_w_est < 27.0f || line_w_est > 35.0f) w_obs = 31.25f;
        }

        if (w_obs >= 11.0f && w_obs <= 45.0f) {
            float w_clamped = clampf(w_obs, 12.0f, 44.0f);
            float tau = line_w_valid ? 0.08f : 0.04f;
            line_w_est += (w_clamped - line_w_est) * (dt / (tau + dt));
            line_w_valid = true;
        }
    }

    // ---- [V4] Theo dõi hai cánh S1,S4 ---------------------------------------------------------
    // Ghi odometer tại cạnh LÊN / cạnh XUỐNG của vạch ngang trên từng cánh.
    // Hai cánh KHÔNG cần chạm vạch ngang cùng lúc: chỉ cần cả hai đã chạm trong vòng WING_LATCH_MM xe tiến
    // => đúng với ngã tư xiên và xe vào lệch góc (hai cánh chạm vạch cách nhau 40*tan(góc xiên) mm).
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
                if (v >= WING_ON) w_half_fall_d[k] = dist_mm;   // bám sát mép xuống 50% để đo bề rộng vạch không bị trễ hysteresis
                if (v < WING_OFF) { w_on[k] = false; w_fell[k] = true; w_fall_d[k] = dist_mm; }
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
                // Xóa latch nếu cánh kia không chạm kịp HOẶC xung đã tắt >18 mm mà không có bằng chứng tách nhánh
                w_rose[k] = false; w_fell[k] = false; w_ticks[k] = 0;
            }
        }
        // Bằng chứng "ngã tư thực sự" (không nhầm với vạch đơn khi cua):
        //   A) Có khe giảm sáng giữa cánh ngoài và vạch chính (2 đỉnh tách biệt)
        //   B) Cả hai cánh ngoài cùng đen đồng thời (độ phủ ngang >= 40 mm) và hai mắt giữa cũng có vạch
        //   C) Nhánh xiên mạnh: cánh ngoài bão hòa >=0.80 và KHÔNG tối thua mắt giữa kề bên, trong khi phía kia vẫn thấy vạch chính
        if (w_rose[0] || w_rose[1] || w_ticks[0] > 0 || w_ticks[1] > 0) {
            bool dip_sep    = (s[0] > 0.48f && s[1] < s[0] - 0.05f && s[2] > 0.16f) ||
                              (s[3] > 0.48f && s[2] < s[3] - 0.05f && s[1] > 0.16f);
            bool both_outer = (s[0] > 0.48f && s[3] > 0.48f && (s[1] > 0.16f || s[2] > 0.16f));
            bool edge_skew  = (s[0] > 0.80f && s[0] >= s[1] - 0.22f && s[2] > 0.35f) ||
                              (s[3] > 0.80f && s[3] >= s[2] - 0.22f && s[1] > 0.35f);
            if (dip_sep || both_outer || edge_skew) w_sep = true;
        } else {
            w_sep = false;
        }
        updateWingBase(s, dt);
    }

    // Ngã tư = cả hai cánh đã chạm vạch ngang (mỗi cánh đã lọc 3 tick = 6 ms), có bằng chứng hai vạch riêng biệt,
    // cảm biến giữa còn ở vạch chính (hoặc cả hai cánh ngoài đang đè vạch ở ngã ba chữ T), và góc xiên hợp lý.
    bool detectCross(const float* s, bool allowed) {
        float skew_max = v_cal_valid ? 55.0f : 70.0f;
        bool skew_ok = fabsf(w_rise_d[1] - w_rise_d[0]) <= skew_max;
        bool center = (s[1] > 0.16f || s[2] > 0.16f || (s[0] > 0.45f && s[3] > 0.45f));
        bool trig = allowed && w_rose[0] && w_rose[1] && w_sep && center && skew_ok;
        cross_confirm = trig ? (cross_confirm + 1) : 0;
        return cross_confirm >= 1;
    }

    // Lập kế hoạch qua ngã tư ngay khi phát hiện, dùng bề rộng vạch tự đo line_w_est (không hardcode):
    //   độ lệch thời điểm hai cánh chạm vạch -> góc xiên -> bề dày vạch dọc đường đi = line_w_est / cos(góc)
    //   tâm ngã tư (odometer) = trung điểm hai cạnh lên + nửa bề dày | cánh cuối rời vạch = cạnh lên muộn nhất + bề dày
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

    // Tự hiệu chỉnh tốc độ thực v_scale & căn tâm ngã tư:
    //   - Ưu tiên đo từ CẢ HAI cánh khi cả hai đều rời vạch sạch sẽ
    //   - Trong M_TURN_ALIGN, nếu cánh phía rẽ bị dính nhánh cong không rời vạch sau 60 ms, dùng cánh ĐỐI DIỆN hướng rẽ
    void refineCenter() {
        if (center_refined) return;
        float L_meas = -1.0f;
        float fall_mid = dist_mm;
        float L_min_ok = 0.42f * line_w_est;
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
        } else if (mode == M_TURN_ALIGN && (mode_t > 0.055f || (w_fell[turn_dir > 0 ? 0 : 1] && (dist_mm - w_half_fall_d[turn_dir > 0 ? 0 : 1]) > 6.0f))) {
            int outer_k = (turn_dir > 0) ? 0 : 1;
            if (w_fell[outer_k]) {
                float Lo = w_half_fall_d[outer_k] - w_rise_d[outer_k];
                if (Lo > L_min_ok && Lo < L_max_ok) {
                    center_refined = true;
                    L_meas = Lo;
                    // Bù nửa độ lệch xiên để quy chiếu đúng về tâm giữa S1-S4
                    fall_mid = w_half_fall_d[outer_k] + 0.5f * (w_rise_d[1 - outer_k] - w_rise_d[outer_k]);
                }
            }
        }
        if (L_meas <= 0.0f) return;

        float d_skew = fabsf(w_rise_d[1] - w_rise_d[0]);
        float span   = SENSOR_X_MM[3] - SENSOR_X_MM[0];
        float skew_term = (line_w_est * d_skew) / span;
        float denom_sq  = max2f(L_meas * L_meas - skew_term * skew_term, 0.25f * L_meas * L_meas);
        float r = clampf(line_w_est / sqrtf(denom_sq), 0.42f, 2.20f);

        float v_scale_old = v_scale;
        float v_scale_obs = clampf(v_scale_old * r, 0.40f, 2.20f);
        float blend = !v_cal_valid ? 1.0f : ((mode == M_TURN_ALIGN && !(w_fell[0] && w_fell[1])) ? 0.35f : 0.45f);
        v_scale = clampf(v_scale_old + blend * (v_scale_obs - v_scale_old), 0.40f, 2.20f);
        v_cal_valid = true;
        float r_eff = v_scale / v_scale_old;
        v_est *= r_eff;

        float t_corr = clampf((r_eff * d_skew) / span, 0.0f, 3.0f);
        cross_sec = sqrtf(1.0f + t_corr * t_corr);
        float L_true = line_w_est * cross_sec;

        if (mode == M_TURN_ALIGN) {
            float past_fall_real = (dist_mm - fall_mid) * r_eff;
            float rem = AXLE_TO_OUTER_ROW_MM - 0.5f * L_true - past_fall_real;
            align_rem_mm = min2f(rem, AXLE_TO_OUTER_ROW_MM);
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
        u_hold = u_ref;   // [V4] lái "sạch" từ trước khi vạch ngang xuất hiện (u_cmd lúc này đã bị vạch ngang xiên làm nhiễu)
        clear_ticks = 0; white_ticks = 0;
        mode_t = 0.0f; mode_d = 0.0f;
        cross_confirm = 0;
        planCross();
        align_rem_mm = max2f(0.0f, (center_odo + AXLE_TO_OUTER_ROW_MM) - dist_mm);
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
        bool allowed = (handover_t <= 0.08f) && !in_loss && (run_t > 0.15f) &&
                       ((dist_mm - last_cross_dist) >= CROSS_MIN_SPACING_MM);
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
                    // [V5] rời hẳn dải cảm biến (không phải khe hở): xe đang bẻ hết cỡ về phía vạch -> vạch sẽ trôi VỀ TÂM
                    reacq_vx = (fabsf(lost_x) >= GAP_X_MM) ? -(float)lost_side * REACQ_VX : 0.0f;
                }
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
            if (need_reinit) { x = z; vx = reacq_vx; reacq_vx = 0.0f; need_reinit = false; }
            if (fabsf(x) > 3.0f) last_side = (x > 0.0f) ? 1 : -1;
            updateLineWidth(s, dt);
        } else {
            if (!in_loss) {
                in_loss = true; lost_t = 0.0f; lost_d = 0.0f;
                // [V5] phía + độ lệch lúc mất vạch tính theo x DỰ BÁO: vạch đang trôi nhanh ra ngoài thì dù x còn nhỏ
                // đó cũng là rời vạch thật (không phải khe hở trên đường thẳng)
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

            // [V3] Nguy cơ rời vạch cho LÁI: vị trí hiện tại VÀ vị trí dự báo T_PRED giây tới (x + T*vx).
            // Nguy cơ cao -> nới sàn bánh trong -> trần lái a_cap tăng (xe quay gắt hơn đúng lúc cần).
            float x_pred = x + T_PRED * vx;
            risk = clampf((max2f(fabsf(x_pred), ae) - RISK_X0) / (RISK_X1 - RISK_X0), 0.0f, 1.0f);
            // [V5] Nguy cơ cho PHANH: nhìn xa hơn (T_PRED_V). Vạch trôi 300 mm/s -> báo động khi x mới ~0 mm,
            // trước khi sai số kịp lớn -> tốc độ giảm sớm, bánh có thời gian quay.
            float x_pred_v = x + T_PRED_V * vx;
            risk_v = clampf((max2f(fabsf(x_pred_v), ae) - RISK_V_X0) / (RISK_V_X1 - RISK_V_X0), 0.0f, 1.0f);
            if (!handover) {
                f_in  = F_IN_MIN + (F_IN_EMERG - F_IN_MIN) * risk;
                a_cap = 0.5f * (255.0f - f_in);
            }

            float kp = KP * (1.0f + ae / KP_NL_MM);
            float kd = KD * (1.0f + KD_NL * min2f(fabsf(vx) / KD_NL_VX, 1.0f));
            float u_raw = kp * x + integ + kd * vx;
            bool  sat = fabsf(u_raw) > a_cap;

            // [V5] Tích phân: chỉ tích khi vạch gần đứng yên (cua đều / đường thẳng); khi vạch đang trôi nhanh (chuyển
            // cua) thì ngừng tích + rò nhanh; I ngược dấu sai số (tàn dư cua trước) bị xả mạnh -> vào cua kế không bị kéo ngược.
            float dyn = clampf(fabsf(vx) / I_VX_REF, 0.0f, 1.0f);
            if (ae < I_ACTIVE_MM && !(sat && ((u_raw > 0.0f) == (x > 0.0f)))) integ += KI * x * dt * (1.0f - dyn);
            float leak = 1.0f / I_LEAK_TAU + I_LEAK_DYN * dyn;
            if (ae > 3.0f && integ * x < 0.0f) leak *= I_LEAK_WRONG;
            integ -= integ * (leak * dt);
            integ = clampf(integ, -I_MAX, I_MAX);

            u = clampf(u_raw, -a_cap, a_cap);
            // [V5] Giới hạn tốc độ đổi của lái khi đường êm (cắt xung nhiễu cảm biến -> xe không rung);
            // nguy cơ tăng thì nới dần tới gần như không giới hạn nên không làm chậm phản ứng lúc cần.
            if (!handover) {
                float lim = (U_SLEW_CALM + (U_SLEW_FAST - U_SLEW_CALM) * max2f(risk, risk_v)) * dt;
                u = clampf(u, u_cmd - lim, u_cmd + lim);
                u = clampf(u, -a_cap, a_cap);
            }

            float uf_tau = (fabsf(u) > u_abs_f) ? U_ABS_TAU_UP : U_ABS_TAU_DN;
            u_abs_f += (fabsf(u) - u_abs_f) * (dt / (uf_tau + dt));
            v_t = V_MAX - K_SPD_U * u_abs_f - K_SPD_E * max2f(0.0f, ae - E_SPD_DEAD_MM) - K_SPD_R * risk_v;
            float v_floor = V_CORNER_MIN - (V_CORNER_MIN - V_BRAKE_MIN) * risk_v;
            v_t = clampf(v_t, v_floor, V_MAX);
            dn_rate = ACC_DN_SOFT + (ACC_DN - ACC_DN_SOFT) * risk_v;   // êm khi chưa nguy hiểm, khẩn khi cần
        } else {
            float lx = fabsf(lost_x);
            if (lx < GAP_X_MM && lost_d < GAP_HOLD_MM) {
                // Khe hở trên vạch thẳng: giữ hướng, giữ I
                u = clampf(integ, -a_cap, a_cap);
                v_t = min2f(v_cmd, V_GAP_MAX);
            } else {
                // Vạch rời khỏi dải cảm biến: bẻ về phía vạch vừa rời, tăng dần.
                // Khi mất vạch ở cua siêu gắt thì dùng F_IN_LOST (phanh ngược nhẹ bánh trong để bắt lại vạch ngay).
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
            float v_ho = (fabsf(x) > 7.0f || fabsf(vx) > 110.0f) ? V_HANDOVER : 135.0f;
            if (v_t > v_ho) v_t = v_ho;
        }
        if (off_track_t > 0.03f && !in_loss) {
            v_t = min2f(v_t, V_LOST);
        }
        rampSpeed(v_t, dt, dn_rate);
        mixWheels(v_cmd, u, f_in, vl, vr);
        u_cmd = u;
        risk_dbg = max2f(risk, risk_v);
        // [V4] lái "sạch": chỉ cập nhật khi chưa có cánh nào chạm vạch ngang (xem enterCross)
        if (!handover && !in_loss && !w_rose[0] && !w_rose[1] && w_ticks[0] == 0 && w_ticks[1] == 0)
            u_ref += (u - u_ref) * (dt / (U_REF_TAU + dt));
    }

    // ---- Đi thẳng qua ngã tư ----
    // Giữ hướng (lái "sạch" tắt dần) cho tới khi cánh cuối rời vạch ngang. Thoát khi: cả hai cánh trắng (đo được),
    // hoặc xe đã tiến qua điểm end_odo (ước lượng, dùng khi cánh dính vạch chính), hoặc hết chốt chặn.
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

    // ---- Rẽ tắt ----
    // Tiến thẳng (giảm tốc về V_ALIGN, lái tắt dần) tới khi trục bánh trùng tâm ngã tư.
    // Bù quán tính trượt khi bắt đầu đảo bánh pivot (~0.025*v_est) và KHÔNG chờ kẹt 0.45s nếu 1 cánh dính nhánh cong.
    void outAlign(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        mode_d += v_est * dt;
        align_rem_mm -= v_est * dt;
        trackWings(s, dt);
        refineCenter();
        rampSpeed(V_ALIGN, dt);
        u_hold -= u_hold * (dt / 0.10f);
        mixWheels(v_cmd, u_hold, F_IN_MIN, vl, vr);
        u_cmd = u_hold;
        ev_center_d = center_odo;
        float pivot_lead_mm = clampf(6.0f + 0.045f * v_est, 6.0f, 22.0f);
        int outer_k = (turn_dir > 0) ? 0 : 1;
        bool wings_cleared = (w_fell[0] && w_fell[1]);
        bool both_still_on_bar = (s[0] > 0.50f && s[3] > 0.50f);
        // Không kẹt chờ 0.35s khi 1 cánh dính nhánh rẽ: cánh ngoài rơi hoặc đã qua vạch ngang là đủ điều kiện rẽ
        bool can_exit_by_dist = (wings_cleared || w_fell[outer_k] || !both_still_on_bar || mode_t > (v_cal_valid ? 0.08f : 0.14f));
        bool ready_by_dist = (align_rem_mm <= pivot_lead_mm) && can_exit_by_dist;
        if (ready_by_dist || mode_d > ALIGN_FALLBACK_MM || mode_t > ALIGN_FALLBACK_S) {
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

    // Pivot: quay về phía rẽ. Khi quay, mọi vạch trong tầm nhìn đều trôi về phía NGƯỢC chiều quay, nhánh mới đi vào từ
    // phía CÙNG chiều quay. Cánh ngoài cùng phía ngược chiều quay (S1 nếu rẽ phải) là "cửa ra" của vạch cũ:
    //   1) cánh đó đen (vạch cũ quét qua)  2) rồi trắng lại = vạch cũ đã rời hẳn dải cảm biến
    //   3) từ lúc đó, cảm biến nào còn thấy vạch đều là NHÁNH MỚI -> giao cho PID.
    // Không cần khoảng trắng giữa hai vạch -> chạy với mọi góc nhánh, vạch rộng, trục lệch tâm.
    // Chờ vạch cũ rời hẳn là then chốt với nhánh thoải: giao sớm thì trọng tâm của CẢ HAI vạch kéo xe quay về vạch cũ.
    void outPivot(const float* s, float dt, float& vl, float& vr) {
        mode_t += dt;
        float old_out = (turn_dir > 0) ? s[0] : s[3];
        float inner_on = (line_w_est < 22.0f) ? 0.25f : 0.40f;
        bool  new_in  = (turn_dir > 0) ? (s[1] > inner_on || s[2] > inner_on || s[3] > PIVOT_NEW_OUTER)
                                       : (s[2] > inner_on || s[1] > inner_on || s[0] > PIVOT_NEW_OUTER);
        if (old_out > PIVOT_OLD_ON) { if (++white_ticks >= 2) pivot_old_seen = true; }
        else white_ticks = 0;
        bool old_gone = (pivot_old_seen || mode_t > PIVOT_RELAX_T) && old_out < PIVOT_OLD_GONE;

        // Đo trực tiếp vận tốc quét ngang thực tế của nhánh mới (mm/s) để handover chuẩn cả khi pin yếu
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
        float p = !old_gone ? P_CLEAR : ((new_out > 0.20f) ? P_APPROACH : P_SEEK);
        vl =  (float)turn_dir * p;
        vr = -(float)turn_dir * p;
        v_cmd = 0.0f;
        if (old_gone && new_in && mode_t > 0.055f) {
            float z = 0.0f;
            if (!measure(s, z)) z = (float)turn_dir * 25.0f;
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
// 5. PHẦN CỨNG: ĐỘNG CƠ, CẢM BIẾN
// ============================================================================
Brain brain;

int ir_white_cal[4] = {ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM, ADC_WHITE_NOM};
int ir_black_cal[4] = {ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM, ADC_BLACK_NOM};
int adc_white_anchor = ADC_WHITE_NOM;   // mốc nền trắng thực tế của sân (tự đo ở setup & tự bám khi đổi sân/tụt pin)
int track_span_est   = 1500;            // độ tương phản ước lượng của sân hiện tại
float floor_cm = 0.0f;   // [V5] mức nâng nền chung hiện tại (0..CM_MAX), tính theo phần của (đen - trắng)

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
// 7. KHỞI TẠO + AUTO-CALIB  [V5.1: viết lại theo darker(), không còn so sánh ADC trực tiếp]
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

    // Bước 1: đợi xe được đặt xuống sàn = có cảm biến ngoài thấy bề mặt phản xạ (không phải đang giơ giữa không trung)
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]);
        int r3 = readIRFiltered(PIN_IR[3]);
        bool r0_ok = (IR_POL < 0) ? (r0 > ADC_RAW_LO_OK + 350) : (r0 < ADC_RAW_HI_OK - 350);
        bool r3_ok = (IR_POL < 0) ? (r3 > ADC_RAW_LO_OK + 350) : (r3 < ADC_RAW_HI_OK - 350);
        if (r0_ok || r3_ok) break;
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

    // Bước 3: đo mẫu nhanh tại vị trí xuất phát. Với mỗi cảm biến lấy mẫu SÁNG nhất (nền) và TỐI nhất (vạch).
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

    int g_white = bright_seen[0];   // mẫu sáng nhất toàn xe  (= nền trắng)
    int g_dark  = dark_seen[0];     // mẫu tối nhất toàn xe   (= vạch đen)
    for (int i = 1; i < 4; i++) {
        g_white = brighterOf(g_white, bright_seen[i]);
        g_dark  = darkerOf(g_dark, dark_seen[i]);
    }

    // Chỉ dùng trắng danh định nếu chưa chạm sàn (bão hòa đáy/trần ADC); còn lại TIN vào nền thực vừa đo được (kể cả sân tối / pin yếu)!
    bool white_saturated = (IR_POL < 0) ? (g_white <= ADC_RAW_LO_OK + 200) : (g_white >= ADC_RAW_HI_OK - 200);
    if (white_saturated) {
        g_white = ADC_WHITE_NOM;
        g_dark  = ADC_BLACK_NOM;
        for (int i = 0; i < 4; i++) dark_seen[i] = ADC_BLACK_NOM;
    }
    adc_white_anchor = g_white;

    // [V5.3] Cần tương phản đủ lớn (>= INIT_CONTRAST_SPAN) mới coi là đang đè vạch đen;
    // tránh hiểu nhầm độ lệch độ nhạy giữa các mắt trên nền trắng (vd 4095 vs 3307) thành vạch đen.
    int init_span = darker(g_dark, g_white);
    bool startup_has_line = (init_span >= INIT_CONTRAST_SPAN && g_dark > ADC_RAW_LO_OK && g_dark < ADC_RAW_HI_OK);
    if (!startup_has_line) {
        g_dark = constrain(g_white + IR_POL * 1600, ADC_RAW_LO_OK + 50, ADC_RAW_HI_OK - 50);
        init_span = abs(g_dark - g_white);
    }
    track_span_est = constrain(init_span, INIT_CONTRAST_SPAN, 3000);

    int white_tol = startup_has_line ? ((init_span / 3 < 850) ? (init_span / 3) : 850) : 950;
    for (int i = 0; i < 4; i++) {
        // Mức trắng riêng: lấy mẫu sáng nhất của chính nó nếu thuộc nền trắng (chịu được sai khác giữa các mắt)
        if (darker(bright_seen[i], g_white) < white_tol) ir_white_cal[i] = bright_seen[i];
        else                                              ir_white_cal[i] = g_white;

        // Mức đen riêng: chỉ lấy mẫu lúc khởi động nếu thật sự có tương phản vạch đen
        int dk = dark_seen[i];
        if (startup_has_line && darker(dk, ir_white_cal[i]) >= INIT_CONTRAST_SPAN && dk > ADC_RAW_LO_OK && dk < ADC_RAW_HI_OK)
            ir_black_cal[i] = dk;
        else
            ir_black_cal[i] = darkerOf(g_dark, ir_white_cal[i] + IR_POL * track_span_est);
    }

    brain.reset();
    floor_cm = 0.0f;
    startMicros    = micros();
    lastStepMicros = startMicros;

#if ENABLE_SERIAL_TELEMETRY
    Serial.printf("[CAL] white_is_high=%d white=[%d,%d,%d,%d] black=[%d,%d,%d,%d]\n", (int)IR_WHITE_IS_HIGH,
                  ir_white_cal[0], ir_white_cal[1], ir_white_cal[2], ir_white_cal[3],
                  ir_black_cal[0], ir_black_cal[1], ir_black_cal[2], ir_black_cal[3]);
#endif
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

    // [V5.2] Tự thích ứng chất liệu sân/vạch & pin yếu, hỗ trợ nền trắng đọc tới 4095 (như log phần cứng thực tế).
    int  frame_span     = frame_max_raw - frame_min_raw;
    int  frame_bright   = (IR_POL > 0) ? frame_min_raw : frame_max_raw;
    bool raw_in_range   = (IR_POL < 0) ? (frame_min_raw > ADC_RAW_LO_OK) : (frame_max_raw < ADC_FRAME_HI_OK);
    bool slow_tick      = ((loopFrameCount & 0x1F) == 0);

    if (raw_in_range && frame_span > MIN_CONTRAST_SPAN) {
        if (darker(frame_bright, adc_white_anchor) < 0) {
            adc_white_anchor = (adc_white_anchor * 3 + frame_bright) / 4;
        } else if (slow_tick && darker(frame_bright, adc_white_anchor) > 150) {
            bool not_in_air = (IR_POL < 0) ? (frame_bright > ADC_RAW_LO_OK + 350) : (frame_bright < ADC_RAW_HI_OK - 350);
            if (not_in_air) adc_white_anchor += IR_POL;   // bám chậm khi nền tối hơn hoặc LED IR tối dần theo pin
        }
    }
    bool valid_contrast = (frame_span > MIN_CONTRAST_SPAN) && (darker(frame_bright, adc_white_anchor) < WHITE_SEEN_MAX_DARK) && raw_in_range;

    float n_raw[4];       // [V5] chuẩn hóa CHƯA kẹp: 0 = đúng mức trắng, 1 = đúng mức đen (có thể < 0 hoặc > 1)
    int   eff_span[4];    // bề rộng hiệu dụng (luôn dương, đơn vị ADC)
    int   span_i_arr[4];
    int   max_ch_span = MIN_CONTRAST_SPAN;
    int   span_sum = 0;
    int   black_learn_thr = (track_span_est > MIN_CONTRAST_SPAN * 2) ? (track_span_est / 2) : MIN_CONTRAST_SPAN;
    for (int i = 0; i < 4; i++) {
        int  raw    = sData.raw_ir[i];
        bool raw_ok = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK) : (raw < ADC_RAW_HI_OK);
        bool floor_bright_ok = (IR_POL < 0) ? (raw > ADC_RAW_LO_OK + 400) : (raw < ADC_RAW_HI_OK - 400);

        // Cho phép mức trắng bám ngay lên nền trắng mới (kể cả vừa từ không khí đặt xuống nền trắng 4095)
        if (raw_ok && floor_bright_ok && darker(raw, ir_white_cal[i]) < 0) {
            ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            if (darker(raw, adc_white_anchor) < 0) adc_white_anchor = (adc_white_anchor * 3 + raw) / 4;
        }

        if (valid_contrast && !brain.was_lifted && (brain.off_track_t < 0.04f || frame_span < INIT_CONTRAST_SPAN)) {
            int dw = darker(raw, ir_white_cal[i]);
            int white_drift_lim = (frame_span < INIT_CONTRAST_SPAN) ? 600 : 250;
            if (slow_tick && dw < white_drift_lim && darker(ir_white_cal[i], adc_white_anchor) < WHITE_DRIFT_MAX_DARK) {
                ir_white_cal[i] += IR_POL;                                            // trôi chậm về phía TỐI (nền ngả tối dần)
            }
            dw = darker(raw, ir_white_cal[i]);
            if (brain.has_seen_line && brain.off_track_t < 0.04f && raw_ok && dw > black_learn_thr && darker(raw, ir_black_cal[i]) > 0) {
                ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;                    // đen bám nhanh về phía tối hơn nữa
            }
        }

        // CHỈ co rất chậm (2 đơn vị / 64 ms) khi xe ĐANG BÁM VẠCH và có tương phản hợp lệ — tuyệt đối không sập span khi nhấc xe hay ở nền trắng!
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

    // Đồng bộ sàn tương phản tối thiểu giữa 4 mắt: tránh mắt rìa S1/S4 bị kẹt ở span quá nhỏ rồi bão hòa 1.00 trên mép vạch
    int shared_span_floor = (max_ch_span * 72) / 100;
    if (shared_span_floor < EFF_SPAN_MIN) shared_span_floor = EFF_SPAN_MIN;
    for (int i = 0; i < 4; i++) {
        int effective_span = (span_i_arr[i] * 86) / 100;
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
    // [V5] Bộ bám nền. Mức trắng hiệu chuẩn chỉ nhích 1 đơn vị mỗi 64 ms và chỉ khi nền lệch < 250 đơn vị ADC, nên khi gặp
    // miếng nền tối hơn / bóng đổ thì cảm biến cứ "thấy vạch mờ" -> trọng tâm sai, ngã tư bị chặn (WING_BASE_MAX).
    // Ở đây: cảm biến tối nhất (n_min) = mức nền hiện tại -> trừ chung cho cả 4 và co giãn lại về [0..1].
    if (brain.was_lifted) {
        floor_cm = 0.0f;                                   // nhấc xe: bỏ học, đặt lại khi đặt xuống
    } else {
        float n_min = min4f(n_raw[0], n_raw[1], n_raw[2], n_raw[3]);
        bool  cm_ok = (frame_span > EFF_SPAN_MIN) && raw_in_range && (brain.off_track_t < 0.04f);
        float cm_tgt = floor_cm;                           // mặc định: ĐÓNG BĂNG (n_min quá cao = ngã tư / vạch rộng)
        if (n_min <= CM_MAX) {
            cm_tgt = max2f(0.0f, n_min - CM_DEAD);
            if (cm_tgt > floor_cm && !cm_ok) cm_tgt = floor_cm;   // chỉ cho phép TĂNG khi tín hiệu hợp lệ
        }
        float cm_tau = (cm_tgt > floor_cm) ? CM_TAU_UP : CM_TAU_DN;
        floor_cm += (cm_tgt - floor_cm) * (dt / (cm_tau + dt));
        floor_cm = clampf(floor_cm, 0.0f, CM_MAX - CM_DEAD);

        // Nền tối kéo dài: chuyển dần vào mức trắng hiệu chuẩn (floor_cm tự giảm theo vì n_min giảm) -> bám được
        // cả khi nền tối thêm nữa. Rời miếng nền thì "nhanh về phía sáng" của ir_white_cal tự trả lại trong ~20 ms.
        if (slow_tick && cm_ok && floor_cm > CM_ABSORB_MIN) {
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
        Serial.printf("[CROSS] #%d act=%+d odo=%.0f center=%.0f sec=%.2f W=%.1f vs=%.2f\n",
                      brain.ev_idx, brain.ev_act, brain.dist_mm, brain.ev_center_d, brain.cross_sec,
                      brain.line_w_est, brain.v_scale);
    }
    if (brain.ev_align_end) {
        brain.ev_align_end = false;
        Serial.printf("[ALIGN->PIVOT] t=%.3fs d=%.1fmm rem=%.1fmm wf=%d W=%.1f vs=%.2f\n",
                      brain.ev_al_t, brain.ev_al_d, brain.ev_al_rem, (int)brain.ev_al_wf,
                      brain.line_w_est, brain.v_scale);
    }
    if (brain.ev_pivot_end) {
        brain.ev_pivot_end = false;
        Serial.printf("[PIVOT->RUN] t=%.3fs z=%.1fmm\n", brain.ev_pv_t, brain.ev_pv_z);
    }
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 120 && Serial.availableForWrite() > 64) {
        lastTelemetryMillis = nowMs;
        Serial.printf("[t=%5.2f] M%d x=%+5.1f vx=%+6.0f u=%+4.0f rk=%.2f cm=%.2f | IR=[%.2f,%.2f,%.2f,%.2f] | RAW=[%4d,%4d,%4d,%4d] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, (int)brain.mode, brain.x, brain.vx, brain.u_cmd, brain.risk_dbg, floor_cm,
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      sData.raw_ir[0], sData.raw_ir[1], sData.raw_ir[2], sData.raw_ir[3], pwm_l, pwm_r);
    }
#endif
}
