#include <Arduino.h>
#include <math.h>

// ============================================================================
// 1. CAU HINH CHIEN THUAT & THONG SO DUONG DUA (CHINH NHANH TAI DAY)
// ============================================================================
#define ENABLE_SERIAL_TELEMETRY 0 // 0 = Tat Serial khi chay thi that | 1 = Bat log debug
#define ENABLE_CROSS_SHORTCUT   0 // 0 = Bam line 100% (Chuan thi dau) | 1 = Re tat nga tu
#define SHORTCUT_CROSS_COUNT    3 // So lan qua nga tu truoc khi re tat (neu bat = 1)

// Thoi gian cho an toan sau khi bat cong tac nguon (3000ms = 3 giay)
// Giup ban dat xe chinh giua line, rut tay ra va doi ap pin on dinh hoan toan!
#define STARTUP_SAFE_DELAY_MS   3000UL

// --- CAU HINH TOC DO (TOI UU CHO CHAY PIN - CHONG SUT AP & BAM CUA CUC KHIT) ---
const float STRAIGHT_MAX_PWM   = 232.0f; // Toc do toi da tren duong thang (220..240 an toan cho pin)
const float NOMINAL_BASE_PWM   = 205.0f; // Toc do nen khi chay on dinh (195..220)
const float CORNER_MIN_PWM     = 100.0f; // Toc do nen toi thieu o cua gat 90 do / cua U (90..120)
const float MAX_PWM_OUT        = 238.0f; // Tran PWM de tranh bao hoa cuon day N20 gay sut ap pin
const float MAX_REVERSE_PWM    = -125.0f;// Luc phanh/dao chieu banh trong vua du bam cua, khong xoay 360

// --- THONG SO BO DIEU KHIEN PD PHI TUYEN 500Hz (dt = 0.002s) ---
// Sai so e nam trong dai [-4.20 .. +4.20]:
const float KP_BASE            = 50.0f;  // He so P tuyen tinh giu tam cuc khit (chinh 44..58)
const float KP_NONLINEAR       = 17.5f;  // He so P bac 2 tang luc be cua: Kp2 * e * |e| (chinh 14..22)
const float KD_BASE            = 2.10f;  // He so D giam chan duong thang (chinh 1.6..2.8)
const float KD_CORNER_BOOST    = 0.75f;  // Tang cuong D khi vao cua de chong vang duoi / chong xoay

// --- THONG SO GIAM TOC THICH UNG THEO DO CONG & DAO HAM ---
const float CURVE_SLOWDOWN_KE  = 35.0f;  // He so ha toc do nen theo sai so |e|
const float CURVE_SLOWDOWN_KD  = 1.20f;  // He so phanh som ngay khi vach vua dich chuyen nhanh

// ============================================================================
// 2. CAU HINH CHAN PHAN CUNG ESP32 & DONG CO N20 (DA BO W_LED_ON)
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

// W_LED_ON da duoc go bo khoi phan cung, chi giu IR_LED_ON cho mat doc line
#define IR_LED_ON 21

const int PIN_IR[4]          = {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN};
const int MIN_N20_PWM        = 34;     // Nguong bu ma sat tinh cua hop so N20
const float RIGHT_MOTOR_TRIM = 1.00f;  // Chinh 0.96..1.04 neu 2 banh khong deu

const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;
const bool SWAP_MOTORS_LR     = false;

// ============================================================================
// 3. HAM TOAN HOC NHANH INLINE
// ============================================================================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) {
    return max2f(max2f(a, b), max2f(c, d));
}
static inline float min4f(float a, float b, float c, float d) {
    return min2f(min2f(a, b), min2f(c, d));
}
static inline float lerpf(float a, float b, float t) {
    return a + (b - a) * clampf(t, 0.0f, 1.0f);
}

struct SensorData {
    float ir[4];      // Tin hieu da chuan hoa vi sai [0.0 = Nen trang, 1.0 = Vach den]
    int raw_ir[4];    // Gia tri ADC 12-bit [0 .. 4095]
    int contrast_span;// Do tuong phan tuc thoi giua mat den nhat va trang nhat
    double elapsed_sec;
};

enum TrackState {
    STATE_WAIT_LINE = 0,     // Cho dat xe len vach den hoac sau khi nhac xe
    STATE_SOFT_LOCK,         // Moi dat xe len vach: khoa xoay gat, chi bam muot
    STATE_NORMAL_RACE,       // Chay dua toc do cao tren vach
    STATE_ARC_RECOVERY,      // Vua khuat vach o cua gat: quet cung ngan (< 250ms) de bat lai vach
    STATE_SAFE_HALT,         // Vang khoi ban dua hoac nhac xe: ngat dong co tuyet doi (chong xoay 360)
    STATE_CROSS_LOCK,
    STATE_SHORTCUT_ALIGN,
    STATE_SHORTCUT_TURN
};

// ============================================================================
// 4. BO CHUAN HOA CAM BIEN VI SAI KHONG GIAN (THAY THE AUTO-CALIB CU)
// ============================================================================
// Nguyen ly:
// - Thay vi phu thuoc vao nguong ADC tuyet doi (rat de sai khi pin sut ap),
//   ta ket hop:
//     (1) Bo nho Nen Trang Co So (base_white[4]) duoc loc chong nhieu cuc vung,
//     (2) So sanh Vi Sai Không Gian giua 4 mat trong cung 1 khung hinh (instant_min)
//         de triet tieu 100% nhieu sut ap nguon pin!
// ============================================================================
struct SmartSensorNormalizer {
    float base_white[4];
    float ref_span;

    void initFromStartupSamples(const int startup_min[4], const int startup_max[4]) {
        int g_min = startup_min[0];
        int g_max = startup_max[0];
        for (int i = 1; i < 4; i++) {
            if (startup_min[i] < g_min) g_min = startup_min[i];
            if (startup_max[i] > g_max) g_max = startup_max[i];
        }

        // Neu luc bat nguon xe dat tren san (g_min hop le < 1600)
        if (g_min > 1600) g_min = 350;

        for (int i = 0; i < 4; i++) {
            // Neu mat cam bien do dang nam tren nen trang luc khoi dong, lay chinh no
            // de bu sai lech giua cac mat LED. Neu mat do dang de len vach den, lay g_min!
            if (startup_min[i] < g_min + 220) {
                base_white[i] = (float)startup_min[i];
            } else {
                base_white[i] = (float)g_min;
            }
        }

        int span = g_max - g_min;
        if (span >= 550 && g_max < 3850) {
            ref_span = (float)span * 0.85f;
        } else {
            ref_span = 1150.0f; // Bien do mac dinh an toan cho cam bien IR 12-bit
        }
        ref_span = clampf(ref_span, 550.0f, 2200.0f);
    }

    void processFrame(SensorData& s) {
        int f_min = s.raw_ir[0];
        int f_max = s.raw_ir[0];
        for (int i = 1; i < 4; i++) {
            if (s.raw_ir[i] < f_min) f_min = s.raw_ir[i];
            if (s.raw_ir[i] > f_max) f_max = s.raw_ir[i];
        }
        s.contrast_span = f_max - f_min;

        // Kiem tra xe co dang nam tren mat san hop le khong (tranh cap nhat khi nhac xe len khong trung)
        bool on_floor_with_line = (f_min < 1650) && (f_max < 3920) && (s.contrast_span > 420);

        if (on_floor_with_line) {
            // Cap nhat do tuong phan tham chieu (EMA cuc cham, khong bao gio bi troi dot ngot)
            float target_span = clampf((float)s.contrast_span * 0.86f, 520.0f, 2300.0f);
            ref_span = 0.992f * ref_span + 0.008f * target_span;

            // Cap nhat nen trang co so chi cho nhung mat dang thuc su nam tren nen trang
            for (int i = 0; i < 4; i++) {
                if (s.raw_ir[i] < f_min + 160 && s.raw_ir[i] > 30) {
                    base_white[i] = 0.985f * base_white[i] + 0.015f * (float)s.raw_ir[i];
                }
            }
        }

        // Tinh gia tri chuan hoa [0.0 .. 1.0] ket hop Nen Co So va Vi Sai Tuc Thoi:
        // Khi pin sut ap lam ca 4 mat cung tang/giam ADC, f_min cung dịch chuyen theo,
        // giup dynamic_white[i] tu dong bam theo dien ap pin tuc thoi!
        for (int i = 0; i < 4; i++) {
            float dynamic_white = base_white[i];
            if (f_min < 1650) {
                // Neu f_min cao hon base_white do ap pin giam, nang nen trang len theo f_min
                float common_mode_floor = (float)f_min + (base_white[i] - min4f(base_white[0], base_white[1], base_white[2], base_white[3]));
                dynamic_white = 0.65f * base_white[i] + 0.35f * common_mode_floor;
            }

            float norm = ((float)s.raw_ir[i] - dynamic_white) / ref_span;
            s.ir[i] = clampf(norm, 0.0f, 1.0f);
        }
    }
};

// ============================================================================
// 5. BO DIEU KHIEN TRUNG TAM CHONG XOAY 360 DO (COMPETITION BRAIN 500Hz)
// ============================================================================
class CobotBrain {
public:
    TrackState state = STATE_WAIT_LINE;

    float pos_error         = 0.0f;
    float prev_pos_error    = 0.0f;
    float filtered_error    = 0.0f;
    float filtered_deriv    = 0.0f;
    float outer_peak_memory = 0.0f;

    int   last_valid_side   = 0;     // 0: Chua xac dinh (Cam xoay!), -1: Trai, +1: Phai
    bool  center_locked     = true;
    bool  was_in_recovery   = false;

    double state_time       = 0.0;   // Thoi gian o trong state hien tai
    double straight_time    = 0.0;   // Thoi gian duy tri duong thang
    double soft_start_timer = 0.0;   // Tang toc em khi moi bat dau chay
    double catch_brake_timer= 0.0;   // Thoi gian dap phanh chong vang khi vua bat lai vach

    // Xu ly Nga tu & Odometry ao
    int    cross_count          = 0;
    int    cross_confirm_frames = 0;
    int    wing_l_hold          = 0;
    int    wing_r_hold          = 0;
    bool   shortcut_taken       = false;

    float  est_v_left        = 0.0f;
    float  est_v_right       = 0.0f;
    double total_dist_m      = 0.0;
    double total_yaw_rad     = 0.0;
    double cross_dist_m      = 0.0;
    double cross_yaw_rad     = 0.0;
    double max_yaw_excursion = 0.0;
    double last_cross_time   = -10.0;
    double run_start_time    = 0.0;

    CobotBrain() { reset(0.0); }

    void reset(double now_sec = 0.0) {
        state               = STATE_WAIT_LINE;
        pos_error           = 0.0f;
        prev_pos_error      = 0.0f;
        filtered_error      = 0.0f;
        filtered_deriv      = 0.0f;
        outer_peak_memory   = 0.0f;

        last_valid_side     = 0; // QUAN TRONG: Khoi tao = 0 de khong bao gio tu xoay khi chua thay vach!
        center_locked       = true;
        was_in_recovery     = false;

        state_time          = 0.0;
        straight_time       = 0.0;
        soft_start_timer    = 0.0;
        catch_brake_timer   = 0.0;

        cross_count          = 0;
        cross_confirm_frames = 0;
        wing_l_hold          = 0;
        wing_r_hold          = 0;
        shortcut_taken       = false;

        est_v_left           = 0.0f;
        est_v_right          = 0.0f;
        total_dist_m         = 0.0;
        total_yaw_rad        = 0.0;
        cross_dist_m         = 0.0;
        cross_yaw_rad        = 0.0;
        max_yaw_excursion    = 0.0;
        last_cross_time      = -10.0;
        run_start_time       = now_sec;
    }

    void changeState(TrackState new_state) {
        if (state != new_state) {
            state = new_state;
            state_time = 0.0;
        }
    }

    void updateVirtualOdometry(int pwm_l, int pwm_r, double dt) {
        if (dt <= 0.0 || dt > 0.1) return;
        const float v_max = 0.85f;
        float cmd_vl = (pwm_l / 255.0f) * v_max;
        float cmd_vr = (pwm_r / 255.0f) * v_max;

        float alpha_m = clampf((float)(dt / (0.015 + dt)), 0.1f, 1.0f);
        est_v_left  += alpha_m * (cmd_vl - est_v_left);
        est_v_right += alpha_m * (cmd_vr - est_v_right);

        double v_center = 0.5 * (double)(est_v_left + est_v_right);
        double omega    = 0.75 * (double)(est_v_right - est_v_left) / 0.120;

        if (v_center > 0.0) total_dist_m += v_center * dt;
        total_yaw_rad += omega * dt;

        double yaw_from_cross = fabs(total_yaw_rad - cross_yaw_rad);
        if (yaw_from_cross > max_yaw_excursion) {
            max_yaw_excursion = yaw_from_cross;
        }
    }

    // ========================================================================
    // NOI SUY VI TRI VACH LIEN TUC [-4.20 .. +4.20]
    // Toa do 4 mat cam bien: s0 = -3.0, s1 = -1.0, s2 = +1.0, s3 = +3.0
    // ========================================================================
    float computeLineError(const float s[4]) {
        float w[4];
        for (int i = 0; i < 4; i++) {
            float clean = clampf((s[i] - 0.08f) / 0.92f, 0.0f, 1.0f);
            w[i] = clean * (0.75f + 0.25f * clean);
        }

        bool both_wings_active = (w[0] > 0.40f && w[3] > 0.40f);
        if (state == STATE_CROSS_LOCK) {
            w[0] = 0.0f;
            w[3] = 0.0f;
        } else if (both_wings_active) {
            if (w[1] > 0.22f || w[2] > 0.22f) {
                w[0] = 0.0f;
                w[3] = 0.0f;
            } else if (last_valid_side != 0) {
                if (last_valid_side > 0) w[0] = 0.0f;
                else w[3] = 0.0f;
            }
        }

        float w_sum = w[0] + w[1] + w[2] + w[3];

        // Neu ca 4 mat deu trang
        if (w_sum < 0.05f) {
            if (state == STATE_CROSS_LOCK || last_valid_side == 0) return 0.0f;
            if (center_locked && fabsf(pos_error) < 0.65f) {
                return pos_error; // Vach manh lot khe giua s1-s2 -> giu nguyen huong thang
            }
            return (last_valid_side > 0) ? 4.20f : -4.20f;
        }

        float num   = (-3.0f * w[0]) + (-1.0f * w[1]) + (1.0f * w[2]) + (3.0f * w[3]);
        float raw_e = num / w_sum;

        // Ngoai suy suon ngoai khi chi con mat ngoai cung (s0 hoac s3) nhin thay vach
        if (!both_wings_active && state == STATE_NORMAL_RACE) {
            if (w[3] > 0.05f && w[2] < 0.12f && w[1] < 0.05f && w[0] < 0.05f) {
                if (w[3] > outer_peak_memory) outer_peak_memory = w[3];
                if (pos_error >= 2.10f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[3]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = 3.00f + 1.20f * decay_ratio;
                    if (extrap_e > raw_e) raw_e = extrap_e;
                }
            } else if (w[0] > 0.05f && w[1] < 0.12f && w[2] < 0.05f && w[3] < 0.05f) {
                if (w[0] > outer_peak_memory) outer_peak_memory = w[0];
                if (pos_error <= -2.10f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[0]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = -3.00f - 1.20f * decay_ratio;
                    if (extrap_e < raw_e) raw_e = extrap_e;
                }
            } else if (w[1] > 0.25f || w[2] > 0.25f) {
                outer_peak_memory = 0.0f;
            }
        }

        if ((w[1] > 0.25f || w[2] > 0.25f) && w[0] < 0.18f && w[3] < 0.18f && fabsf(raw_e) < 0.90f) {
            center_locked = true;
        } else if (fabsf(raw_e) > 1.20f || w[0] > 0.35f || w[3] > 0.35f) {
            center_locked = false;
        }

        // Chi ghi nhan huong lech khi xe DANG CHAY TREN VACH va co tin hieu ro rang
        if (!both_wings_active && (state == STATE_NORMAL_RACE || state == STATE_SOFT_LOCK)) {
            if (w[3] > 0.25f || raw_e > 0.35f) {
                last_valid_side = 1;
            } else if (w[0] > 0.25f || raw_e < -0.35f) {
                last_valid_side = -1;
            }
        }

        return clampf(raw_e, -4.20f, 4.20f);
    }

    // ========================================================================
    // VONG LAP DIEU KHIEN CHINH 500Hz (CO MAY TRANG THAI CHONG XOAY 360)
    // ========================================================================
    void computeControl(const SensorData& s, double dt, int& pwm_left, int& pwm_right) {
        state_time += dt;

        int max_raw = max(max(s.raw_ir[0], s.raw_ir[1]), max(s.raw_ir[2], s.raw_ir[3]));
        int min_raw = min(min(s.raw_ir[0], s.raw_ir[1]), min(s.raw_ir[2], s.raw_ir[3]));

        float max_ir = max4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);

        // Kiem tra xe bi nhac len khong trung (ca 4 mat deu den bao hoa hoac deu lo lung)
        bool lifted_in_air = (min_raw > 2600);
        // Kiem tra co vach den thuc su duoi gam xe (co do tuong phan ro rang giua cac mat)
        bool line_present  = (!lifted_in_air) && (max_ir > 0.28f) && (s.contrast_span > 350 || max_raw > 1100);
        // Kiem tra vach dang nam o 2 mat trung tam (s1, s2)
        bool center_seen   = (!lifted_in_air) && (s.ir[1] > 0.30f || s.ir[2] > 0.30f) && (s.contrast_span > 350);

        // --------------------------------------------------------------------
        // 1. BAO VE KHI NHAC XE LEN KHONG TRUNG -> DUNG NGAY LAP TUC!
        // --------------------------------------------------------------------
        if (lifted_in_air) {
            changeState(STATE_SAFE_HALT);
            last_valid_side = 0;
            pwm_left  = 0;
            pwm_right = 0;
            return;
        }

        // --------------------------------------------------------------------
        // 2. XU LY TRANG THAI CHO VACH (WAIT_LINE) & DUNG AN TOAN (SAFE_HALT)
        // -> Giai quyet triet de loi:
        //    + Bat cong tac len ngoai vach tu xoay 360
        //    + Nhac xe tu ngoai dat vao vach tu xoay 360
        //    + Vang khoi vach xoay 360 vinh vien
        // --------------------------------------------------------------------
        if (state == STATE_WAIT_LINE || state == STATE_SAFE_HALT) {
            pwm_left  = 0;
            pwm_right = 0;

            // Chi khi ban dat xe len vach den (1 trong 2 mat giua s1, s2 nhin thay vach)
            // va giu on dinh tren vach >= 60ms (30 chu ky) de tay ban roi ra hoan toan,
            // xe moi bat dau khoi hanh o che do SOFT_LOCK!
            if (center_seen) {
                if (state_time >= 0.060) {
                    reset(s.elapsed_sec);
                    state            = STATE_SOFT_LOCK;
                    state_time       = 0.0;
                    soft_start_timer = 0.0;
                    float init_e     = computeLineError(s.ir);
                    pos_error        = init_e;
                    prev_pos_error   = init_e;
                    filtered_error   = init_e;
                }
            } else {
                // Chua dat dung vach -> reset bo dem thoi gian, giu dong co dung im 100%
                state_time = 0.0;
            }
            return;
        }

        // --------------------------------------------------------------------
        // 3. TRANG THAI KHOI HANH EM & KHOA TAM VACH (STATE_SOFT_LOCK)
        // Trong 180ms dau tien sau khi dat xe vao vach:
        // - Tang toc muot (Soft-Start) de khong sut ap pin
        // - CAM HOAN TOAN banh xe quay nguoc chieu (khong bao gio xoay 360 khi vua cham vach)
        // --------------------------------------------------------------------
        if (state == STATE_SOFT_LOCK) {
            if (!line_present) {
                // Neu vua nhac tay ra ma xe bi truot khoi vach ngay -> dung im, khong xoay!
                if (state_time > 0.050) {
                    changeState(STATE_WAIT_LINE);
                    pwm_left  = 0;
                    pwm_right = 0;
                    return;
                }
            } else if (state_time >= 0.180) {
                // Da bam vach on dinh -> Mo khoa sang che do dua full toc do!
                changeState(STATE_NORMAL_RACE);
            }
        }

        // --------------------------------------------------------------------
        // 4. XU LY KHI MAT VACH O KHUC CUA (STATE_ARC_RECOVERY) - CHONG XOAY 360!
        // --------------------------------------------------------------------
        if (state == STATE_NORMAL_RACE && !line_present) {
            // Neu vach dang o sat tam (lot khe giua s1-s2), cho phep di thang 14ms
            if (center_locked && fabsf(filtered_error) < 0.65f && state_time < 0.014) {
                // Giu nguyen trang thai
            } else if (last_valid_side != 0) {
                // Chuyen sang che do quet cung ngan de bat lai vach
                changeState(STATE_ARC_RECOVERY);
                was_in_recovery = true;
            } else {
                changeState(STATE_SAFE_HALT);
                pwm_left  = 0;
                pwm_right = 0;
                return;
            }
        }

        if (state == STATE_ARC_RECOVERY) {
            // A. Neu bat lai duoc vach den -> Kich hoat PHANH D-CATCH de khoa vach ngay,
            //    KHONG cho dau xe lao vuot qua vach sang ben kia!
            if (line_present) {
                changeState(STATE_NORMAL_RACE);
                catch_brake_timer = 0.045; // Kich hoat 45ms giam chan manh khi vua bat lai vach
                float new_e    = computeLineError(s.ir);
                pos_error      = new_e;
                prev_pos_error = new_e;
                filtered_error = new_e;
                // Tao xung dao ham nguoc chieu quay de dap phanh goc ngay lap tuc!
                filtered_deriv = (last_valid_side > 0) ? -85.0f : 85.0f;
            }
            // B. GIOI HAN THOI GIAN TIM VACH TOI DA 260ms (~ Goc quay 110 do):
            //    Neu sau 260ms van khong thay vach -> Xe da vang khoi ban dua!
            //    NGAT DONG CO DUNG NGAY LAP TUC (STATE_SAFE_HALT), TUYET DOI KHONG XOAY 360!
            else if (state_time > 0.260) {
                changeState(STATE_SAFE_HALT);
                pwm_left  = 0;
                pwm_right = 0;
                return;
            }
            // C. Trong 260ms dau khi chom mat vach o cua gat:
            //    Quay cung muot voi toc do vua phai de mat cam bien kip bat vach, khong quay dien loan!
            else {
                float rec_progress = clampf((float)(state_time / 0.180), 0.0f, 1.0f);
                float outer_pwm    = lerpf(185.0f, 160.0f, rec_progress); // Giam dan banh ngoai de thu hep ban kinh
                float inner_pwm    = lerpf(-65.0f, -95.0f, rec_progress); // Phanh nhe banh trong (khong quay -165 gay sut ap!)

                if (last_valid_side > 0) {
                    pwm_left  = (int)outer_pwm;
                    pwm_right = (int)inner_pwm;
                } else {
                    pwm_left  = (int)inner_pwm;
                    pwm_right = (int)outer_pwm;
                }
                updateVirtualOdometry(pwm_left, pwm_right, dt);
                return;
            }
        }

        // --------------------------------------------------------------------
        // 5. XU LY NGA TU & RE TAT (Neu ENABLE_CROSS_SHORTCUT == 1)
        // --------------------------------------------------------------------
        if (ENABLE_CROSS_SHORTCUT && (state == STATE_NORMAL_RACE || state == STATE_CROSS_LOCK ||
                                      state == STATE_SHORTCUT_ALIGN || state == STATE_SHORTCUT_TURN)) {
            double dist_since_cross = total_dist_m - cross_dist_m;
            double run_elapsed      = s.elapsed_sec - run_start_time;

            if (state == STATE_NORMAL_RACE && !shortcut_taken && cross_count < SHORTCUT_CROSS_COUNT) {
                if (s.ir[0] > 0.50f) wing_l_hold = 4;
                else if (wing_l_hold > 0) wing_l_hold--;

                if (s.ir[3] > 0.50f) wing_r_hold = 4;
                else if (wing_r_hold > 0) wing_r_hold--;

                bool center_on_line  = (s.ir[1] > 0.40f || s.ir[2] > 0.40f);
                bool live_wing_seen  = (s.ir[0] > 0.42f && s.ir[3] > 0.42f);
                bool all_wings_black = (wing_l_hold > 0 && wing_r_hold > 0 && live_wing_seen &&
                                        center_on_line && fabsf(filtered_error) < 1.15f);

                if (all_wings_black) cross_confirm_frames++;
                else cross_confirm_frames = 0;

                double req_yaw = (SHORTCUT_CROSS_COUNT >= 3 && cross_count >= 1) ? 0.65 : 0.10;
                bool topo_ok   = (cross_count == 0)
                                 ? (run_elapsed > 0.25 && total_dist_m > 0.12)
                                 : ((s.elapsed_sec - last_cross_time) > 0.50 &&
                                    dist_since_cross > 0.22 &&
                                    max_yaw_excursion > req_yaw);

                if (cross_confirm_frames >= 2 && topo_ok) {
                    cross_confirm_frames = 0;
                    wing_l_hold          = 0;
                    wing_r_hold          = 0;
                    cross_count++;
                    last_cross_time   = s.elapsed_sec;
                    cross_dist_m      = total_dist_m;
                    cross_yaw_rad     = total_yaw_rad;
                    max_yaw_excursion = 0.0;

                    if (cross_count < SHORTCUT_CROSS_COUNT) {
                        changeState(STATE_CROSS_LOCK);
                    } else {
                        changeState(STATE_SHORTCUT_ALIGN);
                    }
                }
            }

            if (state == STATE_CROSS_LOCK) {
                bool cleared_wings = (state_time > 0.035 && s.ir[0] < 0.20f && s.ir[3] < 0.20f);
                if (cleared_wings || state_time > 0.070) {
                    changeState(STATE_NORMAL_RACE);
                }
            } else if (state == STATE_SHORTCUT_ALIGN) {
                if (state_time < 0.050) {
                    pwm_left  = 185;
                    pwm_right = 185;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                } else {
                    changeState(STATE_SHORTCUT_TURN);
                }
            }

            if (state == STATE_SHORTCUT_TURN) {
                if (state_time < 0.12) {
                    pwm_left  =  195;
                    pwm_right = -110;
                    last_valid_side = 1;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }
                if (s.ir[1] > 0.42f || s.ir[2] > 0.42f || state_time > 0.32) {
                    shortcut_taken = true;
                    changeState(STATE_NORMAL_RACE);
                    float init_err  = computeLineError(s.ir);
                    pos_error       = init_err;
                    prev_pos_error  = init_err;
                    filtered_error  = init_err;
                    filtered_deriv  = 0.0f;
                    last_valid_side = 1;
                } else {
                    pwm_left  =  175;
                    pwm_right = -95;
                    last_valid_side = 1;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }
            }
        }

        // --------------------------------------------------------------------
        // 6. TINH TOAN DIEU KHIEN PD PHI TUYEN & BAM LINE SIEU MUOT
        // --------------------------------------------------------------------
        prev_pos_error = pos_error;
        pos_error      = computeLineError(s.ir);

        filtered_error = 0.20f * filtered_error + 0.80f * pos_error;

        float raw_deriv = 0.0f;
        if (dt > 0.0002) {
            raw_deriv = (pos_error - prev_pos_error) / (float)dt;
            raw_deriv = clampf(raw_deriv, -200.0f, 200.0f);
        }

        float alpha_d  = clampf((float)(dt / (0.0060 + dt)), 0.18f, 0.55f);
        filtered_deriv = (1.0f - alpha_d) * filtered_deriv + alpha_d * raw_deriv;

        float abs_e = fabsf(filtered_error);
        float abs_d = fabsf(filtered_deriv);

        // Tich luy thoi gian duong thang
        if (abs_e < 0.70f && state == STATE_NORMAL_RACE) {
            straight_time = min2f((float)(straight_time + dt), 0.25f);
        } else if (abs_e > 1.25f) {
            straight_time = 0.0;
        } else {
            straight_time = max2f(0.0f, (float)(straight_time - 2.0 * dt));
        }

        float turbo_ratio = clampf((float)(straight_time / 0.14), 0.0f, 1.0f);
        float peak_base   = lerpf(NOMINAL_BASE_PWM, STRAIGHT_MAX_PWM, turbo_ratio);

        float err_penalty   = CURVE_SLOWDOWN_KE * powf(max2f(0.0f, abs_e - 0.12f), 1.30f);
        float deriv_penalty = CURVE_SLOWDOWN_KD * max2f(0.0f, abs_d - 10.0f);
        float target_base   = clampf(peak_base - err_penalty - deriv_penalty, CORNER_MIN_PWM, STRAIGHT_MAX_PWM);

        // Neu vua thoat khoi ARC_RECOVERY (vua bat lai vach), ha toc do nen trong 45ms de xe on dinh
        if (catch_brake_timer > 0.0) {
            catch_brake_timer = max2f(0.0f, (float)(catch_brake_timer - dt));
            target_base = min2f(target_base, 125.0f);
        }

        // Neu dang o pha khoi hanh mem (SOFT_LOCK 180ms dau tien), tang toc tu tu tu 110 len target_base
        soft_start_timer = min2f(0.30f, (float)(soft_start_timer + dt));
        if (soft_start_timer < 0.20f) {
            float ramp = clampf((float)(soft_start_timer / 0.20f), 0.0f, 1.0f);
            target_base = min2f(target_base, lerpf(115.0f, NOMINAL_BASE_PWM, ramp));
        }

        // Tinh luc lai PD phi tuyen
        float center_softener = (abs_e < 0.35f) ? (0.85f + 0.42f * abs_e) : 1.0f;
        float p_term = (KP_BASE * center_softener) * filtered_error
                     + KP_NONLINEAR * filtered_error * abs_e;

        float effective_kd = KD_BASE + KD_CORNER_BOOST * clampf(abs_e / 2.40f, 0.0f, 1.2f);
        if (catch_brake_timer > 0.0) effective_kd *= 1.45f; // Tang giam chan D khi vua bat lai vach
        float d_term = effective_kd * filtered_deriv;

        float turn_effort = p_term + d_term;
        if (state == STATE_CROSS_LOCK) {
            target_base = NOMINAL_BASE_PWM;
            turn_effort = clampf(turn_effort, -40.0f, 40.0f);
        } else if (state == STATE_SOFT_LOCK) {
            turn_effort = clampf(turn_effort, -110.0f, 110.0f);
        } else {
            turn_effort = clampf(turn_effort, -340.0f, 340.0f);
        }

        // Phoi luc vi sai bat doi xung (Asymmetric Torque Vectoring)
        float v_left  = target_base + turn_effort;
        float v_right = target_base - turn_effort;

        if (v_left > MAX_PWM_OUT) {
            float excess = v_left - MAX_PWM_OUT;
            v_left   = MAX_PWM_OUT;
            v_right -= 0.80f * excess;
        } else if (v_right > MAX_PWM_OUT) {
            float excess = v_right - MAX_PWM_OUT;
            v_right  = MAX_PWM_OUT;
            v_left  -= 0.80f * excess;
        }

        // QUY TAC CHONG XOAY:
        // - Khi o STATE_SOFT_LOCK hoac sai so |e| < 0.85: KHONG cho phep banh trong quay lui (min_allowed >= 0)
        // - Chi khi vao cua gat thuc su (|e| > 0.85 o STATE_NORMAL_RACE) moi cho phanh lui muot toi MAX_REVERSE_PWM
        float min_allowed = 0.0f;
        if (state == STATE_NORMAL_RACE && abs_e > 0.85f) {
            float rev_allow = clampf((abs_e - 0.85f) / 2.15f, 0.0f, 1.0f);
            min_allowed = lerpf(0.0f, MAX_REVERSE_PWM, rev_allow);
        }

        v_left  = clampf(v_left,  min_allowed, MAX_PWM_OUT);
        v_right = clampf(v_right, min_allowed, MAX_PWM_OUT);

        pwm_left  = (int)v_left;
        pwm_right = (int)v_right;

        updateVirtualOdometry(pwm_left, pwm_right, dt);
    }
};

// ============================================================================
// 6. DIEU KHIEN PHAN CUNG DONG CO & CHONG SUT AP PIN (ANTI-BROWNOUT)
// ============================================================================
CobotBrain brain;
SmartSensorNormalizer sensorNorm;

unsigned long startMicros         = 0;
unsigned long lastStepMicros      = 0;
unsigned long lastTelemetryMillis = 0;
int prev_hw_pwm_l = 0, prev_hw_pwm_r = 0;

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
    if (mag >= 245) return (pwm > 0) ? 245 : -245;
    int out = MIN_N20_PWM + ((mag * (245 - MIN_N20_PWM)) / 245);
    out = constrain(out, 0, 245);
    return (pwm > 0) ? out : -out;
}

// ============================================================================
// BO GIOI HAN DONG KHOI DONG & CHONG DAO CHIEU DOT NGOT (ANTI-BROWNOUT SLEW)
// - Ngan chan tuyet doi hien tuong dong co N20 dao chieu tu +230 sang -130
//   trong 1 chu ky lam sut ap pin va reset ESP32!
// - Neu dong co dang quay tien (> 0) ma lenh yeu cau quay lui (< 0),
//   bat buoc di qua diem 0 (Short-Brake Discharge) de triet tieu dong xung nguoc.
// ============================================================================
static inline int slewLimitPWM(int target, int prev) {
    if (target == 0 && abs(prev) < 45) return 0;

    // Chong dao chieu truc tiep qua 0 trong 1 chu ky (Nguyen nhan so 1 gay sut ap pin!)
    if ((prev > 0 && target < 0) || (prev < 0 && target > 0)) {
        if (abs(prev) > 55) {
            return (prev > 0) ? (prev - 55) : (prev + 55);
        }
        return 0; // Dung 1 chu ky (2ms) tai 0V de xa dong cam ung truoc khi dao chieu
    }

    int diff = target - prev;
    bool is_accelerating_mag = (abs(target) > abs(prev));

    if (is_accelerating_mag) {
        // Gioi han toc do tang PWM: 28 don vi / 2ms (~ 35ms de len max) -> Khong bao gio sut ap pin!
        if (diff > 28)  return prev + 28;
        if (diff < -28) return prev - 28;
        return target;
    } else {
        // Giam toc ve 0: nhanh va muot (85 don vi / 2ms)
        if (diff > 85)  return prev + 85;
        if (diff < -85) return prev - 85;
        return target;
    }
}

void setMotorHardware(int pwm_l, int pwm_r) {
    if (SWAP_MOTORS_LR) {
        int tmp = pwm_l; pwm_l = pwm_r; pwm_r = tmp;
    }
    if (INVERT_LEFT_MOTOR)  pwm_l = -pwm_l;
    if (INVERT_RIGHT_MOTOR) pwm_r = -pwm_r;

    int target_l = applyN20Deadzone(pwm_l);
    int target_r = applyN20Deadzone((int)(pwm_r * RIGHT_MOTOR_TRIM));

    int out_l = slewLimitPWM(target_l, prev_hw_pwm_l);
    int out_r = slewLimitPWM(target_r, prev_hw_pwm_r);

    // Gioi han tong cong suat tuc thoi (Power Budget Cap) khi chay pin:
    // Neu 1 banh dang tien manh va 1 banh dang phanh nguoc, giu tong bien do <= 345
    int total_current_load = abs(out_l) + abs(out_r);
    if ((out_l * out_r < 0) && total_current_load > 345) {
        float scale = 345.0f / (float)total_current_load;
        out_l = (int)(out_l * scale);
        out_r = (int)(out_r * scale);
    }

    prev_hw_pwm_l = out_l;
    prev_hw_pwm_r = out_r;

    if (out_l >= 0) {
        writeLEDC(CH_L_A, out_l);
        writeLEDC(CH_L_B, 0);
    } else {
        writeLEDC(CH_L_A, 0);
        writeLEDC(CH_L_B, -out_l);
    }

    if (out_r >= 0) {
        writeLEDC(CH_R_A, out_r);
        writeLEDC(CH_R_B, 0);
    } else {
        writeLEDC(CH_R_A, 0);
        writeLEDC(CH_R_B, -out_r);
    }
}

// Doc ADC kem bo loc Trung vi 5 mau (Median-of-5) + Xa dien tich MUX ADC ESP32
static inline int readIRFiltered(int pin) {
    (void)analogRead(pin); // Xa dien tich kenh MUX truoc do
    int v[5];
    for (int i = 0; i < 5; i++) {
        v[i] = analogRead(pin);
    }
    // Sap xep nhanh 5 phan tu lay trung vi (loai bo 100% gai nhieu xung PWM dong co)
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (v[j] < v[i]) {
                int tmp = v[i]; v[i] = v[j]; v[j] = tmp;
            }
        }
    }
    return v[2];
}

// ============================================================================
// 7. KHOI TAO HE THONG VOI 3 GIAY AN TOAN TUYET DOI
// ============================================================================
void setup() {
    // 1. KHOA CUNG DONG CO = 0 NGAY DONG DAU TIEN KHI VUA CAP NGUON PIN!
    setupMotorsHardware();
    writeLEDC(CH_L_A, 0);
    writeLEDC(CH_L_B, 0);
    writeLEDC(CH_R_A, 0);
    writeLEDC(CH_R_B, 0);

    // 2. Bat LED hong ngoai doc line (da bo hoan toan W_LED_ON)
    pinMode(IR_LED_ON, OUTPUT);
    digitalWrite(IR_LED_ON, HIGH);

#if ENABLE_SERIAL_TELEMETRY
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
#endif

#if defined(ESP32)
    analogReadResolution(12);
#endif

    for (int i = 0; i < 4; i++) {
        pinMode(PIN_IR[i], INPUT);
    }

    // 3. THOI GIAN CHO AN TOAN 3.0 GIAY (STARTUP_SAFE_DELAY_MS):
    // - Trong 2.2 giay dau: Cho ban dat xe ngay ngan vao vach, rut tay ra xa, va de ap pin on dinh.
    // - Trong 0.8 giay cuoi: Do mau nen trang / vach den tinh khi xe da nam im tren san.
    unsigned long bootStart = millis();
    int min_seen[4] = {4095, 4095, 4095, 4095};
    int max_seen[4] = {0, 0, 0, 0};

    while (millis() - bootStart < STARTUP_SAFE_DELAY_MS) {
        // Luon giu dong co = 0 trong suot 3 giay khoi dong
        writeLEDC(CH_L_A, 0);
        writeLEDC(CH_L_B, 0);
        writeLEDC(CH_R_A, 0);
        writeLEDC(CH_R_B, 0);

        // Chi thu thap mau calib trong 800ms cuoi cung (khi tay nguoi dung da buong ra hoan toan)
        if (millis() - bootStart >= (STARTUP_SAFE_DELAY_MS - 800UL)) {
            for (int i = 0; i < 4; i++) {
                int val = readIRFiltered(PIN_IR[i]);
                if (val < min_seen[i]) min_seen[i] = val;
                if (val > max_seen[i]) max_seen[i] = val;
            }
        }
        delay(5);
    }

    // 4. Khoi tao Bo Chuan Hoa Vi Sai Khong Gian
    sensorNorm.initFromStartupSamples(min_seen, max_seen);

    brain.reset(0.0);
    startMicros    = micros();
    lastStepMicros = startMicros;
}

// ============================================================================
// 8. VONG LAP THOI GIAN THUC 500Hz (CHU KY CHUAN 2000 MICROSECONDS)
// ============================================================================
void loop() {
    unsigned long nowMicros = micros();
    unsigned long diffMicros = nowMicros - lastStepMicros;
    if (diffMicros < 2000) return; // Khoa tan so dieu khien chuan 500Hz (dt = 2.0ms)
    lastStepMicros = nowMicros;

    double dt = diffMicros * 1e-6;
    if (dt > 0.025) dt = 0.002;

    SensorData sData;
    sData.elapsed_sec = (nowMicros - startMicros) * 1e-6;

    // 1. Doc 4 mat cam bien IR voi bo loc Median-of-5 chong nhieu xung dong co
    for (int i = 0; i < 4; i++) {
        sData.raw_ir[i] = readIRFiltered(PIN_IR[i]);
    }

    // 2. Chuan hoa tin hieu bang thuat toan Vi Sai Khong Gian (khang sut ap pin)
    sensorNorm.processFrame(sData);

    // 3. Tinh toan luat dieu khien & xuat xung PWM an toan
    int pwm_l = 0, pwm_r = 0;
    brain.computeControl(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);

#if ENABLE_SERIAL_TELEMETRY
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 120 && Serial.availableForWrite() > 64) {
        lastTelemetryMillis = nowMs;
        Serial.printf("[t=%5.2fs] St=%d Err=%+5.2f dE=%+6.1f | Span=%4d | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, (int)brain.state, brain.filtered_error, brain.filtered_deriv,
                      sData.contrast_span,
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      pwm_l, pwm_r);
    }
#endif
}
