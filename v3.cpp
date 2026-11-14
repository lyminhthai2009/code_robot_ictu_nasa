#include <Arduino.h>
#include <math.h>

// ============================================================================
// 1. CAU HINH CHIEN THUAT & THONG SO DUONG DUA (CHINH NHANH TAI DAY)
// ============================================================================
#define ENABLE_SERIAL_TELEMETRY 0 // 0 = Tat Serial khi chay thi that | 1 = Bat log debug
#define ENABLE_CROSS_SHORTCUT   1 // 1 = Kich hoat: Gap nga tu -> Dung lai -> Tinh toan 2s -> Xoay
#define SHORTCUT_CROSS_COUNT    1 // 1 = Cu gap nga tu dau tien la dung lai ngay
#define REPEAT_EVERY_CROSS      1 // 1 = Lap lai o MOI nga tu gap phai | 0 = Chi dung & xoay 1 lan
#define CROSS_STOP_WAIT_SEC     2.00f // Thoi gian dung xe tinh toan tai nga tu truoc khi xoay (2.0 giay)
#define CROSS_TURN_DIRECTION    1 // +1 = Xoay PHAI, -1 = Xoay TRAI, 0 = Tu dong tinh huong xoay trong 2s dung

// --- CAU HINH TOC DO (DAT TOC DO CAO NHUNG VAO CUA CUC DINH) ---
// Luu y: Voi dong co N20 toc do cao, STRAIGHT_PWM = 245..255 tren duong thang,
// nhung khi chom vao cua phai ha muot ve CORNER_MIN_PWM de banh xe khong truot.
const float STRAIGHT_MAX_PWM   = 245.0f; // Toc do toi da tren duong thang (230..255)
const float NOMINAL_BASE_PWM   = 205.0f; // Toc do nen khi chay on dinh (190..225)
const float CORNER_MIN_PWM     = 105.0f; // Toc do nen toi thieu o cua gat 90 do / cua U (90..125)
const float MAX_PWM_OUT        = 255.0f; // Tran PWM phan cung
const float MAX_REVERSE_PWM    = -165.0f;// Luc phanh/dao chieu toi da banh trong khi cua gat (-130..-190)

// --- THONG SO BO DIEU KHIEN PD PHI TUYEN 500Hz (dt = 0.002s) ---
// Sai so e nam trong dai [-4.50 .. +4.50]:
//   |e| = 0.0 .. 1.0 : Vach nam giua 2 mat trung tam (s1, s2)
//   |e| = 1.0 .. 3.0 : Vach chuyen sang mat ngoai (s0, s3)
//   |e| = 3.0 .. 4.5 : Vach o mep ngoai cung / bat dau khuatvach
const float KP_BASE            = 48.0f;  // He so P tuyen tinh giu tam (chinh 42..58)
const float KP_NONLINEAR       = 18.5f;  // He so P bac 2 tang luc be cua: Kp2 * e * |e| (chinh 14..24)
const float KD_BASE            = 1.85f;  // He so D giam chan duong thang (tinh tren de/dt thuc, chinh 1.4..2.5)
const float KD_CORNER_BOOST    = 0.55f;  // Tang cuong D khi vao cua de chong vang duoi (chinh 0.3..0.9)

// --- THONG SO GIAM TOC THICH UNG THEO DO CONG & DAO HAM ---
const float CURVE_SLOWDOWN_KE  = 34.0f;  // He so ha toc do nen theo sai so |e| (chinh 28..42)
const float CURVE_SLOWDOWN_KD  = 1.15f;  // He so phanh som ngay khi vach vua dich chuyen nhanh (chinh 0.8..1.6)

// ============================================================================
// 2. CAU HINH CHAN PHAN CUNG ESP32 & DONG CO N20 (GIU NGUYEN PHAN CUNG CUA BAN)
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
const int MIN_N20_PWM        = 36;    // Nguong bu ma sat tinh cua hop so N20
const float RIGHT_MOTOR_TRIM = 1.00f; // Chinh 0.96..1.04 neu 2 banh khong deu

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
    float ir[4];      // Gia tri chuan hoa [0.0 = Trang tinh, 1.0 = Den tuyet doi]
    int raw_ir[4];    // Gia tri ADC 12-bit [0 .. 4095]
    double elapsed_sec;
};

enum TrackState {
    STATE_NORMAL_RACE = 0,
    STATE_CROSS_LOCK,
    STATE_SHORTCUT_ALIGN, // DUNG XE & TINH TOAN TRONG 2.0 GIAY TAI NGA TU
    STATE_SHORTCUT_TURN   // XOAY XE TIM VACH MOI SAU KHI DUNG DU 2.0 GIAY
};

// ============================================================================
// 4. BO DIEU KHIEN TRUNG TAM SIEU TOC (COMPETITION BRAIN 500Hz)
// ============================================================================
class CobotBrain {
public:
    TrackState state = STATE_NORMAL_RACE;

    // Trang thai bo quan sat vi tri vach & PD
    float pos_error          = 0.0f;
    float prev_pos_error     = 0.0f;
    float filtered_error     = 0.0f;
    float filtered_deriv     = 0.0f;
    float outer_peak_memory  = 0.0f; // Nho dinh tin hieu mat ngoai de ngoai suy suon ngoai

    int   last_valid_side    = 1;    // -1: Ben trai (s0,s1), +1: Ben phai (s2,s3)
    bool  center_locked      = true; // Co bao vach dang nam o vung trung tam
    bool  was_all_white      = false;
    bool  was_lifted         = false;
    bool  has_seen_line      = false;

    double lost_line_time    = 0.0;
    double off_track_time    = 0.0;
    double straight_time     = 0.0;  // Thoi gian duy tri duong thang de kich hoat Turbo muot

    // Trang thai xu ly nga tu & Bo nho dem lich su sai so (Chong nham cua & chong tu khoa khi vao xien)
    int    cross_count          = 0;
    int    cross_confirm_frames = 0;
    int    wing_l_hold          = 0;
    int    wing_r_hold          = 0;
    float  wing_l_peak          = 0.0f;
    float  wing_r_peak          = 0.0f;
    bool   shortcut_taken       = false;

    // Ring buffer luu |filtered_error| 8 chu ky truoc (16ms truoc khi cham mep nga tu)
    float  err_history[8];
    int    err_hist_idx         = 0;

    float  calc_lr_bias         = 0.0f; // Tich luy phan tich huong trong 2s dung xe
    int    calc_samples         = 0;
    int    decided_turn_dir     = 1;    // Huong xoay sau khi tinh toan (+1: Phai, -1: Trai)
    bool   turn_cleared_line    = false;// Co xac nhan da xoay thoat khoi vach cu

    float  est_v_left        = 0.0f;
    float  est_v_right       = 0.0f;
    double total_dist_m      = 0.0;
    double total_yaw_rad     = 0.0;
    double cross_dist_m      = 0.0;
    double cross_yaw_rad     = 0.0;
    double max_yaw_excursion = 0.0;
    double last_cross_time   = -10.0;

    double state_enter_time  = 0.0;
    double run_start_time    = 0.0;

    CobotBrain() { reset(0.0); }

    void reset(double now_sec = 0.0) {
        state                = STATE_NORMAL_RACE;
        pos_error            = 0.0f;
        prev_pos_error       = 0.0f;
        filtered_error       = 0.0f;
        filtered_deriv       = 0.0f;
        outer_peak_memory    = 0.0f;

        last_valid_side      = 1;
        center_locked        = true;
        was_all_white        = false;
        was_lifted           = false;
        has_seen_line        = false;

        lost_line_time       = 0.0;
        off_track_time       = 0.0;
        straight_time        = 0.0;

        cross_count          = 0;
        cross_confirm_frames = 0;
        wing_l_hold          = 0;
        wing_r_hold          = 0;
        wing_l_peak          = 0.0f;
        wing_r_peak          = 0.0f;
        shortcut_taken       = false;

        for (int i = 0; i < 8; i++) err_history[i] = 0.0f;
        err_hist_idx         = 0;

        calc_lr_bias         = 0.0f;
        calc_samples         = 0;
        decided_turn_dir     = (CROSS_TURN_DIRECTION != 0) ? CROSS_TURN_DIRECTION : 1;
        turn_cleared_line    = false;

        est_v_left           = 0.0f;
        est_v_right          = 0.0f;
        total_dist_m         = 0.0;
        total_yaw_rad        = 0.0;
        cross_dist_m         = 0.0;
        cross_yaw_rad        = 0.0;
        max_yaw_excursion    = 0.0;
        last_cross_time      = -10.0;

        state_enter_time     = now_sec;
        run_start_time       = now_sec;
    }

    void enterState(TrackState new_state, double now_sec) {
        state = new_state;
        state_enter_time = now_sec;
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
    // THUAT TOAN NOI SUY VI TRI VACH CHINH XAC CAO & KHOA SUON NGOAI [-4.50 .. +4.50]
    // Toa do 4 mat cam bien: s0 = -3.0, s1 = -1.0, s2 = +1.0, s3 = +3.0
    // ========================================================================
    float computeLineError(const float s[4]) {
        // 1. Tach nen trang mem mai voi nguong thap (0.06) de KHONG BAO GIO mat vach
        //    khi vach den lot vao khe giua 2 mat cam bien!
        float w[4];
        for (int i = 0; i < 4; i++) {
            float clean = clampf((s[i] - 0.06f) / 0.94f, 0.0f, 1.0f);
            // Nhan nhe mu 1.15 de tang do tach biet giua mat chinh va nhieu anh sang tan xa
            w[i] = clean * (0.75f + 0.25f * clean);
        }

        // 2. Chong nhieu khi di qua nga tu / vach ngang (ca 2 mat ngoai s0 va s3 cung den)
        bool both_wings_active = (w[0] > 0.38f && w[3] > 0.38f);
        if (state == STATE_CROSS_LOCK) {
            w[0] = 0.0f;
            w[3] = 0.0f;
        } else if (both_wings_active) {
            if (w[1] > 0.20f || w[2] > 0.20f) {
                // Dang cat ngang nga tu nhung van co tam -> khoa 2 canh ngoai de lao thang
                w[0] = 0.0f;
                w[3] = 0.0f;
            } else {
                // Neu dang o cua gat gap nhanh re -> chi giu lai mat cung phia voi huong cua
                if (last_valid_side >= 0) w[0] = 0.0f;
                else w[3] = 0.0f;
            }
        }

        float w_sum = w[0] + w[1] + w[2] + w[3];

        // 3. Xu ly khi ca 4 mat deu trang (w_sum < 0.05f)
        if (w_sum < 0.05f) {
            if (state == STATE_CROSS_LOCK) return 0.0f;

            // Neu truoc do vach dang o sat tam (|pos_error| < 0.65) va moi khuat < 12ms
            // (do vach qua manh lot khe s1-s2), giu nguyen sai so cu thay vi be lai!
            if (center_locked && lost_line_time < 0.012 && fabsf(pos_error) < 0.65f) {
                return pos_error;
            }

            // Con neu vach da lech ra mep hoac dang vao cua -> KHOA CUNG SAI SO CUC DAI
            // de be cua ngay lap tuc, khong chan chu!
            return (last_valid_side >= 0) ? 4.50f : -4.50f;
        }

        // 4. Tinh trong tam (Centroid) CHUAN TOAN HOC (khong ep denom = 0.20 nhu code cu)
        float num   = (-3.0f * w[0]) + (-1.0f * w[1]) + (1.0f * w[2]) + (3.0f * w[3]);
        float raw_e = num / w_sum;

        // 5. THUAT TOAN NGOAI SUY SUON NGOAI (MONOTONIC OUTER-EDGE EXTRAPOLATION):
        // Khi vach da di qua tam cua mat ngoai cung (s3 hoac s0) va dang truot ra ngoai mep,
        // chi con duy nhat mat ngoai cung nhin thay vach (w[3] > 0, cac mat khac ~ 0).
        // Luc nay raw_e = +3.0. Neu vach tiep tuc truot ra xa hon, w[3] se giam tu 0.70 xuong 0.05.
        // Ta bien su suy giam cua w[3] thanh sai so tang dan tu +3.00 len +4.45!
        if (!both_wings_active) {
            // Phia ben PHAI (Mat s3)
            if (w[3] > 0.04f && w[2] < 0.12f && w[1] < 0.05f && w[0] < 0.05f) {
                if (w[3] > outer_peak_memory) outer_peak_memory = w[3];
                if (pos_error >= 2.20f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[3]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = 3.00f + 1.45f * decay_ratio;
                    if (extrap_e > raw_e) raw_e = extrap_e;
                }
            }
            // Phia ben TRAI (Mat s0)
            else if (w[0] > 0.04f && w[1] < 0.12f && w[2] < 0.05f && w[3] < 0.05f) {
                if (w[0] > outer_peak_memory) outer_peak_memory = w[0];
                if (pos_error <= -2.20f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[0]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = -3.00f - 1.45f * decay_ratio;
                    if (extrap_e < raw_e) raw_e = extrap_e;
                }
            } else if (w[1] > 0.25f || w[2] > 0.25f) {
                // Reset bo nho dinh khi vach da quay ve 2 mat giua
                outer_peak_memory = 0.0f;
            }
        }

        // 6. Cap nhat co vung trung tam & ghi nho ben cuoi cung thay vach
        if ((w[1] > 0.22f || w[2] > 0.22f) && w[0] < 0.18f && w[3] < 0.18f && fabsf(raw_e) < 0.90f) {
            center_locked = true;
        } else if (fabsf(raw_e) > 1.15f || w[0] > 0.30f || w[3] > 0.30f) {
            center_locked = false;
        }

        // Luu huong vach cuc nhay de neu mat vach o ms tiep theo la biet phai quay ben nao ngay
        if (!both_wings_active) {
            if (w[3] > 0.15f || raw_e > 0.18f) {
                last_valid_side = 1;
            } else if (w[0] > 0.15f || raw_e < -0.18f) {
                last_valid_side = -1;
            }
        }

        return clampf(raw_e, -4.50f, 4.50f);
    }

    // ========================================================================
    // VONG LAP TINH TOAN DIEU KHIEN CHINH (500Hz)
    // ========================================================================
    void computeControl(const SensorData& s, double dt, int& pwm_left, int& pwm_right) {
        float max_ir = max4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);
        float min_ir = min4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);

        bool all_white_out = (max_ir < 0.10f);
        bool all_black_air = (min_ir > 0.88f);

        // 1. Bao ve an toan khi nhac xe khoi san hoac vang khoi san > 0.85s
        // Luu y: Khong kich hoat was_lifted khi xe dang DUNG 2 GIAY TAI NGA TU (STATE_SHORTCUT_ALIGN)
        // hoac dang XOAY TAI NGA TU (STATE_SHORTCUT_TURN).
        if ((all_white_out || all_black_air) &&
            state != STATE_SHORTCUT_ALIGN && state != STATE_SHORTCUT_TURN) {
            off_track_time += dt;
            if (off_track_time > 0.85) was_lifted = true;
        } else {
            if (was_lifted) {
                // Khi dat xe xuong vach tro lai -> tu dong reset va chay tiep
                if ((s.ir[1] > 0.35f || s.ir[2] > 0.35f) && min_ir < 0.45f) {
                    reset(s.elapsed_sec);
                    has_seen_line = true;
                }
            } else {
                if (max_ir >= 0.20f && min_ir < 0.75f) {
                    if (!has_seen_line) run_start_time = s.elapsed_sec;
                    has_seen_line = true;
                }
                off_track_time = 0.0;
            }
        }

        if (was_lifted) {
            pwm_left  = 0;
            pwm_right = 0;
            return;
        }

        if (all_white_out) {
            lost_line_time += dt;
        } else {
            lost_line_time = 0.0;
        }

        // ====================================================================
        // 2. NHAN DIEN NGA TU CHUAN XAC & DUNG XE TINH TOAN 2 GIAY ROI XOAY
        // ====================================================================
        if (ENABLE_CROSS_SHORTCUT && has_seen_line && off_track_time < 0.05) {
            double run_elapsed = s.elapsed_sec - run_start_time;

            if (state == STATE_NORMAL_RACE && !shortcut_taken && cross_count < SHORTCUT_CROSS_COUNT) {
                // A. Cua so giu dinh 14ms (7 frames) cho 2 mat ngoai s0 va s3
                // Giup bat dinh nga tu 100% ke ca khi xe lao vao nga tu bi lech goc 15-20 do!
                if (s.ir[0] > 0.34f) {
                    wing_l_hold = 7;
                    if (s.ir[0] > wing_l_peak) wing_l_peak = s.ir[0];
                } else if (wing_l_hold > 0) {
                    // Neu mat ngoai da trang tinh (< 0.12f) thi xa nhanh de khong luu nhieu cua
                    if (s.ir[0] < 0.12f) wing_l_hold = 0;
                    else wing_l_hold--;
                    if (wing_l_hold == 0) wing_l_peak = 0.0f;
                }

                if (s.ir[3] > 0.34f) {
                    wing_r_hold = 7;
                    if (s.ir[3] > wing_r_peak) wing_r_peak = s.ir[3];
                } else if (wing_r_hold > 0) {
                    if (s.ir[3] < 0.12f) wing_r_hold = 0;
                    else wing_r_hold--;
                    if (wing_r_hold == 0) wing_r_peak = 0.0f;
                }

                // B. Kiem tra sai so 16ms TRUOC KHI CHAM NGA TU (lay tu ring buffer err_history)
                // Tai sao phai lay 16ms truoc? Vi ngay khoanh khac 1 canh cham nga tu som hon 2ms,
                // filtered_error hien tai se bi giat len. Lay 16ms truoc giup biet chinh xac xe dang
                // di tren vach thang hay dang vang o khuc cua gat!
                float pre_cross_err = err_history[err_hist_idx]; // Gia tri cach day 8 chu ky (16ms)
                bool  approach_not_cornering = (pre_cross_err < 1.85f && lost_line_time == 0.0);

                // C. Nguyen ly hinh hoc 4 mat:
                // Khoang cach s0 den s3 la ~50mm, vach don chi rong ~20mm.
                // Vach don KHONG BAO GIO lam ca s0, tam (s1/s2), va s3 cung den!
                bool center_on_line   = (s.ir[1] > 0.32f || s.ir[2] > 0.32f) &&
                                        ((s.ir[1] + s.ir[2]) > 0.48f);
                bool both_wings_live  = (s.ir[0] > 0.26f && s.ir[3] > 0.26f) &&
                                        (max2f(s.ir[0], s.ir[3]) > 0.38f);
                bool both_wings_held  = (wing_l_hold > 0 && wing_r_hold > 0 &&
                                         wing_l_peak > 0.36f && wing_r_peak > 0.36f);
                float total_black_sum = s.ir[0] + s.ir[1] + s.ir[2] + s.ir[3];
                float peak_black_sum  = wing_l_peak + s.ir[1] + s.ir[2] + wing_r_peak;

                bool is_cross_pattern = center_on_line && both_wings_live && both_wings_held &&
                                        (total_black_sum >= 1.45f || peak_black_sum >= 1.70f) &&
                                        approach_not_cornering;

                if (is_cross_pattern) {
                    cross_confirm_frames++;
                } else if (s.ir[0] < 0.20f || s.ir[3] < 0.20f) {
                    cross_confirm_frames = 0;
                }

                // D. Cooldown chong nhan trung (Cho phep bat ngay tu 50ms dau tien sau khi dat xe!)
                bool cooldown_ok = (last_cross_time < 0.0)
                                   ? (run_elapsed > 0.05)
                                   : ((s.elapsed_sec - last_cross_time) > 0.40);

                if (cross_confirm_frames >= 2 && cooldown_ok) {
                    cross_confirm_frames = 0;
                    wing_l_hold          = 0;
                    wing_r_hold          = 0;
                    wing_l_peak          = 0.0f;
                    wing_r_peak          = 0.0f;
                    cross_count++;
                    last_cross_time   = s.elapsed_sec;
                    cross_dist_m      = total_dist_m;
                    cross_yaw_rad     = total_yaw_rad;
                    max_yaw_excursion = 0.0;

                    if (cross_count < SHORTCUT_CROSS_COUNT) {
                        enterState(STATE_CROSS_LOCK, s.elapsed_sec);
                    } else {
                        // KICH HOAT DUNG XE NGAY LAP TUC & TINH TOAN 2.0 GIAY!
                        calc_lr_bias      = (s.ir[3] - s.ir[0]);
                        calc_samples      = 1;
                        turn_cleared_line = false;
                        enterState(STATE_SHORTCUT_ALIGN, s.elapsed_sec);
                    }
                }
            }

            double state_dur = s.elapsed_sec - state_enter_time;

            if (state == STATE_CROSS_LOCK) {
                bool cleared_wings = (state_dur > 0.035 && s.ir[0] < 0.20f && s.ir[3] < 0.20f);
                if (cleared_wings || state_dur > 0.070) {
                    state = STATE_NORMAL_RACE;
                }
            } else if (state == STATE_SHORTCUT_ALIGN) {
                // ============================================================
                // GIAI DOAN 1: DUNG HAN XE TAI NGA TU & TINH TOAN TRONG 2.0 GIAY
                // ============================================================
                if (state_dur < CROSS_STOP_WAIT_SEC) {
                    // Trong 55ms dau tien: Phanh nguoc (-180 PWM) de xe dung khing ngay tai nga tu
                    if (state_dur < 0.055) {
                        pwm_left  = -180;
                        pwm_right = -180;
                    } else {
                        // Dung im tuyet doi (PWM = 0, 0) cho den khi du 2.0 giay
                        pwm_left  = 0;
                        pwm_right = 0;
                    }

                    // TINH TOAN TRONG 2 GIAY DUNG XE:
                    calc_lr_bias += (s.ir[3] + 0.5f * s.ir[2]) - (s.ir[0] + 0.5f * s.ir[1]);
                    calc_samples++;

                    filtered_deriv = 0.0f;
                    straight_time  = 0.0;
                    if (state_dur >= 0.060) {
                        est_v_left  = 0.0f;
                        est_v_right = 0.0f;
                    }

                    if (CROSS_TURN_DIRECTION != 0) {
                        decided_turn_dir = (CROSS_TURN_DIRECTION > 0) ? 1 : -1;
                    } else {
                        decided_turn_dir = (calc_lr_bias >= 0.0f) ? 1 : -1;
                    }
                    last_valid_side = decided_turn_dir;

                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                } else {
                    // Het 2.0 giay dung xe -> Chuyen sang XOAY!
                    turn_cleared_line = false;
                    enterState(STATE_SHORTCUT_TURN, s.elapsed_sec);
                    state_dur = 0.0;
                }
            }

            if (state == STATE_SHORTCUT_TURN) {
                // ============================================================
                // GIAI DOAN 2: XOAY TAI CHO SAU KHI DA DUNG DU 2.0 GIAY
                // ============================================================
                int dir = decided_turn_dir; // +1: Xoay Phai, -1: Xoay Trai

                // 0.00s .. 0.16s: Xoay vuot qua vach thang hien tai
                if (state_dur < 0.16) {
                    pwm_left  =  205 * dir;
                    pwm_right = -185 * dir;
                    last_valid_side = dir;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }

                if ((s.ir[1] < 0.32f && s.ir[2] < 0.32f) || state_dur > 0.20) {
                    turn_cleared_line = true;
                }

                // Tiep tuc xoay den khi 2 mat giua bat duoc vach nhanh moi
                bool caught_new_line = turn_cleared_line && (s.ir[1] > 0.45f || s.ir[2] > 0.45f);
                if (caught_new_line || state_dur > 0.55) {
                    if (REPEAT_EVERY_CROSS) {
                        shortcut_taken = false;
                        cross_count    = 0;
                    } else {
                        shortcut_taken = true;
                    }

                    state           = STATE_NORMAL_RACE;
                    last_cross_time = s.elapsed_sec; // Cap nhat moc thoi gian ket thuc xoay
                    cross_dist_m    = total_dist_m;
                    wing_l_hold     = 0;
                    wing_r_hold     = 0;
                    wing_l_peak     = 0.0f;
                    wing_r_peak     = 0.0f;
                    for (int i = 0; i < 8; i++) err_history[i] = 0.0f;

                    float init_err = computeLineError(s.ir);
                    pos_error      = init_err;
                    prev_pos_error = init_err;
                    filtered_error = init_err;
                    filtered_deriv = 0.0f;
                    last_valid_side = dir;
                } else {
                    pwm_left  =  185 * dir;
                    pwm_right = -165 * dir;
                    last_valid_side = dir;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }
            }
        }

        // 3. TINH SAI SO & BO LOC DAO HAM KHONG TRE PHA (ZERO-LAG DERIVATIVE)
        prev_pos_error = pos_error;
        pos_error      = computeLineError(s.ir);

        // Khi vua bat lai vach sau khi mat trang hoan toan -> dong bo ngay lap tuc
        // de khong tao xung dao ham nguoc lam giat dau xe
        if (was_all_white && !all_white_out) {
            prev_pos_error = pos_error;
            filtered_error = pos_error;
            filtered_deriv *= 0.25f;
        }
        was_all_white = all_white_out;

        // Loc sieu nhe cho P (alpha = 0.78 -> do tre < 0.5ms, phan xa tuc thi)
        filtered_error = 0.22f * filtered_error + 0.78f * pos_error;

        // Luu |filtered_error| vao ring buffer 8 chu ky (16ms) khi 2 canh ngoai khong cung den
        if (!(s.ir[0] > 0.26f && s.ir[3] > 0.26f)) {
            err_history[err_hist_idx] = fabsf(filtered_error);
            err_hist_idx = (err_hist_idx + 1) & 0x07;
        }

        // Tinh dao ham thuc de/dt (Don vi: don vi sai so / giay)
        // Khong kep tran 45 nhu code cu! Cho phep len toi +-220 de bat kip cua gat
        float raw_deriv = 0.0f;
        if (!all_white_out && dt > 0.0002) {
            raw_deriv = (pos_error - prev_pos_error) / (float)dt;
            raw_deriv = clampf(raw_deriv, -220.0f, 220.0f);
        }

        // Bo loc EMA cho D voi hang so thoi gian tau_d = 6.5ms (du muot chong nhieu ADC, du nhanh de phanh)
        float alpha_d  = clampf((float)(dt / (0.0065 + dt)), 0.18f, 0.55f);
        filtered_deriv = (1.0f - alpha_d) * filtered_deriv + alpha_d * raw_deriv;

        float abs_e = fabsf(filtered_error);
        float abs_d = fabsf(filtered_deriv);

        // 4. QUY DAO TOC DO THICH UNG THEO DO CONG + PHANH DU DOAN DAO HAM
        // Neu xe dang o gan tam (|e| < 0.75) -> tich luy thoi gian duong thang de len Turbo muot
        if (abs_e < 0.75f && !all_white_out) {
            straight_time = min2f((float)(straight_time + dt), 0.25f);
        } else if (abs_e > 1.35f || all_white_out) {
            straight_time = 0.0; // Ngat Turbo ngay lap tuc khi chom vao cua!
        } else {
            straight_time = max2f(0.0f, (float)(straight_time - 2.0 * dt));
        }

        float turbo_ratio = clampf((float)(straight_time / 0.12), 0.0f, 1.0f);
        float peak_base   = lerpf(NOMINAL_BASE_PWM, STRAIGHT_MAX_PWM, turbo_ratio);

        // Giam toc nen theo:
        //  (a) Do lech vach phi tuyen: CURVE_SLOWDOWN_KE * |e|^1.30
        //  (b) Toc do lao ngang ra khoi vach: CURVE_SLOWDOWN_KD * max(0, |de/dt| - 12.0)
        // -> Giup xe TU DONG RÀ PHANH ngay khoanh khac vach vua dich chuyen nhanh vao cua!
        float err_penalty   = CURVE_SLOWDOWN_KE * powf(max2f(0.0f, abs_e - 0.15f), 1.30f);
        float deriv_penalty = CURVE_SLOWDOWN_KD * max2f(0.0f, abs_d - 12.0f);
        float target_base   = clampf(peak_base - err_penalty - deriv_penalty, CORNER_MIN_PWM, STRAIGHT_MAX_PWM);

        // 5. TINH LUC LAI PD PHI TUYEN + TANG CUONG GIAM CHAN O CUA (CORNER D-BOOST)
        // O vung trung tam (|e| < 0.40), giam nhe Kp 12% de xe luot thang cuc em khong ngoay duoi.
        // Khi |e| > 1.0 (vao cua), thanh phan KP_NONLINEAR * e * |e| tang luc be lai theo ham bac 2!
        float center_softener = (abs_e < 0.40f) ? (0.88f + 0.30f * abs_e) : 1.0f;
        float p_term = (KP_BASE * center_softener) * filtered_error
                     + KP_NONLINEAR * filtered_error * abs_e;

        // He so D tu dong manh hon khi xe dang o mep ngoai (abs_e lon) de chong vang qua da
        float effective_kd = KD_BASE + KD_CORNER_BOOST * clampf(abs_e / 2.50f, 0.0f, 1.2f);
        float d_term       = effective_kd * filtered_deriv;

        float turn_effort  = p_term + d_term;
        if (state == STATE_CROSS_LOCK) {
            target_base = NOMINAL_BASE_PWM;
            turn_effort = clampf(turn_effort, -40.0f, 40.0f);
        } else {
            turn_effort = clampf(turn_effort, -380.0f, 380.0f);
        }

        // 6. PHOI LUC VI SAI BAT DOI XUNG (ASYMMETRIC TORQUE VECTORING) & KHOA CUA MAT VACH
        float v_left = 0.0f, v_right = 0.0f;

        // Kiem tra mat vach thuc su o khuc cua (chi can mat trang >= 6ms khi dang lech tam)
        bool corner_line_lost = (all_white_out && state == STATE_NORMAL_RACE &&
                                 (!center_locked || lost_line_time >= 0.010));

        if (!has_seen_line) {
            // Luc moi bat nguon dat tren san, tien thang nhe de bat vach
            if (off_track_time < 0.45) {
                v_left  = 145.0f;
                v_right = 145.0f;
            }
        } else if (off_track_time > 0.85) {
            v_left  = 0.0f;
            v_right = 0.0f;
        } else if (corner_line_lost) {
            // CHE DO PIVOT RECOVERY TUC THI:
            // Banh ngoai day manh, banh trong dao chieu phanh xoay quanh truc de bat lai vach ngay!
            float lost_boost = clampf((float)(lost_line_time / 0.060), 0.0f, 1.0f);
            float outer_pwm  = lerpf(210.0f, 235.0f, lost_boost);
            float inner_pwm  = lerpf(-115.0f, MAX_REVERSE_PWM, lost_boost);

            if (last_valid_side >= 0) {
                // Vach o ben PHAI -> Quay PHAI gat
                v_left  = outer_pwm;
                v_right = inner_pwm;
            } else {
                // Vach o ben TRAI -> Quay TRAI gat
                v_left  = inner_pwm;
                v_right = outer_pwm;
            }
        } else {
            // CHE DO PHOI LUC VI SAI LIEN TUC TREN VACH:
            // Khi turn_effort lon (vao cua gat), thay vi de banh ngoai bi bao hoa o 255
            // va mat hieu luc vi sai, ta uu tien GIU TRAN BANH NGOAI va TRU THANG VAO BANH TRONG
            // de banh trong phanh/dao chieu lam tam xoay!
            v_left  = target_base + turn_effort;
            v_right = target_base - turn_effort;

            // Neu banh ngoai vuot tran MAX_PWM_OUT, chuyen phan du sang dap phanh banh trong:
            if (v_left > MAX_PWM_OUT) {
                float excess = v_left - MAX_PWM_OUT;
                v_left   = MAX_PWM_OUT;
                v_right -= 0.85f * excess;
            } else if (v_right > MAX_PWM_OUT) {
                float excess = v_right - MAX_PWM_OUT;
                v_right  = MAX_PWM_OUT;
                v_left  -= 0.85f * excess;
            }

            // Mo khoa phanh dao chieu banh trong muot ma ngay tu khi |e| > 0.55 (rat som!)
            float rev_allow   = clampf((abs_e - 0.55f) / 1.95f, 0.0f, 1.0f);
            float min_allowed = lerpf(0.0f, MAX_REVERSE_PWM, rev_allow);

            v_left  = clampf(v_left,  min_allowed, MAX_PWM_OUT);
            v_right = clampf(v_right, min_allowed, MAX_PWM_OUT);
        }

        pwm_left  = (int)clampf(v_left,  -MAX_PWM_OUT, MAX_PWM_OUT);
        pwm_right = (int)clampf(v_right, -MAX_PWM_OUT, MAX_PWM_OUT);

        updateVirtualOdometry(pwm_left, pwm_right, dt);
    }
};

// ============================================================================
// 5. DIEU KHIEN PHAN CUNG DONG CO & AUTO-CALIB CAM BIEN THONG MINH
// ============================================================================
CobotBrain brain;

// Luu nguong Trang / Den doc lap cho tung mat cam bien
int ir_white_cal[4] = {300, 300, 300, 300};
int ir_black_cal[4] = {2400, 2400, 2400, 2400};

unsigned long startMicros         = 0;
unsigned long lastStepMicros      = 0;
unsigned long lastTelemetryMillis = 0;
uint32_t      loopFrameCount      = 0;
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
    if (mag >= 250) return (pwm > 0) ? 255 : -255;
    int out = MIN_N20_PWM + ((mag * (255 - MIN_N20_PWM)) / 255);
    out = constrain(out, 0, 255);
    return (pwm > 0) ? out : -out;
}

// ============================================================================
// BO GIOI HAN GIA TOC BAT DOI XUNG (ASYMMETRIC SLEW LIMITER):
// - Khi PHANH hoac DAO CHIEU VAO CUA: Cho phep dap ung CUC NHANH (180 PWM / 2ms)
//   tuc la mat chua toi 4ms de dao chieu banh xe -> XOA BO HOAN TOAN DO TRE VAO CUA!
// - Khi TANG TOC DUONG THANG: Tang muot (65 PWM / 2ms) de banh xe bam duong, khong xoan banh.
// ============================================================================
static inline int slewLimitPWM(int target, int prev) {
    int diff = target - prev;
    // Kiem tra xem dong co dang tang toc cung chieu hay dang giam toc / phanh nguoc
    bool is_accelerating_forward = (target > prev && prev >= 0);
    if (is_accelerating_forward) {
        if (diff > 65) return prev + 65;
        return target;
    } else {
        // Phanh hoac dao chieu: dap ung gan nhu tuc thi!
        if (diff > 180)  return prev + 180;
        if (diff < -180) return prev - 180;
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

// Doc ADC kem bo loc Trung vi 3 mau (Median-of-3) + Xa tu MUX ADC ESP32
static inline int readIRFiltered(int pin) {
    (void)analogRead(pin); // Xa dien tich kenh MUX truoc do
    int a = analogRead(pin);
    int b = analogRead(pin);
    int c = analogRead(pin);
    int hi = (a > b) ? a : b;
    int lo = (a < b) ? a : b;
    return (c > hi) ? hi : ((c < lo) ? lo : c);
}

// ============================================================================
// 6. KHOI TAO HE THONG & CALIB TUONG PHAN THONG MINH
// ============================================================================
void setup() {
    pinMode(IR_LED_ON, OUTPUT);
    digitalWrite(IR_LED_ON, HIGH);
    pinMode(W_LED_ON, OUTPUT);
    digitalWrite(W_LED_ON, HIGH);

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

    setupMotorsHardware();
    setMotorHardware(0, 0);

    // Buoc 1: Doi nguoi thi dat xe xuong san (toi da 1.2s neu dang cam tren tay)
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]);
        int r3 = readIRFiltered(PIN_IR[3]);
        if (r0 < 1500 || r3 < 1500) break;
        delay(8);
    }

    // Buoc 2: Cho tay nguoi dung roi khoi xe (do dao dong ADC on dinh trong 90ms)
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

    // Buoc 3: Do mau nhanh tai vi tri xuat phat (Khi dat xe de vach: co mat tren nen trang, co mat tren vach den)
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

    // Tim nen trang thuc te (mat dang nam ngoai vach) va vach den thuc te (mat dang nam tren vach)
    int global_min = min(min(min_seen[0], min_seen[1]), min(min_seen[2], min_seen[3]));
    int global_max = max(max(max_seen[0], max_seen[1]), max(max_seen[2], max_seen[3]));

    // Dam bao an toan tuyet doi du dat xe o bat ky dau (ke ca dat lech han ra nen trang)
    if (global_min > 1200) global_min = 320;
    if (global_max < global_min + 650 || global_max > 3850) {
        global_max = constrain(global_min + 1500, 1400, 3200);
    }

    // Khoi tao dong deu cho ca 4 mat tu global_min / global_max de 2 mat giua KHONG BAO GIO
    // bi nhan nham nen den la nen trang!
    for (int i = 0; i < 4; i++) {
        if (min_seen[i] < global_min + 220) {
            ir_white_cal[i] = min_seen[i];
        } else {
            ir_white_cal[i] = global_min;
        }

        if (max_seen[i] > ir_white_cal[i] + 650 && max_seen[i] < 3850) {
            ir_black_cal[i] = max_seen[i];
        } else {
            ir_black_cal[i] = max(global_max, ir_white_cal[i] + 1200);
        }
    }

    brain.reset(0.0);
    startMicros    = micros();
    lastStepMicros = startMicros;
}

// ============================================================================
// 7. VONG LAP THOI GIAN THUC 500Hz (CHU KY CHUAN 2000 MICROSECONDS)
// ============================================================================
void loop() {
    unsigned long nowMicros = micros();
    unsigned long diffMicros = nowMicros - lastStepMicros;
    if (diffMicros < 2000) return; // Khoa tan so dieu khien chuan 500Hz (dt = 2.0ms)
    lastStepMicros = nowMicros;

    double dt = diffMicros * 1e-6;
    if (dt > 0.025) dt = 0.002; // Bao ve neu co ngat he thong

    loopFrameCount++;

    SensorData sData;
    sData.elapsed_sec = (nowMicros - startMicros) * 1e-6;

    // 1. Doc 4 mat cam bien IR
    int frame_min_raw = 4095;
    int frame_max_raw = 0;
    for (int i = 0; i < 4; i++) {
        int raw = readIRFiltered(PIN_IR[i]);
        sData.raw_ir[i] = raw;
        if (raw < frame_min_raw) frame_min_raw = raw;
        if (raw > frame_max_raw) frame_max_raw = raw;
    }

    // Kiem tra khung hinh co do tuong phan hop le (dang co ca mat tren nen trang va mat tren vach den)
    int frame_span = frame_max_raw - frame_min_raw;
    bool valid_contrast = (frame_span > 500) && (frame_min_raw < 1300) && (frame_max_raw < 3950);
    bool slow_tick      = ((loopFrameCount & 0x1F) == 0); // Moi 32 chu ky (~64ms)

    // 2. Cap nhat Auto-Calib thich ung lien tuc cho tung mat doc lap & Chuan hoa [0.0 .. 1.0]
    for (int i = 0; i < 4; i++) {
        int raw = sData.raw_ir[i];

        if (valid_contrast && brain.off_track_time < 0.04) {
            // Cap nhat muc nen trang (nhanh khi thay trang hon, hoi phục cham khi anh sang san thay doi)
            if (raw > 20 && raw < ir_white_cal[i]) {
                ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            } else if (slow_tick && raw < ir_white_cal[i] + 250 && ir_white_cal[i] < 1100) {
                ir_white_cal[i] += 1;
            }

            // Cap nhat muc vach den (khong khoa tran 2200 nhu code cu, cho phep len toi 3800)
            if (raw > ir_white_cal[i] + 600 && raw < 3850) {
                if (raw > ir_black_cal[i]) {
                    ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
                }
            }
        }

        // Tu dong ha nhe dinh black_cal ve muc (88% dinh thuc) de khi xe luot qua vach o toc do 245 PWM
        // tin hieu van luon dat bao hoa 0.90..1.00 ma khong bi suy hao!
        int min_valid_black = ir_white_cal[i] + 650;
        if (slow_tick && ir_black_cal[i] > min_valid_black + 250) {
            ir_black_cal[i] -= 3;
        }
        if (ir_black_cal[i] < min_valid_black) {
            ir_black_cal[i] = min_valid_black;
        }

        // Lay 88% bien do den-trang lam nguong bao hoa 1.0 de tang do nhay khi luot nhanh
        int effective_span = ((ir_black_cal[i] - ir_white_cal[i]) * 88) / 100;
        if (effective_span < 500) effective_span = 500;

        float norm = (float)(raw - ir_white_cal[i]) / (float)effective_span;
        sData.ir[i] = clampf(norm, 0.0f, 1.0f);
    }

    // 3. Tinh toan luat dieu khien va xuat xung PWM
    int pwm_l = 0, pwm_r = 0;
    brain.computeControl(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);

#if ENABLE_SERIAL_TELEMETRY
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 120 && Serial.availableForWrite() > 64) {
        lastTelemetryMillis = nowMs;
        Serial.printf("[t=%5.2fs] Err=%+5.2f dE=%+6.1f | Raw=[%4d,%4d,%4d,%4d] | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, brain.filtered_error, brain.filtered_deriv,
                      sData.raw_ir[0], sData.raw_ir[1], sData.raw_ir[2], sData.raw_ir[3],
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      pwm_l, pwm_r);
    }
#endif
}
