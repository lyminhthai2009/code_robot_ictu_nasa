#include <Arduino.h>
#include <math.h>
#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
// 1. CAU HINH CHIEN THUAT & THONG SO DUONG DUA (CHINH NHANH TAI DAY)
// ============================================================================
#define ENABLE_SERIAL_TELEMETRY 0 // 0 = Tat Serial khi chay thi that | 1 = Bat log debug
#define ENABLE_CROSS_SHORTCUT   0 // 0 = Di thang qua moi nga tu (Chuan) | 1 = Re tat Vong tron 2
#define SHORTCUT_CROSS_COUNT    3 // Nga tu thu 3 (Vong tron 2) se re tat (neu bat = 1)

// --- CAU HINH TOC DO: DUONG THANG NHANH, VAO CUA GIAM TOC SOM BAM DUONG ---
const float STRAIGHT_MAX_PWM   = 248.0f; // Toc do toi da tren duong thang
const float NOMINAL_BASE_PWM   = 212.0f; // Toc do nen on dinh
const float CORNER_MIN_PWM     = 108.0f; // Ha toc do nen du thap khi vao cua de lop xe khong bao gio truot vang!
const float MAX_PWM_OUT        = 255.0f; // Tran PWM phan cung
const float MAX_REVERSE_PWM    = -150.0f;// Luc phanh/dao chieu banh trong khi cua gat

// --- THONG SO BO DIEU KHIEN PD LIEN TUC SIEU MUOT (500Hz, dt = 0.002s) ---
const float KP_BASE            = 48.0f;  // He so P tuyen tinh giu tam muot ma
const float KP_NONLINEAR       = 19.0f;  // He so P bac 2 tang luc om cua: Kp2 * e * |e|
const float KD_BASE            = 1.80f;  // He so D lien tuc (khong nhay bac gay giat trong cua!)
const float KD_CORNER_BOOST    = 0.45f;  // Tang nhe D khi o mep ngoai de chong vang

// --- THONG SO PHANH SOM KHI VAO CUA (CHONG VANG LINE 100%) ---
const float CURVE_SLOWDOWN_KE  = 36.0f;  // Phanh som ngay khi vach bat dau lech tam (|e| > 0.15)
const float CURVE_SLOWDOWN_KD  = 1.25f;  // Phanh du doan ngay khoanh khac vach dich chuyen nhanh vao cua

// ============================================================================
// 2. CAU HINH CHAN PHAN CUNG ESP32 & DONG CO N20 (GIU NGUYEN BAN GOC CUA BAN)
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
const int MIN_N20_PWM        = 34;
const float RIGHT_MOTOR_TRIM = 1.00f;

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
    float ir[4];      // [0.0 = Trang tinh, 1.0 = Den tuyet doi]
    int raw_ir[4];    // [0 .. 4095]
    double elapsed_sec;
};

enum TrackState {
    STATE_NORMAL_RACE = 0,
    STATE_SHORTCUT_ALIGN,
    STATE_SHORTCUT_TURN
};

// ============================================================================
// 4. BO DIEU KHIEN TRUNG TAM (SMOOTH CORNERING & SAFE INTERSECTION 500Hz)
// ============================================================================
class CobotBrain {
public:
    TrackState state = STATE_NORMAL_RACE;

    float pos_error          = 0.0f;
    float prev_pos_error     = 0.0f;
    float filtered_error     = 0.0f;
    float filtered_deriv     = 0.0f;
    float outer_peak_memory  = 0.0f;

    int   last_valid_side    = 1;
    bool  center_locked      = true;
    bool  was_all_white      = false;
    bool  was_lifted         = false;
    bool  has_seen_line      = false;
    bool  in_cross_zone      = false; // Co bao dang di qua giao lo nga tu

    double lost_line_time    = 0.0;
    double off_track_time    = 0.0;
    double straight_time     = 0.0;
    double active_run_time   = 0.0;
    double cross_zone_timer  = 0.0;

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
        in_cross_zone        = false;

        lost_line_time       = 0.0;
        off_track_time       = 0.0;
        straight_time        = 0.0;
        active_run_time      = 0.0;
        cross_zone_timer     = 0.0;

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
    // THUAT TOAN NOI SUY VACH & CHONG BAY KHOI LINE O NGA TU CHU X
    // ========================================================================
    float computeLineError(const float s[4], double dt) {
        float w[4];
        for (int i = 0; i < 4; i++) {
            float clean = clampf((s[i] - 0.06f) / 0.94f, 0.0f, 1.0f);
            w[i] = clean * (0.75f + 0.25f * clean);
        }

        // --- XU LY NGA TU CHU X THONG MINH (KHONG KHOA MU TAY LAI!) ---
        // Tai nga tu chu X (Vong tron 1 & 2), nhanh giao cat di vao tu 2 ben canh (w[0] va w[3]).
        // Thay vi khoa cung tay lai trong 65ms lam xe lao vao khe trang giua chu '<',
        // ta chi khu phan chung (common-mode) cua 2 canh ngoai ngay trong khoanh khac cat qua giao diem,
        // va GIU NGUYEN 100% phan xa bam tam cua 2 mat giua (w[1], w[2])!
        float raw_sum = w[0] + w[1] + w[2] + w[3];
        bool center_strong = (w[1] > 0.32f || w[2] > 0.32f);
        bool both_wings_active = (w[0] > 0.35f && w[3] > 0.35f);

        // Neu xe dang bam tam (|pos_error| < 0.85) ma dot ngot 1 canh ngoai cham nhanh cheo cua chu X
        // trong khi ca 2 mat giua van dang de len vach chinh (w[1] > 0.28 && w[2] > 0.28):
        // Giam bớt trọng số của cánh ngoài để xe không bị nhánh chéo hút lệch khỏi đường thẳng!
        if (center_locked && w[1] > 0.28f && w[2] > 0.28f && fabsf(pos_error) < 0.75f) {
            w[0] *= 0.25f;
            w[3] *= 0.25f;
        } else if (both_wings_active) {
            if (center_strong) {
                // Triệt tiêu đối xứng 2 cánh ngoài khi đang cắt qua tâm chữ X
                float common_wing = min2f(w[0], w[3]);
                w[0] -= common_wing * 0.90f;
                w[3] -= common_wing * 0.90f;
            } else {
                // Nếu đang ở cua gắt gặp nhánh rẽ -> chỉ giữ mắt cùng phía với hướng cua
                if (last_valid_side >= 0) w[0] = 0.0f;
                else w[3] = 0.0f;
            }
        }

        if (raw_sum > 1.75f && center_strong) {
            in_cross_zone    = true;
            cross_zone_timer = 0.035; // Duy trì giảm tốc nhẹ 35ms khi đi qua giao điểm chữ X
        } else if (cross_zone_timer > 0.0) {
            cross_zone_timer -= dt;
            if (cross_zone_timer <= 0.0) in_cross_zone = false;
        }

        float w_sum = w[0] + w[1] + w[2] + w[3];

        // Xử lý khi cả 4 mắt đều trắng
        if (w_sum < 0.05f) {
            // Nếu trước đó vạch đang ở sát tâm (|pos_error| < 0.65) và mới khuất < 12ms -> giữ nguyên
            if (center_locked && lost_line_time < 0.012 && fabsf(pos_error) < 0.65f) {
                return pos_error;
            }
            // Nếu vạch đã lệch ra mép -> Khóa cứng sai số cực đại để bẻ cua ngay lập tức!
            return (last_valid_side >= 0) ? 4.50f : -4.50f;
        }

        float num   = (-3.0f * w[0]) + (-1.0f * w[1]) + (1.0f * w[2]) + (3.0f * w[3]);
        float raw_e = num / w_sum;

        // NGOẠI SUY SƯỜN NGOÀI LIÊN TỤC (MƯỢT KHÔNG ĐIỂM GÃY ĐẠO HÀM):
        if (!both_wings_active) {
            // Sườn ngoài bên PHẢI (Mắt s3)
            if (w[3] > 0.04f && w[2] < 0.16f && w[1] < 0.05f && w[0] < 0.05f) {
                if (w[3] > outer_peak_memory) outer_peak_memory = w[3];
                if (pos_error >= 2.00f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[3]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = 3.00f + 1.45f * decay_ratio;
                    float blend       = clampf((0.16f - w[2]) / 0.14f, 0.0f, 1.0f);
                    raw_e             = lerpf(raw_e, max2f(raw_e, extrap_e), blend);
                }
            }
            // Sườn ngoài bên TRÁI (Mắt s0)
            else if (w[0] > 0.04f && w[1] < 0.16f && w[2] < 0.05f && w[3] < 0.05f) {
                if (w[0] > outer_peak_memory) outer_peak_memory = w[0];
                if (pos_error <= -2.00f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[0]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = -3.00f - 1.45f * decay_ratio;
                    float blend       = clampf((0.16f - w[1]) / 0.14f, 0.0f, 1.0f);
                    raw_e             = lerpf(raw_e, min2f(raw_e, extrap_e), blend);
                }
            } else if (w[1] > 0.25f || w[2] > 0.25f) {
                outer_peak_memory = 0.0f;
            }
        }

        if ((w[1] > 0.22f || w[2] > 0.22f) && w[0] < 0.18f && w[3] < 0.18f && fabsf(raw_e) < 0.90f) {
            center_locked = true;
        } else if (fabsf(raw_e) > 1.15f || w[0] > 0.30f || w[3] > 0.30f) {
            center_locked = false;
        }

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

        // 1. Bảo vệ khi nhấc xe khỏi sàn
        if ((all_white_out || all_black_air) && state != STATE_SHORTCUT_TURN) {
            off_track_time += dt;
            if (off_track_time > 0.85) was_lifted = true;
        } else {
            if (was_lifted) {
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
            active_run_time = 0.0;
            pwm_left  = 0;
            pwm_right = 0;
            return;
        }

        active_run_time += dt;

        if (all_white_out) {
            lost_line_time += dt;
        } else {
            lost_line_time = 0.0;
        }

        // 2. Xử lý Rẽ tắt Ngã tư Vòng tròn 2 (chỉ hoạt động khi ENABLE_CROSS_SHORTCUT == 1)
        if (ENABLE_CROSS_SHORTCUT && has_seen_line && off_track_time < 0.05) {
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

                    if (cross_count >= SHORTCUT_CROSS_COUNT) {
                        enterState(STATE_SHORTCUT_ALIGN, s.elapsed_sec);
                    }
                }
            }

            double state_dur = s.elapsed_sec - state_enter_time;

            if (state == STATE_SHORTCUT_ALIGN) {
                if (state_dur < 0.045) {
                    pwm_left  = 185;
                    pwm_right = 185;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                } else {
                    enterState(STATE_SHORTCUT_TURN, s.elapsed_sec);
                    state_dur = 0.0;
                }
            }

            if (state == STATE_SHORTCUT_TURN) {
                if (state_dur < 0.13) {
                    pwm_left  =  215;
                    pwm_right = -135;
                    last_valid_side = 1;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }
                if (s.ir[1] > 0.42f || s.ir[2] > 0.42f || state_dur > 0.36) {
                    shortcut_taken = true;
                    state          = STATE_NORMAL_RACE;
                    float init_err = computeLineError(s.ir, dt);
                    pos_error      = init_err;
                    prev_pos_error = init_err;
                    filtered_error = init_err;
                    filtered_deriv = 0.0f;
                    last_valid_side = 1;
                } else {
                    pwm_left  =  195;
                    pwm_right = -115;
                    last_valid_side = 1;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }
            }
        }

        // 3. TÍNH SAI SỐ & BỘ LỌC ĐẠO HÀM ÊM MƯỢT (KHÔNG GIẬT CỤC TRONG CUA!)
        prev_pos_error = pos_error;
        pos_error      = computeLineError(s.ir, dt);

        if (was_all_white && !all_white_out) {
            prev_pos_error = pos_error;
            filtered_error = pos_error;
            filtered_deriv *= 0.25f;
        }
        was_all_white = all_white_out;

        // Bộ lọc cho P vừa đủ nhạy vừa khử gai nhiễu (alpha = 0.72)
        filtered_error = 0.28f * filtered_error + 0.72f * pos_error;

        float raw_deriv = 0.0f;
        if (!all_white_out && dt > 0.0002) {
            raw_deriv = (pos_error - prev_pos_error) / (float)dt;
            raw_deriv = clampf(raw_deriv, -180.0f, 180.0f);
        }

        // Hằng số lọc EMA tau_d = 8.5ms giúp lực D mượt như nhung, triệt tiêu hoàn toàn hiện tượng giật cục trong cua!
        float alpha_d  = clampf((float)(dt / (0.0085 + dt)), 0.15f, 0.42f);
        filtered_deriv = (1.0f - alpha_d) * filtered_deriv + alpha_d * raw_deriv;

        float abs_e = fabsf(filtered_error);
        float abs_d = fabsf(filtered_deriv);

        // 4. QUỸ ĐẠO TỐC ĐỘ THÍCH ỨNG: ĐƯỜNG THẲNG NHANH, CHỚM VÀO CUA LÀ RÀ PHANH NGAY!
        if (abs_e < 0.70f && !all_white_out && !in_cross_zone) {
            straight_time = min2f((float)(straight_time + dt), 0.25f);
        } else if (abs_e > 1.25f || all_white_out) {
            straight_time = 0.0; // Ngắt Turbo ngay lập tức khi chớm vào cua!
        } else {
            straight_time = max2f(0.0f, (float)(straight_time - 2.0f * dt));
        }

        float turbo_ratio = clampf((float)(straight_time / 0.10), 0.0f, 1.0f);
        float peak_base   = lerpf(NOMINAL_BASE_PWM, STRAIGHT_MAX_PWM, turbo_ratio);

        // Giảm tốc độ nền sớm ngay khi |e| > 0.15 để bánh xe bám chặt xuống mặt bàn khi vào cua!
        float err_penalty   = CURVE_SLOWDOWN_KE * powf(max2f(0.0f, abs_e - 0.15f), 1.30f);
        float deriv_penalty = CURVE_SLOWDOWN_KD * max2f(0.0f, abs_d - 12.0f);
        float target_base   = clampf(peak_base - err_penalty - deriv_penalty, CORNER_MIN_PWM, STRAIGHT_MAX_PWM);

        // Khi đang đi qua giao điểm Ngã tư chữ X (in_cross_zone), tự động kìm tốc độ ở mức 170 PWM
        // để xe đi thẳng qua giao lộ chữ X cực kỳ đằm và chuẩn xác, không bị bay vào khe trắng!
        if (in_cross_zone && target_base > 170.0f) {
            target_base = 170.0f;
        }

        if (active_run_time < 0.150) {
            float soft_launch = clampf((float)(active_run_time / 0.150), 0.58f, 1.0f);
            target_base *= soft_launch;
        }

        // 5. TÍNH LỰC LÁI PD PHI TUYẾN LIÊN TỤC (KHÔNG NHẢY BẬC GÂY GIẬT)
        float center_softener = (abs_e < 0.40f) ? (0.88f + 0.30f * abs_e) : 1.0f;
        float p_term = (KP_BASE * center_softener) * filtered_error
                     + KP_NONLINEAR * filtered_error * abs_e;

        float effective_kd = KD_BASE + KD_CORNER_BOOST * clampf(abs_e / 2.50f, 0.0f, 1.15f);
        float d_term       = effective_kd * filtered_deriv;

        float turn_effort  = clampf(p_term + d_term, -360.0f, 360.0f);

        // 6. PHỐI LỰC VI SAI MƯỢT MÀ & KHÓA CUA TỨC THÌ (10ms) KHI MẤT VẠCH Ở CUA
        float v_left = 0.0f, v_right = 0.0f;

        // Khôi phục phản xạ bắt cua 10ms cực nhạy để xe không bao giờ bị văng khỏi khúc cua gắt!
        bool corner_line_lost = (all_white_out && state == STATE_NORMAL_RACE &&
                                 (!center_locked || lost_line_time >= 0.010));

        if (!has_seen_line) {
            if (off_track_time < 0.45) {
                v_left  = 150.0f;
                v_right = 150.0f;
            }
        } else if (off_track_time > 0.85) {
            v_left  = 0.0f;
            v_right = 0.0f;
        } else if (corner_line_lost) {
            // PIVOT RECOVERY MƯỢT NHƯNG DỨT KHOÁT:
            float lost_boost = clampf((float)(lost_line_time / 0.055), 0.0f, 1.0f);
            float outer_pwm  = lerpf(205.0f, 235.0f, lost_boost);
            float inner_pwm  = lerpf(-105.0f, MAX_REVERSE_PWM, lost_boost);

            if (last_valid_side >= 0) {
                v_left  = outer_pwm;
                v_right = inner_pwm;
            } else {
                v_left  = inner_pwm;
                v_right = outer_pwm;
            }
        } else {
            v_left  = target_base + turn_effort;
            v_right = target_base - turn_effort;

            if (v_left > MAX_PWM_OUT) {
                float excess = v_left - MAX_PWM_OUT;
                v_left   = MAX_PWM_OUT;
                v_right -= 0.78f * excess;
            } else if (v_right > MAX_PWM_OUT) {
                float excess = v_right - MAX_PWM_OUT;
                v_right  = MAX_PWM_OUT;
                v_left  -= 0.78f * excess;
            }

            // Mở khóa phanh đảo chiều bánh trong mượt mà theo độ lệch |e|
            float rev_allow   = clampf((abs_e - 0.75f) / 1.95f, 0.0f, 1.0f);
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
// 5. DIEU KHIEN PHAN CUNG SIÊU MƯỢT (KHÔNG BƯỚC NHẢY VÙNG CHẾT TRONG CUA)
// ============================================================================
CobotBrain brain;

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

// Bù ma sát tĩnh mượt (Smooth Deadzone): Không nhảy bậc đột ngột quanh điểm 0
// để bánh trong không bị giật khựng khi rà phanh trong khúc cua!
static inline int applyN20Deadzone(int pwm) {
    if (pwm == 0) return 0;
    int mag = abs(pwm);
    if (mag >= 250) return (pwm > 0) ? 255 : -255;
    int out;
    if (mag < 20) {
        // Vùng chuyển tiếp tuyến tính mềm quanh điểm 0 (0..20 -> 0..48) chống giật bánh trong!
        out = (mag * (MIN_N20_PWM + 14)) / 20;
    } else {
        out = MIN_N20_PWM + ((mag * (255 - MIN_N20_PWM)) / 255);
    }
    out = constrain(out, 0, 255);
    return (pwm > 0) ? out : -out;
}

static inline int slewLimitPWM(int target, int prev, int& zero_hold) {
    if (zero_hold > 0) {
        zero_hold--;
        return 0;
    }

    // Chỉ nghỉ 1 chu kỳ (2ms) ở điểm 0 khi đảo chiều từ tốc độ cao (|prev| > 90) để bảo vệ Pin
    // Còn khi bánh trong đang rà chậm quanh 0 ở trong cua thì cho lướt mượt qua luôn, không khựng!
    if ((target < 0 && prev > 90) || (target > 0 && prev < -90)) {
        zero_hold = 1;
        return 0;
    }

    int diff = target - prev;
    bool is_accelerating_forward = (target > prev && prev >= 0);
    if (is_accelerating_forward) {
        if (diff > 55) return prev + 55;
        return target;
    } else {
        if (diff > 150)  return prev + 150;
        if (diff < -150) return prev - 150;
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

    int out_l = slewLimitPWM(target_l, prev_hw_pwm_l, zero_cross_hold_l);
    int out_r = slewLimitPWM(target_r, prev_hw_pwm_r, zero_cross_hold_r);
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
// 6. KHOI TAO HE THONG & AUTO-CALIB
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

#if ENABLE_SERIAL_TELEMETRY
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
#endif

    for (int i = 0; i < 4; i++) {
        pinMode(PIN_IR[i], INPUT);
    }

    setupMotorsHardware();
    setMotorHardware(0, 0);

    // Bước 1: Đợi người thi đặt xe xuống sàn
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]);
        int r3 = readIRFiltered(PIN_IR[3]);
        if (r0 < 1500 || r3 < 1500) break;
        delay(8);
    }

    // Bước 2: Chờ tay người dùng rời khỏi xe
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

    // Bước 3: Đo mẫu nhanh tại vị trí xuất phát
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
    if (diffMicros < 2000) return;
    lastStepMicros = nowMicros;

    double dt = diffMicros * 1e-6;
    if (dt > 0.025) dt = 0.002;

    loopFrameCount++;

    SensorData sData;
    sData.elapsed_sec = (nowMicros - startMicros) * 1e-6;

    int frame_min_raw = 4095;
    int frame_max_raw = 0;
    for (int i = 0; i < 4; i++) {
        int raw = readIRFiltered(PIN_IR[i]);
        sData.raw_ir[i] = raw;
        if (raw < frame_min_raw) frame_min_raw = raw;
        if (raw > frame_max_raw) frame_max_raw = raw;
    }

    int frame_span = frame_max_raw - frame_min_raw;
    bool valid_contrast = (frame_span > 500) && (frame_min_raw < 1300) && (frame_max_raw < 3950);
    bool slow_tick      = ((loopFrameCount & 0x1F) == 0);

    for (int i = 0; i < 4; i++) {
        int raw = sData.raw_ir[i];

        if (valid_contrast && brain.off_track_time < 0.04) {
            if (raw > 20 && raw < ir_white_cal[i]) {
                ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            } else if (slow_tick && raw < ir_white_cal[i] + 250 && ir_white_cal[i] < 1100) {
                ir_white_cal[i] += 1;
            }

            if (raw > ir_white_cal[i] + 600 && raw < 3850) {
                if (raw > ir_black_cal[i]) {
                    ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
                }
            }
        }

        int min_valid_black = ir_white_cal[i] + 650;
        if (slow_tick && ir_black_cal[i] > min_valid_black + 250) {
            ir_black_cal[i] -= 3;
        }
        if (ir_black_cal[i] < min_valid_black) {
            ir_black_cal[i] = min_valid_black;
        }

        int effective_span = ((ir_black_cal[i] - ir_white_cal[i]) * 88) / 100;
        if (effective_span < 500) effective_span = 500;

        float norm = (float)(raw - ir_white_cal[i]) / (float)effective_span;
        sData.ir[i] = clampf(norm, 0.0f, 1.0f);
    }

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
