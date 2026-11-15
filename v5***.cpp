#include <Arduino.h>
#include <math.h>
#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
// 1. CAU HINH CHIEN THUAT & NGA TU (CHINH NHANH TAI DAY)
// ============================================================================
#define ENABLE_SERIAL_TELEMETRY 0 // 0 = Tat Serial khi chay thi that | 1 = Bat log debug
#define ENABLE_CROSS_SHORTCUT   0 // 0 = Di het Vong tron 1 & 2 (An toan 100%) | 1 = Re tat bo qua Vong tron 2!
#define SHORTCUT_CROSS_COUNT    3 // Nga tu thu 3 (Vong tron 2) se kich hoat re tat (neu bat = 1)

// --- CAU HINH TOC DO CAO & MUOT ---
const float STRAIGHT_MAX_PWM   = 255.0f; // Toc do toi da tren duong thang & Doc cau
const float NOMINAL_BASE_PWM   = 232.0f; // Toc do nen khi chay on dinh
const float CORNER_MIN_PWM     = 140.0f; // Toc do nen o cua gat (banh ngoai van luon dat 240..255)
const float MAX_PWM_OUT        = 255.0f; // Tran PWM phan cung
const float MAX_REVERSE_PWM    = -150.0f;// Luc phanh/dao chieu banh trong khi cua gat

// --- THONG SO BO DIEU KHIEN "SUPERGLUE PD + CURVATURE MEMORY" 500Hz ---
const float KP_BASE            = 54.0f;  // Tang luc giu tam sac ben
const float KP_NONLINEAR       = 22.5f;  // Tang luc ep cua phi tuyen: Kp2 * e * |e|
const float KD_DIVERGE         = 1.95f;  // He so D khi vach dang lech ra xa tam (hut mui xe vao vach)
const float KD_CONVERGE        = 2.55f;  // He so D khi mui xe dang lao ve tam (chong vay duoi 100%!)
const float KD_CORNER_BOOST    = 0.70f;  // Tang cuong D khi o mep ngoai
const float KI_CURVE_MEMORY    = 95.0f;  // Tich phan do cong giup xe bam chinh giua vach trong Vong tron 1 & 2

// --- THONG SO GIAM TOC THICH UNG THEO DO CONG & DAO HAM ---
const float CURVE_SLOWDOWN_KE  = 23.0f;  // Ha toc do nen muot ma theo |e|
const float CURVE_SLOWDOWN_KD  = 0.90f;  // Phanh du doan som khi vach vua dich chuyen nhanh

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
const int MIN_N20_PWM        = 36;
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
    STATE_CROSS_LOCK,
    STATE_SHORTCUT_ALIGN,
    STATE_SHORTCUT_TURN
};

// ============================================================================
// 4. BO DIEU KHIEN TRUNG TAM "SUPERGLUE BRAIN" 500Hz
// ============================================================================
class CobotBrain {
public:
    TrackState state = STATE_NORMAL_RACE;

    float pos_error          = 0.0f;
    float prev_pos_error     = 0.0f;
    float filtered_error     = 0.0f;
    float filtered_deriv     = 0.0f;
    float outer_peak_memory  = 0.0f;
    float curve_bias         = 0.0f; // Bo nho do cong giup xe bam sat tam vach khi om cung tron

    int   last_valid_side    = 1;
    bool  center_locked      = true;
    bool  was_all_white      = false;
    bool  was_lifted         = false;
    bool  has_seen_line      = false;

    double lost_line_time    = 0.0;
    double off_track_time    = 0.0;
    double straight_time     = 0.0;
    double active_run_time   = 0.0;

    // Trang thai xu ly Nga tu (Luon hoat dong de chong giat ngang khi qua nga tu)
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
        curve_bias           = 0.0f;

        last_valid_side      = 1;
        center_locked        = true;
        was_all_white      = false;
        was_lifted           = false;
        has_seen_line        = false;

        lost_line_time       = 0.0;
        off_track_time       = 0.0;
        straight_time        = 0.0;
        active_run_time      = 0.0;

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
        const float v_max = 0.88f;
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
    // THUAT TOAN NOI SUY VACH LIEN TUC KHONG DIEM GAY [-4.50 .. +4.50]
    // ========================================================================
    float computeLineError(const float s[4]) {
        float w[4];
        for (int i = 0; i < 4; i++) {
            float clean = clampf((s[i] - 0.05f) / 0.95f, 0.0f, 1.0f);
            w[i] = clean * (0.72f + 0.28f * clean);
        }

        // Phat hien nga tu hoac vach cat ngang (ke ca khi 2 canh ngoai cham lech nhau vài ms)
        bool both_wings_active = (w[0] > 0.36f && w[3] > 0.36f) ||
                                 (wing_l_hold > 0 && wing_r_hold > 0 && (w[1] > 0.25f || w[2] > 0.25f));

        if (state == STATE_CROSS_LOCK) {
            w[0] = 0.0f;
            w[3] = 0.0f;
        } else if (both_wings_active) {
            if (w[1] > 0.20f || w[2] > 0.20f) {
                // Đang cắt ngang ngã tư nhưng vẫn có tâm -> Khóa 2 cánh ngoài để đi thẳng mượt như ray!
                w[0] = 0.0f;
                w[3] = 0.0f;
            } else {
                if (last_valid_side >= 0) w[0] = 0.0f;
                else w[3] = 0.0f;
            }
        }

        float w_sum = w[0] + w[1] + w[2] + w[3];

        // Xử lý khi cả 4 mắt đều trắng
        if (w_sum < 0.05f) {
            if (state == STATE_CROSS_LOCK) return 0.0f;

            // Giữ hướng thẳng khi bay qua vạch xanh Gờ giảm tốc / Đỉnh dốc cầu
            if (center_locked && lost_line_time < 0.045 && fabsf(pos_error) < 0.85f) {
                return pos_error;
            }

            return (last_valid_side >= 0) ? 4.50f : -4.50f;
        }

        float num   = (-3.0f * w[0]) + (-1.0f * w[1]) + (1.0f * w[2]) + (3.0f * w[3]);
        float raw_e = num / w_sum;

        // NGOẠI SUY SƯỜN NGOÀI LIÊN TỤC (SEAMLESS BLENDING - KHÔNG GÂY GIẬT ĐẠO HÀM):
        // Hòa trộn mượt từ vùng Centroid (w_inner = 0.18) sang vùng Ngoại suy (w_inner = 0.02)
        if (!both_wings_active) {
            // Sườn ngoài bên PHẢI (Mắt s3)
            if (w[3] > 0.04f && w[2] < 0.18f && w[1] < 0.06f && w[0] < 0.06f) {
                if (w[3] > outer_peak_memory) outer_peak_memory = w[3];
                if (pos_error >= 1.95f || outer_peak_memory > 0.42f) {
                    float ref_peak    = max2f(0.55f, outer_peak_memory);
                    float decay_ratio = clampf((ref_peak - w[3]) / max2f(0.35f, ref_peak - 0.04f), 0.0f, 1.0f);
                    float extrap_e    = 3.00f + 1.48f * decay_ratio;
                    float blend       = clampf((0.18f - w[2]) / 0.15f, 0.0f, 1.0f);
                    float smooth_e    = lerpf(raw_e, max2f(raw_e, extrap_e), blend);
                    if (smooth_e > raw_e) raw_e = smooth_e;
                }
            }
            // Sườn ngoài bên TRÁI (Mắt s0)
            else if (w[0] > 0.04f && w[1] < 0.18f && w[2] < 0.06f && w[3] < 0.06f) {
                if (w[0] > outer_peak_memory) outer_peak_memory = w[0];
                if (pos_error <= -1.95f || outer_peak_memory > 0.42f) {
                    float ref_peak    = max2f(0.55f, outer_peak_memory);
                    float decay_ratio = clampf((ref_peak - w[0]) / max2f(0.35f, ref_peak - 0.04f), 0.0f, 1.0f);
                    float extrap_e    = -3.00f - 1.48f * decay_ratio;
                    float blend       = clampf((0.18f - w[1]) / 0.15f, 0.0f, 1.0f);
                    float smooth_e    = lerpf(raw_e, min2f(raw_e, extrap_e), blend);
                    if (smooth_e < raw_e) raw_e = smooth_e;
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
            if (w[3] > 0.15f || raw_e > 0.16f) {
                last_valid_side = 1;
            } else if (w[0] > 0.15f || raw_e < -0.16f) {
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

        // 1. Bảo vệ an toàn khi nhấc xe khỏi bàn đua
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

        // 2. XỬ LÝ NGÃ TƯ THÔNG MINH (CHỐNG GIẬT NGANG VÒNG TRÒN 1 & RẼ TẮT VÒNG TRÒN 2)
        // Luôn theo dõi 2 cánh ngoài (wing_l_hold, wing_r_hold) để ổn định xe khi cắt qua Ngã tư chữ X!
        if (has_seen_line && off_track_time < 0.05) {
            double dist_since_cross = total_dist_m - cross_dist_m;
            double run_elapsed      = s.elapsed_sec - run_start_time;

            if (state == STATE_NORMAL_RACE) {
                if (s.ir[0] > 0.48f) wing_l_hold = 6; // Nhớ cánh trái trong 12ms
                else if (wing_l_hold > 0) wing_l_hold--;

                if (s.ir[3] > 0.48f) wing_r_hold = 6; // Nhớ cánh phải trong 12ms
                else if (wing_r_hold > 0) wing_r_hold--;

                bool center_on_line  = (s.ir[1] > 0.38f || s.ir[2] > 0.38f);
                bool sum_very_dark   = ((s.ir[0] + s.ir[1] + s.ir[2] + s.ir[3]) > 1.85f);
                bool all_wings_black = (wing_l_hold > 0 && wing_r_hold > 0 && sum_very_dark &&
                                        center_on_line && fabsf(filtered_error) < 1.20f);

                if (all_wings_black) cross_confirm_frames++;
                else cross_confirm_frames = 0;

                // Điều kiện hình học đường đua:
                // - Ngã tư 1 (Vào Vòng tròn 1): cách vạch xuất phát > 0.15m
                // - Ngã tư 2 (Ra Vòng tròn 1): phải vừa quay 1 vòng tròn (max_yaw_excursion > 1.8 rad ~ 105 độ)
                // - Ngã tư 3 (Vào Vòng tròn 2): phải vừa đi qua Cua chữ U giữa 2 gờ giảm tốc (max_yaw_excursion > 1.2 rad ~ 70 độ)
                // -> Nhờ kiểm tra góc quay max_yaw_excursion, các vạch ngang trên Gờ giảm tốc KHÔNG BAO GIỜ bị đếm nhầm là Ngã tư!
                double req_yaw = (cross_count == 1) ? 1.80 : ((cross_count >= 2) ? 1.15 : 0.08);
                bool topo_ok   = (cross_count == 0)
                                 ? (run_elapsed > 0.22 && total_dist_m > 0.15)
                                 : ((s.elapsed_sec - last_cross_time) > 0.45 &&
                                    dist_since_cross > 0.25 &&
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

                    if (ENABLE_CROSS_SHORTCUT && !shortcut_taken && cross_count == SHORTCUT_CROSS_COUNT) {
                        enterState(STATE_SHORTCUT_ALIGN, s.elapsed_sec);
                    } else {
                        // Khóa hướng đi thẳng mượt qua Ngã tư chữ X!
                        enterState(STATE_CROSS_LOCK, s.elapsed_sec);
                    }
                }
            }

            double state_dur = s.elapsed_sec - state_enter_time;

            if (state == STATE_CROSS_LOCK) {
                bool cleared_wings = (state_dur > 0.030 && s.ir[0] < 0.22f && s.ir[3] < 0.22f);
                if (cleared_wings || state_dur > 0.065) {
                    state = STATE_NORMAL_RACE;
                }
            } else if (state == STATE_SHORTCUT_ALIGN) {
                // Tiến lên 42ms để tâm trục bánh xe đặt đúng vào giao điểm chữ X của Vòng tròn 2
                if (state_dur < 0.042) {
                    pwm_left  = 205;
                    pwm_right = 205;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                } else {
                    enterState(STATE_SHORTCUT_TURN, s.elapsed_sec);
                    state_dur = 0.0;
                }
            }

            if (state == STATE_SHORTCUT_TURN) {
                // Bẻ mũi sang nhánh phải (tắt qua Vòng tròn 2 để ra thẳng Dốc cầu)
                if (state_dur < 0.125) {
                    pwm_left  =  230;
                    pwm_right = -135;
                    last_valid_side = 1;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }
                if (s.ir[1] > 0.42f || s.ir[2] > 0.42f || state_dur > 0.36) {
                    shortcut_taken = true;
                    state          = STATE_NORMAL_RACE;
                    float init_err = computeLineError(s.ir);
                    pos_error      = init_err;
                    prev_pos_error = init_err;
                    filtered_error = init_err;
                    filtered_deriv = 0.0f;
                    curve_bias     = 0.0f;
                    last_valid_side = 1;
                } else {
                    pwm_left  =  210;
                    pwm_right = -110;
                    last_valid_side = 1;
                    updateVirtualOdometry(pwm_left, pwm_right, dt);
                    return;
                }
            }
        }

        // 3. TÍNH SAI SỐ & ĐẠO HÀM TÁCH PHA (SPLIT-PHASE ZERO-LAG DERIVATIVE)
        prev_pos_error = pos_error;
        pos_error      = computeLineError(s.ir);

        if (was_all_white && !all_white_out) {
            prev_pos_error = pos_error;
            filtered_error = pos_error;
            filtered_deriv *= 0.25f;
        }
        was_all_white = all_white_out;

        // Lọc siêu nhạy cho P (alpha = 0.82 -> phản xạ tức thì như keo)
        filtered_error = 0.18f * filtered_error + 0.82f * pos_error;

        float raw_deriv = 0.0f;
        if (!all_white_out && dt > 0.0002) {
            float delta_e = pos_error - prev_pos_error;
            // Khử rung nhiễu vi mô trên đường thẳng để xe lướt êm như nhung
            if (fabsf(delta_e) < 0.012f && fabsf(filtered_error) < 0.35f) delta_e = 0.0f;
            raw_deriv = delta_e / (float)dt;
            raw_deriv = clampf(raw_deriv, -240.0f, 240.0f);
        }

        float alpha_d  = clampf((float)(dt / (0.0060 + dt)), 0.18f, 0.55f);
        filtered_deriv = (1.0f - alpha_d) * filtered_deriv + alpha_d * raw_deriv;

        float abs_e = fabsf(filtered_error);
        float abs_d = fabsf(filtered_deriv);

        // 4. BỘ TÍCH PHÂN ĐỘ CONG THÔNG MINH (CURVATURE MEMORY - BÍ QUYẾT BÁM NHƯ KEO!)
        // Khi xe đang ôm cung tròn (cùng dấu sai số), curve_bias tự động gánh lực lái nền,
        // ép sai số về 0.0 (chính giữa vạch). Khi đổi dấu hoặc về tâm, xả ngay tức khắc!
        if (state == STATE_CROSS_LOCK || all_white_out) {
            curve_bias *= 0.85f;
        } else if (filtered_error * curve_bias < -0.05f) {
            // Đổi hướng cua -> Xả ngay lập tức để không bị quá đà!
            curve_bias *= 0.35f;
        } else if (abs_e > 0.30f && abs_e < 3.20f) {
            curve_bias += KI_CURVE_MEMORY * filtered_error * (float)dt;
            curve_bias  = clampf(curve_bias, -65.0f, 65.0f);
        } else {
            curve_bias *= 0.90f;
        }

        // 5. QUỸ ĐẠO TỐC ĐỘ THÍCH ỨNG
        if (abs_e < 0.90f && !all_white_out) {
            straight_time = min2f((float)(straight_time + dt), 0.25f);
        } else if (abs_e > 1.85f || all_white_out) {
            straight_time = 0.0;
        } else {
            straight_time = max2f(0.0f, (float)(straight_time - 1.5f * dt));
        }

        float turbo_ratio = clampf((float)(straight_time / 0.060), 0.0f, 1.0f);
        float peak_base   = lerpf(NOMINAL_BASE_PWM, STRAIGHT_MAX_PWM, turbo_ratio);

        float err_penalty   = CURVE_SLOWDOWN_KE * powf(max2f(0.0f, abs_e - 0.30f), 1.25f);
        float deriv_penalty = CURVE_SLOWDOWN_KD * max2f(0.0f, abs_d - 16.0f);
        float target_base   = clampf(peak_base - err_penalty - deriv_penalty, CORNER_MIN_PWM, STRAIGHT_MAX_PWM);

        if (active_run_time < 0.150) {
            float soft_launch = clampf((float)(active_run_time / 0.150), 0.58f, 1.0f);
            target_base *= soft_launch;
        }

        // 6. TÍNH LỰC LÁI: SUPERGLUE PD + TÁCH PHA DAMPING (CHỐNG VẪY ĐUÔI)
        // Kiểm tra xe đang lệch ra xa tâm (is_diverging) hay đang quay về tâm (is_converging):
        bool is_diverging = (filtered_error * filtered_deriv >= 0.0f);
        float base_kd     = is_diverging ? KD_DIVERGE : KD_CONVERGE;

        float center_softener = (abs_e < 0.35f) ? (0.90f + 0.28f * abs_e) : 1.0f;
        float p_term = (KP_BASE * center_softener) * filtered_error
                     + KP_NONLINEAR * filtered_error * abs_e;

        float effective_kd = base_kd + KD_CORNER_BOOST * clampf(abs_e / 2.40f, 0.0f, 1.25f);
        float d_term       = effective_kd * filtered_deriv;

        float turn_effort  = p_term + d_term + curve_bias;
        if (state == STATE_CROSS_LOCK) {
            target_base = NOMINAL_BASE_PWM;
            turn_effort = clampf(turn_effort * 0.25f, -30.0f, 30.0f);
        } else {
            turn_effort = clampf(turn_effort, -410.0f, 410.0f);
        }

        // 7. PHỐI LỰC VI SAI BẤT ĐỐI XỨNG (KHÓA TRẦN BÁNH NGOÀI + BÁM CUA TỨC THÌ)
        float v_left = 0.0f, v_right = 0.0f;

        bool corner_line_lost = (all_white_out && state == STATE_NORMAL_RACE &&
                                 (!center_locked || lost_line_time >= 0.045));

        if (!has_seen_line) {
            if (off_track_time < 0.45) {
                v_left  = 155.0f;
                v_right = 155.0f;
            }
        } else if (off_track_time > 0.85) {
            v_left  = 0.0f;
            v_right = 0.0f;
        } else if (corner_line_lost) {
            // PIVOT RECOVERY TỨC THÌ: Bánh ngoài đẩy cực mạnh, bánh trong phanh xoay quanh tâm!
            float lost_boost = clampf((float)(lost_line_time / 0.050), 0.0f, 1.0f);
            float outer_pwm  = lerpf(225.0f, 250.0f, lost_boost);
            float inner_pwm  = lerpf(-95.0f, MAX_REVERSE_PWM, lost_boost);

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

            // Khi bánh ngoài chạm trần 255, truyền toàn bộ lực vi sai dư sang ép bánh trong
            // giúp xe bẻ mũi sắc lẹm mà không bao giờ bị trượt rộng (understeer)!
            if (v_left > MAX_PWM_OUT) {
                float excess = v_left - MAX_PWM_OUT;
                v_left   = MAX_PWM_OUT;
                v_right -= 0.96f * excess;
            } else if (v_right > MAX_PWM_OUT) {
                float excess = v_right - MAX_PWM_OUT;
                v_right  = MAX_PWM_OUT;
                v_left  -= 0.96f * excess;
            }

            float rev_allow   = clampf((abs_e - 0.65f) / 1.80f, 0.0f, 1.0f);
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
// 5. DIEU KHIEN PHAN CUNG & BAO VE NGUON PIN
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

static inline int applyN20Deadzone(int pwm) {
    if (pwm == 0) return 0;
    int mag = abs(pwm);
    if (mag >= 250) return (pwm > 0) ? 255 : -255;
    int out = MIN_N20_PWM + ((mag * (255 - MIN_N20_PWM)) / 255);
    out = constrain(out, 0, 255);
    return (pwm > 0) ? out : -out;
}

static inline int slewLimitPWM(int target, int prev, int& zero_hold) {
    if (zero_hold > 0) {
        zero_hold--;
        return 0;
    }

    // Chốt chặn 4ms tại PWM = 0 khi đảo chiều để xả Back-EMF, giữ áp Pin luôn ổn định
    if ((target < 0 && prev > 0) || (target > 0 && prev < 0)) {
        if (abs(prev) > 145) {
            return (prev > 0) ? (prev - 145) : (prev + 145);
        }
        zero_hold = 2;
        return 0;
    }

    int diff = target - prev;
    bool is_increasing_load = (target > prev && prev >= 0) || (target < prev && prev <= 0);

    if (is_increasing_load) {
        if (diff > 48)  return prev + 48;
        if (diff < -48) return prev - 48;
        return target;
    } else {
        if (diff > 165)  return prev + 165;
        if (diff < -165) return prev - 165;
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

        int effective_span = ((ir_black_cal[i] - ir_white_cal[i]) * 86) / 100;
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
        Serial.printf("[t=%5.2fs] Cross=%d Err=%+5.2f Bias=%+5.1f | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, brain.cross_count, brain.filtered_error, brain.curve_bias,
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      pwm_l, pwm_r);
    }
#endif
}
