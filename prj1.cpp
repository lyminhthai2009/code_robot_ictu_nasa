#include <Arduino.h>
#include <math.h>

// ============================================================================
// 1. CAU HINH CHIEN THUAT & THONG SO DUONG DUA (PRJ1 FINAL: DI THANG QUA NGA TU)
// ============================================================================
#define ENABLE_SERIAL_TELEMETRY 1 // 0 = Tat Serial khi chay thi that | 1 = Bat log debug

// --- CAU HINH TOC DO (GIU NGUYEN BAN CHUAN CUA BAN) ---
const float STRAIGHT_MAX_PWM   = 245.0f; // Toc do toi da tren duong thang (230..255)
const float NOMINAL_BASE_PWM   = 205.0f; // Toc do nen khi chay on dinh (190..225)
const float CORNER_MIN_PWM     = 105.0f; // Toc do nen toi thieu o cua gat 90 do / cua U (90..125)
const float MAX_PWM_OUT        = 255.0f; // Tran PWM phan cung
const float MAX_REVERSE_PWM    = -165.0f;// Luc phanh/dao chieu toi da banh trong khi cua gat (-130..-190)

// --- THONG SO BO DIEU KHIEN PD PHI TUYEN 500Hz (dt = 0.002s) ---
const float KP_BASE            = 48.0f;
const float KP_NONLINEAR       = 18.5f;
const float KD_BASE            = 1.85f;
const float KD_CORNER_BOOST    = 0.55f;

// --- THONG SO GIAM TOC THICH UNG THEO DO CONG & DAO HAM ---
const float CURVE_SLOWDOWN_KE  = 34.0f;
const float CURVE_SLOWDOWN_KD  = 1.15f;

// --- THONG SO KHOA NGA TU DI THANG (PRJ1) ---
const float CROSS_WING_TRIG    = 0.36f;  // Nguong chuan hoa IR [0..1] nhan dien canh cham vach ngang
const float CROSS_WING_CLEAR   = 0.26f;  // Nguong xac nhan canh da roi khoi vach ngang (co tre 0.10 voi TRIG)
const float CROSS_SKEW_WINDOW  = 0.030f; // Cua so bat lech pha giua 2 canh khi vao xien / chu X (30ms)
const float CROSS_MIN_LOCK_SEC = 0.018f; // Thoi gian giu khoa toi thieu (18ms)
const float CROSS_MAX_LOCK_SEC = 0.105f; // Thoi gian giu khoa toi da (105ms)
const float CROSS_COOLDOWN_SEC = 0.055f; // Thoi gian nghi chong kich nham mep sau cua cung 1 nga tu (55ms)

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
    STATE_CROSS_LOCK
};

// ============================================================================
// 4. BO DIEU KHIEN TRUNG TAM SIEU TOC (COMPETITION BRAIN 500Hz - PRJ1)
// ============================================================================
class CobotBrain {
public:
    TrackState state = STATE_NORMAL_RACE;

    // Trang thai bo quan sat vi tri vach & PD
    float pos_error          = 0.0f;
    float prev_pos_error     = 0.0f;
    float filtered_error     = 0.0f;
    float filtered_deriv     = 0.0f;
    float outer_peak_memory  = 0.0f;

    int   last_valid_side    = 1;    // -1: Ben trai (s0,s1), +1: Ben phai (s2,s3)
    bool  center_locked      = true;
    bool  was_all_white      = false;
    bool  was_lifted         = false;
    bool  has_seen_line      = false;

    double lost_line_time    = 0.0;
    double off_track_time    = 0.0;
    double straight_time     = 0.0;

    // Trang thai xu ly nga tu PRJ1 (Skew-Tolerant Cross-Lock + Contrast Re-arm)
    int    cross_count          = 0;
    int    cross_confirm_frames = 0;
    int    cross_clear_frames   = 0;
    int    clean_center_frames  = 0;
    bool   cross_rearmed        = false;
    float  wing_l_timer         = 0.0f;
    float  wing_r_timer         = 0.0f;
    float  cross_cooldown       = 0.0f;
    bool   was_bar_saturated    = false;

    double state_enter_time     = 0.0;
    double run_start_time       = 0.0;

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
        cross_clear_frames   = 0;
        clean_center_frames  = 0;
        cross_rearmed        = false;
        wing_l_timer         = 0.0f;
        wing_r_timer         = 0.0f;
        cross_cooldown       = 0.0f;
        was_bar_saturated    = false;

        state_enter_time     = now_sec;
        run_start_time       = now_sec;
    }

    // ========================================================================
    // THUAT TOAN NOI SUY VI TRI VACH (GIU NGUYEN CHUAN THI DAU + CHONG GIAT CANH)
    // ========================================================================
    float computeLineError(const float s[4]) {
        float w[4];
        for (int i = 0; i < 4; i++) {
            float clean = clampf((s[i] - 0.06f) / 0.94f, 0.0f, 1.0f);
            w[i] = clean * (0.75f + 0.25f * clean);
        }

        bool both_wings_active = (w[0] > 0.34f && w[3] > 0.34f);

        if (state == STATE_CROSS_LOCK) {
            w[0] = 0.0f;
            w[3] = 0.0f;
        } else if (both_wings_active) {
            if (w[1] > 0.20f || w[2] > 0.20f) {
                w[0] = 0.0f;
                w[3] = 0.0f;
            } else {
                if (last_valid_side >= 0) w[0] = 0.0f;
                else w[3] = 0.0f;
            }
        } else if (w[1] > 0.55f && w[2] > 0.55f) {
            // PRE-CROSS ANTI-YANK
            if (w[0] > 0.30f && w[3] < 0.15f) {
                last_valid_side = -1;
                w[0] *= 0.35f;
            } else if (w[3] > 0.30f && w[0] < 0.15f) {
                last_valid_side = 1;
                w[3] *= 0.35f;
            }
        }

        float w_sum = w[0] + w[1] + w[2] + w[3];

        if (w_sum < 0.05f) {
            if (state == STATE_CROSS_LOCK) return 0.0f;

            if (center_locked && lost_line_time < 0.012 && fabsf(pos_error) < 0.65f) {
                return pos_error;
            }

            return (last_valid_side >= 0) ? 4.50f : -4.50f;
        }

        float num   = (-3.0f * w[0]) + (-1.0f * w[1]) + (1.0f * w[2]) + (3.0f * w[3]);
        float raw_e = num / w_sum;

        if (state != STATE_CROSS_LOCK && !both_wings_active) {
            if (w[3] > 0.04f && w[2] < 0.12f && w[1] < 0.05f && w[0] < 0.05f) {
                if (w[3] > outer_peak_memory) outer_peak_memory = w[3];
                if (pos_error >= 2.20f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[3]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = 3.00f + 1.45f * decay_ratio;
                    if (extrap_e > raw_e) raw_e = extrap_e;
                }
            } else if (w[0] > 0.04f && w[1] < 0.12f && w[2] < 0.05f && w[3] < 0.05f) {
                if (w[0] > outer_peak_memory) outer_peak_memory = w[0];
                if (pos_error <= -2.20f || outer_peak_memory > 0.45f) {
                    float decay_ratio = clampf((0.65f - w[0]) / 0.60f, 0.0f, 1.0f);
                    float extrap_e    = -3.00f - 1.45f * decay_ratio;
                    if (extrap_e < raw_e) raw_e = extrap_e;
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

        if (state != STATE_CROSS_LOCK && !both_wings_active) {
            if (w[3] > 0.15f || raw_e > 0.18f) {
                last_valid_side = 1;
            } else if (w[0] > 0.15f || raw_e < -0.18f) {
                last_valid_side = -1;
            }
        }

        return clampf(raw_e, -4.50f, 4.50f);
    }

    // ========================================================================
    // BO NHAN DIEN & KHOA NGA TU DI THANG (CONTRAST RE-ARM + SKEW TOLERANT)
    // ========================================================================
    void updateIntersectionStraight(const SensorData& s, double dt, float max_ir, float min_ir) {
        if (wing_l_timer > 0.0f)   wing_l_timer   = max2f(0.0f, wing_l_timer   - (float)dt);
        if (wing_r_timer > 0.0f)   wing_r_timer   = max2f(0.0f, wing_r_timer   - (float)dt);
        if (cross_cooldown > 0.0f) cross_cooldown = max2f(0.0f, cross_cooldown - (float)dt);

        if (!has_seen_line || off_track_time >= 0.04) {
            wing_l_timer         = 0.0f;
            wing_r_timer         = 0.0f;
            cross_confirm_frames = 0;
            clean_center_frames  = 0;
            cross_rearmed        = false;
            return;
        }

        float center_max = max2f(s.ir[1], s.ir[2]);
        bool wings_clearly_low = (s.ir[0] < CROSS_WING_CLEAR && s.ir[3] < CROSS_WING_CLEAR);
        bool wings_contrast_ok = (center_max > 0.65f) &&
                                 (s.ir[0] < center_max - 0.35f) &&
                                 (s.ir[3] < center_max - 0.35f);

        if (state == STATE_NORMAL_RACE) {
            if (max_ir < 0.12f || min_ir > 0.80f) {
                clean_center_frames  = 0;
                cross_rearmed        = false;
                wing_l_timer         = 0.0f;
                wing_r_timer         = 0.0f;
                cross_confirm_frames = 0;
                return;
            }

            if (!cross_rearmed) {
                bool on_clean_line = (wings_clearly_low || wings_contrast_ok) && (center_max > 0.22f);
                if (on_clean_line) {
                    if (clean_center_frames < 20) clean_center_frames++;
                    if (clean_center_frames >= 4 && cross_cooldown <= 0.0f) {
                        cross_rearmed = true;
                    }
                }
                return;
            }

            if (cross_cooldown > 0.0f) return;

            bool center_present = (center_max > 0.24f);
            if (!center_present) {
                wing_l_timer         = 0.0f;
                wing_r_timer         = 0.0f;
                cross_confirm_frames = 0;
                return;
            }

            bool angle_ok  = (fabsf(filtered_error) < 1.35f);
            bool simul_raw = (s.ir[0] > CROSS_WING_TRIG && s.ir[3] > CROSS_WING_TRIG);

            if (s.ir[0] > CROSS_WING_TRIG && (angle_ok || wing_r_timer > 0.0f || simul_raw)) {
                wing_l_timer = CROSS_SKEW_WINDOW;
            }
            if (s.ir[3] > CROSS_WING_TRIG && (angle_ok || wing_l_timer > 0.0f || simul_raw)) {
                wing_r_timer = CROSS_SKEW_WINDOW;
            }

            bool live_wing = (s.ir[0] > 0.30f || s.ir[3] > 0.30f);
            if (wing_l_timer > 0.0f && wing_r_timer > 0.0f && live_wing) {
                cross_confirm_frames++;
            } else {
                cross_confirm_frames = 0;
            }

            if (cross_confirm_frames >= 2) {
                state                = STATE_CROSS_LOCK;
                state_enter_time     = s.elapsed_sec;
                cross_count++;
                cross_rearmed        = false;
                clean_center_frames  = 0;
                cross_confirm_frames = 0;
                cross_clear_frames   = 0;
                wing_l_timer         = 0.0f;
                wing_r_timer         = 0.0f;
                was_bar_saturated    = (s.ir[1] > 0.62f && s.ir[2] > 0.62f);
                center_locked        = true;
            }
        } else if (state == STATE_CROSS_LOCK) {
            double state_dur = s.elapsed_sec - state_enter_time;

            bool both_wings_clear = wings_clearly_low || (wings_contrast_ok && state_dur >= 0.022);
            if (both_wings_clear) cross_clear_frames++;
            else cross_clear_frames = 0;

            bool center_lost = (s.ir[1] < 0.20f && s.ir[2] < 0.20f);

            bool exit_normal  = (state_dur >= CROSS_MIN_LOCK_SEC && cross_clear_frames >= 2);
            bool exit_abort   = (state_dur >= 0.036 && center_lost);
            bool exit_timeout = (state_dur >= CROSS_MAX_LOCK_SEC);

            if (exit_normal || exit_abort || exit_timeout) {
                state               = STATE_NORMAL_RACE;
                cross_cooldown      = CROSS_COOLDOWN_SEC;
                cross_clear_frames  = 0;
                clean_center_frames = 0;
                cross_rearmed       = false;
                wing_l_timer        = 0.0f;
                wing_r_timer        = 0.0f;
                was_bar_saturated   = false;

                float exit_err = computeLineError(s.ir);
                pos_error      = exit_err;
                prev_pos_error = exit_err;
                filtered_error = exit_err;
                filtered_deriv *= 0.25f;
                outer_peak_memory = 0.0f;

#if ENABLE_SERIAL_TELEMETRY
                if (Serial.availableForWrite() > 64) {
                    const char* reason = exit_normal ? "OK" : (exit_abort ? "ABORT" : "TMO");
                    Serial.printf(">>> [CROSS #%d] t=%.2fs dur=%.0fms (%s) exitE=%+.2f IR=[%.2f,%.2f,%.2f,%.2f]\n",
                                  cross_count, s.elapsed_sec, state_dur * 1000.0, reason, exit_err,
                                  s.ir[0], s.ir[1], s.ir[2], s.ir[3]);
                }
#endif
            }
        }
    }

    // ========================================================================
    // VONG LAP TINH TOAN DIEU KHIEN CHINH (500Hz)
    // ========================================================================
    void computeControl(const SensorData& s, double dt, int& pwm_left, int& pwm_right) {
        float max_ir = max4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);
        float min_ir = min4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);

        bool all_white_out  = (max_ir < 0.10f);
        bool in_short_cross = (state == STATE_CROSS_LOCK) && ((s.elapsed_sec - state_enter_time) < 0.095);
        bool all_black_air  = (min_ir > 0.88f) && !in_short_cross;

        if (all_white_out || all_black_air) {
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
            pwm_left  = 0;
            pwm_right = 0;
            return;
        }

        if (all_white_out) {
            lost_line_time += dt;
        } else {
            lost_line_time = 0.0;
        }

        // 2. Cap nhat may trang thai Nga tu PRJ1
        updateIntersectionStraight(s, dt, max_ir, min_ir);

        // 3. TINH SAI SO & BO LOC DAO HAM
        bool bar_saturating_center = (state == STATE_CROSS_LOCK) &&
                                     (s.ir[1] > 0.62f && s.ir[2] > 0.62f);

        if (bar_saturating_center) {
            pos_error         = 0.0f;
            prev_pos_error    = 0.0f;
            filtered_error    = 0.0f;
            filtered_deriv    = 0.0f;
            was_bar_saturated = true;
        } else {
            prev_pos_error = pos_error;
            pos_error      = computeLineError(s.ir);

            if ((was_all_white && !all_white_out) || was_bar_saturated) {
                prev_pos_error    = pos_error;
                filtered_error    = pos_error;
                filtered_deriv   *= 0.25f;
                was_bar_saturated = false;
            }
            was_all_white = all_white_out;

            filtered_error = 0.22f * filtered_error + 0.78f * pos_error;

            float raw_deriv = 0.0f;
            if (!all_white_out && dt > 0.0002) {
                raw_deriv = (pos_error - prev_pos_error) / (float)dt;
                raw_deriv = clampf(raw_deriv, -220.0f, 220.0f);
            }

            float alpha_d  = clampf((float)(dt / (0.0065 + dt)), 0.18f, 0.55f);
            filtered_deriv = (1.0f - alpha_d) * filtered_deriv + alpha_d * raw_deriv;
        }

        float abs_e = fabsf(filtered_error);
        float abs_d = fabsf(filtered_deriv);

        // 4. QUY DAO TOC DO THICH UNG THEO DO CONG + PHANH DU DOAN DAO HAM (GIU NGUYEN)
        if (abs_e < 0.75f && !all_white_out) {
            straight_time = min2f((float)(straight_time + dt), 0.25f);
        } else if (abs_e > 1.35f || all_white_out) {
            straight_time = 0.0;
        } else {
            straight_time = max2f(0.0f, (float)(straight_time - 2.0 * dt));
        }

        float turbo_ratio = clampf((float)(straight_time / 0.12), 0.0f, 1.0f);
        float peak_base   = lerpf(NOMINAL_BASE_PWM, STRAIGHT_MAX_PWM, turbo_ratio);

        float err_penalty   = CURVE_SLOWDOWN_KE * powf(max2f(0.0f, abs_e - 0.15f), 1.30f);
        float deriv_penalty = CURVE_SLOWDOWN_KD * max2f(0.0f, abs_d - 12.0f);
        float target_base   = clampf(peak_base - err_penalty - deriv_penalty, CORNER_MIN_PWM, STRAIGHT_MAX_PWM);

        // 5. TINH LUC LAI PD PHI TUYEN + TANG CUONG GIAM CHAN O CUA (GIU NGUYEN)
        float center_softener = (abs_e < 0.40f) ? (0.88f + 0.30f * abs_e) : 1.0f;
        float p_term = (KP_BASE * center_softener) * filtered_error
                     + KP_NONLINEAR * filtered_error * abs_e;

        float effective_kd = KD_BASE + KD_CORNER_BOOST * clampf(abs_e / 2.50f, 0.0f, 1.2f);
        float d_term       = effective_kd * filtered_deriv;

        float turn_effort  = p_term + d_term;
        if (state == STATE_CROSS_LOCK) {
            target_base = clampf(peak_base - 0.5f * err_penalty, NOMINAL_BASE_PWM, 235.0f);
            if (bar_saturating_center) {
                turn_effort = 0.0f;
            } else {
                turn_effort = clampf(turn_effort, -75.0f, 75.0f);
            }
        } else {
            turn_effort = clampf(turn_effort, -380.0f, 380.0f);
        }

        // 6. PHOI LUC VI SAI BAT DOI XUNG & KHOA CUA MAT VACH (GIU NGUYEN)
        float v_left = 0.0f, v_right = 0.0f;

        bool corner_line_lost = (all_white_out && state == STATE_NORMAL_RACE &&
                                 (!center_locked || lost_line_time >= 0.010));

        if (!has_seen_line) {
            if (off_track_time < 0.45) {
                v_left  = 145.0f;
                v_right = 145.0f;
            }
        } else if (off_track_time > 0.85) {
            v_left  = 0.0f;
            v_right = 0.0f;
        } else if (corner_line_lost) {
            float lost_boost = clampf((float)(lost_line_time / 0.060), 0.0f, 1.0f);
            float outer_pwm  = lerpf(210.0f, 235.0f, lost_boost);
            float inner_pwm  = lerpf(-115.0f, MAX_REVERSE_PWM, lost_boost);

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
                v_right -= 0.85f * excess;
            } else if (v_right > MAX_PWM_OUT) {
                float excess = v_right - MAX_PWM_OUT;
                v_right  = MAX_PWM_OUT;
                v_left  -= 0.85f * excess;
            }

            float rev_allow   = clampf((abs_e - 0.55f) / 1.95f, 0.0f, 1.0f);
            float min_allowed = lerpf(0.0f, MAX_REVERSE_PWM, rev_allow);

            v_left  = clampf(v_left,  min_allowed, MAX_PWM_OUT);
            v_right = clampf(v_right, min_allowed, MAX_PWM_OUT);
        }

        pwm_left  = (int)clampf(v_left,  -MAX_PWM_OUT, MAX_PWM_OUT);
        pwm_right = (int)clampf(v_right, -MAX_PWM_OUT, MAX_PWM_OUT);
    }
};

// ============================================================================
// 5. DIEU KHIEN PHAN CUNG DONG CO & AUTO-CALIB CAM BIEN (GIU NGUYEN 100%)
// ============================================================================
CobotBrain brain;

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

static inline int slewLimitPWM(int target, int prev) {
    int diff = target - prev;
    bool is_accelerating_forward = (target > prev && prev >= 0);
    if (is_accelerating_forward) {
        if (diff > 65) return prev + 65;
        return target;
    } else {
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
// 6. KHOI TAO HE THONG & CALIB TUONG PHAN THONG MINH (GIU NGUYEN 100%)
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

    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200) {
        int r0 = readIRFiltered(PIN_IR[0]);
        int r3 = readIRFiltered(PIN_IR[3]);
        if (r0 < 1500 || r3 < 1500) break;
        delay(8);
    }

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
// 7. VONG LAP THOI GIAN THUC 500Hz
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
        Serial.printf("[t=%5.2fs] St=%d X=%d Err=%+5.2f | Raw=[%4d,%4d,%4d,%4d] Cal0=(%d,%d) Cal1=(%d,%d) | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, (int)brain.state, brain.cross_count,
                      brain.filtered_error,
                      sData.raw_ir[0], sData.raw_ir[1], sData.raw_ir[2], sData.raw_ir[3],
                      ir_white_cal[0], ir_black_cal[0], ir_white_cal[1], ir_black_cal[1],
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      pwm_l, pwm_r);
    }
#endif
}
