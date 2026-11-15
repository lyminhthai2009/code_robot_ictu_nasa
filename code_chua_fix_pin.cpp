#include <Arduino.h>
#include <math.h>

// ============================================================================
// 1. CAU HINH CHIEN THUAT TOI UU CHO PIN 4x AA 1.5V (~6.0V)
// ============================================================================
#define ENABLE_SERIAL_TELEMETRY 0 // 0 = Tat Serial khi chay thi that | 1 = Bat log debug

// --- CAU HINH TOC DO CHUAN CHO PIN 6V (NHANH, MUOT, KHONG SUT AP PIN) ---
// Luu y: Tren pin 6V, PWM = 220 da nhanh tuong duong PWM = 255 khi cam cap USB 4.6V!
// Giu tran 225 giup chừa du địa cho banh ngoai tang toc len 250 khi be cua.
const float STRAIGHT_MAX_PWM   = 225.0f; // Toc do toi da tren duong thang (210..240)
const float NOMINAL_BASE_PWM   = 195.0f; // Toc do nen tieu chuan (180..215)
const float CORNER_MIN_PWM     = 115.0f; // Toc do nen khi om cua gat (100..135)
const float MAX_PWM_OUT        = 250.0f; // Tran PWM phan cung
const float MAX_REVERSE_PWM    = -110.0f;// Gioi han phanh lui vua du xoay xe, KHONG gay sut ap pin!

// --- THONG SO BO DIEU KHIEN PD 500Hz (TRIET TIEU GIAT LAC TREN PIN 6V) ---
const float KP_BASE            = 36.0f;  // He so P giu tam muot, khong giat lac (30..44)
const float KP_NONLINEAR       = 14.5f;  // He so P phi tuyen tang luc be cua khi |e| lon (11..18)
const float KD_BASE            = 2.35f;  // He so D dap tat dao dong ngang ngay lap tuc (1.8..3.0)
const float KD_CORNER_BOOST    = 0.70f;  // Tang D khi vao cua de chong vang duoi

// --- THONG SO GIAM TOC THONG MINH TRONG CUA ---
const float CURVE_SLOWDOWN_KE  = 26.0f;  // Ha toc do nen muot theo do lech vach
const float DIVERGENCE_BRAKE_K = 0.65f;  // Chi phanh khi xe dang LAO RA KHOI VACH (e * de/dt > 0)

// ============================================================================
// 2. CAU HINH CHAN PHAN CUNG ESP32 & DONG CO N20 (GIU NGUYEN PHAN CUNG)
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
const int MIN_N20_PWM        = 30;     // Nguong bu ma sat tinh hop so N20
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
    float ir[4];      // Gia tri chuan hoa [0.0 = Trang, 1.0 = Den]
    int raw_ir[4];    // Gia tri ADC 12-bit [0 .. 4095]
    double elapsed_sec;
};

// ============================================================================
// 4. BO DIEU KHIEN TRUNG TAM CHONG XOAY VONG & CHONG GIAT LAC (500Hz)
// ============================================================================
class CobotBrain {
public:
    float pos_error         = 0.0f;
    float prev_pos_error    = 0.0f;
    float filtered_error    = 0.0f;
    float filtered_deriv    = 0.0f;
    float err_history[3]    = {0.0f, 0.0f, 0.0f}; // Bo nho 3 mau de tinh dao ham sieu sach

    float peak_mem_left     = 0.0f;
    float peak_mem_right    = 0.0f;

    int   last_valid_side   = 1;     // -1: Ben trai (s0,s1), +1: Ben phai (s2,s3)
    bool  center_locked     = true;
    bool  in_pivot_search   = false; // Co bao dang trong trang thai xoay tim vach
    bool  was_lifted        = false;
    bool  has_seen_line     = false;

    double lost_line_time   = 0.0;
    double line_hold_time   = 0.0;   // Thoi gian nhin thay vach lien tuc (chong reset gia khi dang xoay)
    double off_track_time   = 0.0;
    double straight_time    = 0.0;

    CobotBrain() { reset(); }

    void reset() {
        pos_error         = 0.0f;
        prev_pos_error    = 0.0f;
        filtered_error    = 0.0f;
        filtered_deriv    = 0.0f;
        err_history[0] = err_history[1] = err_history[2] = 0.0f;

        peak_mem_left     = 0.0f;
        peak_mem_right    = 0.0f;

        last_valid_side   = 1;
        center_locked     = true;
        in_pivot_search   = false;
        was_lifted        = false;
        has_seen_line     = false;

        lost_line_time    = 0.0;
        line_hold_time    = 0.0;
        off_track_time    = 0.0;
        straight_time     = 0.0;
    }

    // ========================================================================
    // NOI SUY VI TRI VACH CHINH XAC CAO [-4.50 .. +4.50]
    // ========================================================================
    float computeLineError(const float s[4], double dt) {
        float w[4];
        for (int i = 0; i < 4; i++) {
            // Nguong cat nen trang 0.08 de mien nhiem voi nhieu anh sang san
            float clean = clampf((s[i] - 0.08f) / 0.92f, 0.0f, 1.0f);
            w[i] = clean * (0.75f + 0.25f * clean);
        }

        float mem_decay = clampf((float)(dt * 3.5), 0.0f, 0.2f);
        peak_mem_left   = max2f(0.0f, peak_mem_left  - mem_decay);
        peak_mem_right  = max2f(0.0f, peak_mem_right - mem_decay);

        // Chong nhieu khi di qua nga tu / vach ngang
        bool both_wings_active = (w[0] > 0.40f && w[3] > 0.40f);
        if (both_wings_active) {
            if ((w[1] > 0.30f || w[2] > 0.30f) && fabsf(pos_error) < 1.25f) {
                w[0] = 0.0f;
                w[3] = 0.0f;
            } else {
                if (last_valid_side >= 0) w[0] = 0.0f;
                else                      w[3] = 0.0f;
            }
        }

        float w_sum = w[0] + w[1] + w[2] + w[3];

        // Khi ca 4 mat deu trang (mat vach)
        if (w_sum < 0.06f) {
            if (center_locked && lost_line_time < 0.014 && fabsf(pos_error) < 0.60f) {
                return pos_error;
            }
            return (last_valid_side >= 0) ? 4.50f : -4.50f;
        }

        // Tinh trong tam (Centroid)
        float num   = (-3.0f * w[0]) + (-1.0f * w[1]) + (1.0f * w[2]) + (3.0f * w[3]);
        float raw_e = num / w_sum;

        // Ngoai suy suon ngoai doc lap cho mat Phai (s3) va mat Trai (s0)
        if (!both_wings_active) {
            if (w[3] > 0.05f && w[2] < 0.14f && w[1] < 0.06f && w[0] < 0.06f) {
                if (w[3] > peak_mem_right) peak_mem_right = w[3];
                if (pos_error >= 1.90f || peak_mem_right > 0.42f) {
                    float ref_peak    = max2f(0.80f, peak_mem_right);
                    float decay_ratio = clampf((ref_peak - w[3]) / (ref_peak - 0.05f), 0.0f, 1.0f);
                    float extrap_e    = 3.00f + 1.45f * decay_ratio;
                    if (extrap_e > raw_e) raw_e = extrap_e;
                }
            } else if (w[0] > 0.05f && w[1] < 0.14f && w[2] < 0.06f && w[3] < 0.06f) {
                if (w[0] > peak_mem_left) peak_mem_left = w[0];
                if (pos_error <= -1.90f || peak_mem_left > 0.42f) {
                    float ref_peak    = max2f(0.80f, peak_mem_left);
                    float decay_ratio = clampf((ref_peak - w[0]) / (ref_peak - 0.05f), 0.0f, 1.0f);
                    float extrap_e    = -3.00f - 1.45f * decay_ratio;
                    if (extrap_e < raw_e) raw_e = extrap_e;
                }
            }

            if (w[1] > 0.28f || w[2] > 0.28f) {
                if (raw_e < 0.6f)  peak_mem_right = 0.0f;
                if (raw_e > -0.6f) peak_mem_left  = 0.0f;
            }
        }

        // Cap nhat trang thai khoa tam
        if ((w[1] > 0.25f || w[2] > 0.25f) && w[0] < 0.16f && w[3] < 0.16f && fabsf(raw_e) < 0.85f) {
            center_locked = true;
        } else if (fabsf(raw_e) > 1.15f || w[0] > 0.30f || w[3] > 0.30f) {
            center_locked = false;
        }

        // QUAN TRONG: Chi cap nhat last_valid_side khi KHONG bi xoay tim vach hoac khi vach da vao ro!
        // Giup xe khong bi dao chieu xoay lung tung khi quet ngang qua vach.
        if (!both_wings_active && !in_pivot_search) {
            if (w[3] > 0.22f || raw_e > 0.22f) {
                last_valid_side = 1;
            } else if (w[0] > 0.22f || raw_e < -0.22f) {
                last_valid_side = -1;
            }
        }

        return clampf(raw_e, -4.50f, 4.50f);
    }

    // ========================================================================
    // VONG LAP TINH TOAN DIEU KHIEN CHINH (500Hz, dt = 0.002s)
    // ========================================================================
    void computeControl(const SensorData& s, double dt, int& pwm_left, int& pwm_right) {
        float max_ir = max4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);
        float min_ir = min4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);

        bool all_white_out = (max_ir < 0.12f);
        bool all_black_air = (min_ir > 0.88f);

        // 1. CHONG LOI QUAY VONG VONG VO TAN (ANTI-INFINITE SPIN GUARD):
        // Chi reset off_track_time khi xe THUC SU bam lai duoc vach lien tuc >= 35ms!
        // Neu xe dang quay vong vong ma chi quet ngang qua vạch trong vài ms, off_track_time
        // van tiep tuc tang va se ngat dong co an toan sau 0.75 giay!
        if (all_white_out || all_black_air) {
            line_hold_time = 0.0;
            off_track_time += dt;
            if (off_track_time > 0.75) was_lifted = true;
        } else {
            line_hold_time += dt;
            if (was_lifted) {
                // Khi dat xe xuong vach tro lai va giu on dinh >= 60ms -> tu dong chay tiep
                if (line_hold_time >= 0.060 && (s.ir[1] > 0.35f || s.ir[2] > 0.35f) && min_ir < 0.45f) {
                    reset();
                    has_seen_line = true;
                }
            } else {
                if (line_hold_time >= 0.020 && max_ir >= 0.25f && min_ir < 0.75f) {
                    has_seen_line = true;
                }
                // Phai nhin thay vach lien tuc >= 35ms moi xac nhan da tren vach (xoa off_track_time)
                if (line_hold_time >= 0.035) {
                    off_track_time = 0.0;
                }
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

        // 2. TINH SAI SO & DAO HAM CUA SO 3 MAU (CHONG GIAT LAC TREN DUONG THANG)
        float raw_error = computeLineError(s.ir, dt);

        // Khi vua thoat khoi trang thai xoay tim vach (bat lai duoc vach)
        if (in_pivot_search && !all_white_out) {
            // Neu vach da vao duoc mat cam bien -> dong bo bo loc ngay de khong giat nguoc
            filtered_error = raw_error;
            err_history[0] = err_history[1] = err_history[2] = raw_error;
            // Tao xung phanh dao ham nguoc chieu xoay de HAM QUAN TINH XOAY cua xe ngay lap tuc!
            filtered_deriv = (last_valid_side >= 0) ? -45.0f : 45.0f;
            if (fabsf(raw_error) < 2.20f) {
                in_pivot_search = false;
            }
        }

        filtered_error = 0.30f * filtered_error + 0.70f * raw_error;
        pos_error      = raw_error;

        // Tinh dao ham qua cua so 2 chu ky (2*dt = 4ms) giup triet tieu hoàn toàn nhieu rung ADC
        float raw_deriv = 0.0f;
        if (!all_white_out && dt > 0.0002) {
            raw_deriv = (filtered_error - err_history[1]) / (float)(2.0 * dt);
            raw_deriv = clampf(raw_deriv, -180.0f, 180.0f);
        }
        err_history[2] = err_history[1];
        err_history[1] = err_history[0];
        err_history[0] = filtered_error;

        // Loc muot khau D
        filtered_deriv = 0.68f * filtered_deriv + 0.32f * raw_deriv;

        float abs_e = fabsf(filtered_error);

        // 3. QUY DAO TOC DO THICH UNG TREN PIN 6V
        if (abs_e < 0.80f && !all_white_out) {
            straight_time = min2f((float)(straight_time + dt), 0.20f);
        } else if (abs_e > 1.50f || all_white_out) {
            straight_time = 0.0;
        } else {
            straight_time = max2f(0.0f, (float)(straight_time - dt));
        }

        float turbo_ratio  = clampf((float)(straight_time / 0.10), 0.0f, 1.0f);
        float peak_base    = lerpf(NOMINAL_BASE_PWM, STRAIGHT_MAX_PWM, turbo_ratio);

        // Chi phanh khi xe dang VANG RA KHOI VACH (outward_rate > 0)
        float outward_rate = (filtered_error >= 0.0f) ? filtered_deriv : -filtered_deriv;
        float div_brake    = DIVERGENCE_BRAKE_K * max2f(0.0f, outward_rate - 10.0f);
        float err_penalty  = CURVE_SLOWDOWN_KE * powf(max2f(0.0f, abs_e - 0.25f), 1.25f);

        float target_base  = clampf(peak_base - err_penalty - div_brake, CORNER_MIN_PWM, STRAIGHT_MAX_PWM);

        // 4. LUAT DIEU KHIEN PD PHI TUYEN (MUOT O TAM, MANH O CUA)
        // Lam diu 15% do nhay o sat tam (|e| < 0.35) de xe chay thang khong bao gio lac dau!
        float center_smooth = (abs_e < 0.35f) ? (0.85f + 0.42f * abs_e) : 1.0f;
        float p_term = (KP_BASE * center_smooth) * filtered_error
                     + KP_NONLINEAR * filtered_error * abs_e;

        float effective_kd = KD_BASE + KD_CORNER_BOOST * clampf(abs_e / 2.50f, 0.0f, 1.20f);
        float d_term       = effective_kd * filtered_deriv;

        float turn_effort  = clampf(p_term + d_term, -320.0f, 320.0f);

        // 5. PHOI LUC 2 BANH XE & XOAY TIM VACH KIEM SOAT (CONTROLLED PIVOT)
        float v_left = 0.0f, v_right = 0.0f;

        bool corner_line_lost = (all_white_out && (!center_locked || lost_line_time >= 0.012));

        if (!has_seen_line) {
            // Khi moi dat xuong san, tien thang nhe de bat vach
            if (off_track_time < 0.45) {
                v_left  = 135.0f;
                v_right = 135.0f;
            }
        } else if (off_track_time > 0.75) {
            v_left  = 0.0f;
            v_right = 0.0f;
        } else if (corner_line_lost) {
            in_pivot_search = true;
            // XOAY TIM VACH KIEM SOAT TREN PIN 6V:
            // Khong dung +255/-195 vi tren pin 6V xe se xoay nhu chong chong va vot qua vach!
            // Dung +175 / -85 giup xe xoay quanh truc banh trong cuc gon, khong sut ap pin,
            // va bat dinh lai vach ngay lap tuc khi mat cam bien cham vach!
            float lost_boost = clampf((float)(lost_line_time / 0.050), 0.0f, 1.0f);
            float outer_pwm  = lerpf(165.0f, 185.0f, lost_boost);
            float inner_pwm  = lerpf(-65.0f, -95.0f, lost_boost);

            if (last_valid_side >= 0) {
                v_left  = outer_pwm;
                v_right = inner_pwm;
            } else {
                v_left  = inner_pwm;
                v_right = outer_pwm;
            }
        } else {
            // PHOI LUC VI SAI DOI XUNG + CHUYEN TAI BANH TRONG KHI VAO CUA GAT:
            v_left  = target_base + turn_effort;
            v_right = target_base - turn_effort;

            if (v_left > MAX_PWM_OUT) {
                float excess = v_left - MAX_PWM_OUT;
                v_left   = MAX_PWM_OUT;
                v_right -= 0.90f * excess;
            } else if (v_right > MAX_PWM_OUT) {
                float excess = v_right - MAX_PWM_OUT;
                v_right  = MAX_PWM_OUT;
                v_left  -= 0.90f * excess;
            }

            // Chi cho phep banh trong phanh dao chieu khi |e| > 0.85 (cua that su)
            // Tren duong thang (|e| <= 0.85), banh trong KHONG BAO GIO bi dao chieu -> Chay cuc muot!
            float rev_allow   = clampf((abs_e - 0.85f) / 1.65f, 0.0f, 1.0f);
            float min_allowed = lerpf(0.0f, MAX_REVERSE_PWM, rev_allow);

            v_left  = clampf(v_left,  min_allowed, MAX_PWM_OUT);
            v_right = clampf(v_right, min_allowed, MAX_PWM_OUT);
        }

        pwm_left  = (int)clampf(v_left,  -MAX_PWM_OUT, MAX_PWM_OUT);
        pwm_right = (int)clampf(v_right, -MAX_PWM_OUT, MAX_PWM_OUT);
    }
};

// ============================================================================
// 5. DIEU KHIEN PHAN CUNG DONG CO (5kHz PWM) & DOC ADC CHONG NHIEU MUX
// ============================================================================
CobotBrain brain;

int ir_white_cal[4] = {300, 300, 300, 300};
int ir_black_cal[4] = {2200, 2200, 2200, 2200};

unsigned long startMicros         = 0;
unsigned long lastStepMicros      = 0;
unsigned long lastTelemetryMillis = 0;
uint32_t      loopFrameCount      = 0;
int prev_hw_pwm_l = 0, prev_hw_pwm_r = 0;

void setupMotorsHardware() {
    // Su dung tan so 5000Hz tuong thich hoan hao voi moi IC cau H (DRV8833, MX1508, L9110S)
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

    int out;
    if (mag < 10) {
        out = (mag * MIN_N20_PWM) / 10;
    } else {
        out = MIN_N20_PWM + (((mag - 10) * (255 - MIN_N20_PWM)) / (255 - 10));
    }
    out = constrain(out, 0, 255);
    return (pwm > 0) ? out : -out;
}

// Bo gioi han gia toc chong sut ap pin 4x AA (Anti-Brownout Slew Limiter):
// Ngan chan dong khoi dong nguoc (Inrush current spike) lam tut ap pin AA dot ngot!
static inline int slewLimitPWM(int target, int prev) {
    int diff = target - prev;
    bool is_reversing_sign = ((target > 0 && prev < 0) || (target < 0 && prev > 0));

    if (is_reversing_sign) {
        // Khi dao chieu qua diem 0, gioi han buoc nhay 85 PWM / 2ms de bao ve ap pin AA
        if (diff > 85)  return prev + 85;
        if (diff < -85) return prev - 85;
        return target;
    }

    bool is_accelerating_forward = (target > prev && prev >= 0);
    if (is_accelerating_forward) {
        if (diff > 50) return prev + 50;
        return target;
    } else {
        if (diff > 120)  return prev + 120;
        if (diff < -120) return prev - 120;
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

// Doc ADC kem XA TU MUX (Dummy Read) + Loc Trung vi 3 mau -> Xoa sach nhieu giat lac!
static inline int readIRFiltered(int pin) {
    (void)analogRead(pin); // BAT BUOC: Xa dien tich kenh ADC truoc do tren ESP32
    int a = analogRead(pin);
    int b = analogRead(pin);
    int c = analogRead(pin);
    int hi = (a > b) ? a : b;
    int lo = (a < b) ? a : b;
    return (c > hi) ? hi : ((c < lo) ? lo : c);
}

// ============================================================================
// 6. KHOI TAO AN TOAN KHI BAT CONG TAC PIN & CALIB CHUAN XAC
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

    // BUOC 1 (QUAN TRONG KHI DUNG PIN):
    // Cho bat buoc 900ms sau khi gat cong tac nguon de tay nguoi dung buong han khoi xe
    // va dat xe nam im tren mat ban!
    delay(900);

    // BUOC 2: Kiem tra xe da nam im tren san chua (phai on dinh 5 lan lien tiep)
    unsigned long settleStart = millis();
    int prev_calib[4] = {readIRFiltered(PIN_IR[0]), readIRFiltered(PIN_IR[1]),
                         readIRFiltered(PIN_IR[2]), readIRFiltered(PIN_IR[3])};
    int stable_count = 0;
    while (millis() - settleStart < 1500) {
        delay(20);
        int max_diff = 0;
        int min_val  = 4095;
        for (int i = 0; i < 4; i++) {
            int cur = readIRFiltered(PIN_IR[i]);
            int d = abs(cur - prev_calib[i]);
            if (d > max_diff) max_diff = d;
            if (cur < min_val) min_val = cur;
            prev_calib[i] = cur;
        }
        // Xe phai nam tren san (co it nhat 1 mat nhin thay nen trang < 1300) va khong rung (< 60)
        if (max_diff < 60 && min_val < 1300) {
            stable_count++;
            if (stable_count >= 5) break; // Da nam im tren san 100ms liên tục!
        } else {
            stable_count = 0;
        }
    }

    // BUOC 3: Do mau Calib tai vi tri xuat phat
    int min_seen[4] = {4095, 4095, 4095, 4095};
    int max_seen[4] = {0, 0, 0, 0};
    for (int sample = 0; sample < 25; sample++) {
        for (int i = 0; i < 4; i++) {
            int v = readIRFiltered(PIN_IR[i]);
            if (v < min_seen[i]) min_seen[i] = v;
            if (v > max_seen[i]) max_seen[i] = v;
        }
        delay(3);
    }

    int global_min = min(min(min_seen[0], min_seen[1]), min(min_seen[2], min_seen[3]));
    int global_max = max(max(max_seen[0], max_seen[1]), max(max_seen[2], max_seen[3]));

    if (global_min > 1100) global_min = 300;
    if (global_max < global_min + 600 || global_max > 3800) {
        global_max = constrain(global_min + 1400, 1300, 2800);
    }

    for (int i = 0; i < 4; i++) {
        if (min_seen[i] < global_min + 200) {
            ir_white_cal[i] = min_seen[i];
        } else {
            ir_white_cal[i] = global_min;
        }

        if (max_seen[i] > ir_white_cal[i] + 600 && max_seen[i] < 3800) {
            ir_black_cal[i] = max_seen[i];
        } else {
            ir_black_cal[i] = max(global_max, ir_white_cal[i] + 1100);
        }
    }

    brain.reset();
    startMicros    = micros();
    lastStepMicros = startMicros;
}

// ============================================================================
// 7. VONG LAP THOI GIAN THUC 500Hz (CHU KY 2000 MICROSECONDS)
// ============================================================================
void loop() {
    unsigned long nowMicros = micros();
    unsigned long diffMicros = nowMicros - lastStepMicros;
    if (diffMicros < 2000) return; // Chu ky chuan 500Hz (2.0ms)
    lastStepMicros = nowMicros;

    double dt = diffMicros * 1e-6;
    if (dt > 0.025) dt = 0.002;

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

    int frame_span = frame_max_raw - frame_min_raw;
    bool valid_contrast = (frame_span > 450) && (frame_min_raw < 1200) && (frame_max_raw < 3850);
    bool slow_tick      = ((loopFrameCount & 0x0F) == 0); // Moi 16 chu ky (~32ms)

    // 2. Cap nhat Auto-Calib CHONG SUT AP PIN (Anti-Sag Normalization):
    // Khi motor chay tren pin AA, ap pin sut nhe lam gia tri den thap hon luc dung im.
    // Ta ha nhanh ir_black_cal xuong theo dinh thuc te va lay nguong bao hoa 78% bien do!
    for (int i = 0; i < 4; i++) {
        int raw = sData.raw_ir[i];

        if (valid_contrast && brain.off_track_time < 0.04 && !brain.in_pivot_search) {
            if (raw > 20 && raw < ir_white_cal[i]) {
                ir_white_cal[i] = (ir_white_cal[i] * 3 + raw) / 4;
            } else if (slow_tick && raw < ir_white_cal[i] + 220 && ir_white_cal[i] < 1000) {
                ir_white_cal[i] += 2;
            }

            if (raw > ir_white_cal[i] + 550 && raw < 3700) {
                if (raw > ir_black_cal[i]) {
                    ir_black_cal[i] = (ir_black_cal[i] * 3 + raw) / 4;
                }
            }
        }

        // Ha nhanh dinh black_cal neu luc dau bi calib cao do chua chay motor
        int min_valid_black = ir_white_cal[i] + 600;
        if (slow_tick && ir_black_cal[i] > min_valid_black + 200) {
            ir_black_cal[i] -= 8;
        }
        if (ir_black_cal[i] < min_valid_black) {
            ir_black_cal[i] = min_valid_black;
        }

        // Lay 78% bien do den-trang lam muc 1.0 de du pin AA co sut ap khi motor keo tai,
        // vach den van luon dat do nhay 1.00 tuyet doi!
        int effective_span = ((ir_black_cal[i] - ir_white_cal[i]) * 78) / 100;
        if (effective_span < 450) effective_span = 450;

        float norm = (float)(raw - ir_white_cal[i]) / (float)effective_span;
        sData.ir[i] = clampf(norm, 0.0f, 1.0f);
    }

    // 3. Tinh toan luat dieu khien va xuat xung PWM
    int pwm_l = 0, pwm_r = 0;
    brain.computeControl(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);

#if ENABLE_SERIAL_TELEMETRY
    unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMillis >= 100 && Serial.availableForWrite() > 64) {
        lastTelemetryMillis = nowMs;
        Serial.printf("[t=%5.2fs] Err=%+5.2f dE=%+6.1f | Raw=[%4d,%4d,%4d,%4d] | IR=[%.2f,%.2f,%.2f,%.2f] | PWM=(%+4d,%+4d)\n",
                      sData.elapsed_sec, brain.filtered_error, brain.filtered_deriv,
                      sData.raw_ir[0], sData.raw_ir[1], sData.raw_ir[2], sData.raw_ir[3],
                      sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                      pwm_l, pwm_r);
    }
#endif
}
