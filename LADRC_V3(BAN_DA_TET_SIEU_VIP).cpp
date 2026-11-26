/*
 * ESP32-C3 4-IR Line Follower — 2nd-Order ESO LADRC + Fast-Decay 1:1 Mixer + 100Hz Blackbox
 * Tuned directly from 27.9s Blackbox telemetry (COBOT KM-11-102)
 *
 * Arduino IDE Settings:
 * - Board:            "ESP32C3 Dev Module" (Arduino ESP32 Core 3.x)
 * - USB CDC On Boot:  "Enabled"
 * - CPU Frequency:    "80MHz"
 * - Partition Scheme: "Default 4MB with spiffs"
 */

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <cmath>
#include <cstdint>

// ============================================================================
// 0. PINOUT CONTRACT (Verified on COBOT KM-11-102 + KMM-11-100)
// ============================================================================

static constexpr uint8_t PIN_IR_S1 = 0;       // ADC1_CH0 (Left outer,  -20.00 mm)
static constexpr uint8_t PIN_IR_S2 = 1;       // ADC1_CH1 (Left inner,  -11.25 mm)
static constexpr uint8_t PIN_IR_S3 = 3;       // ADC1_CH3 (Right inner, +11.25 mm)
static constexpr uint8_t PIN_IR_S4 = 4;       // ADC1_CH4 (Right outer, +20.00 mm)
static constexpr uint8_t PIN_IR_ON = 21;      // IR emitter MOSFET gate (HIGH = ON)
static constexpr uint8_t PIN_UNUSED_LED = 20; // Unused white-LED MOSFET gate (keep LOW)
static constexpr uint8_t PIN_BOOT_BTN = 9;    // Optional active-LOW button (internal pull-up)

static constexpr uint8_t PIN_L_A = 2;  // Left Motor INA1
static constexpr uint8_t PIN_L_B = 10; // Left Motor INB1 (has external 10k pull-up R6!)
static constexpr uint8_t PIN_R_A = 6;  // Right Motor INB2
static constexpr uint8_t PIN_R_B = 5;  // Right Motor INA2

// ============================================================================
// 1. LOOP TIMING, POLARITY & MOTOR DRIVE CONFIG (SAFE FAST-DECAY @ 2 kHz)
// ============================================================================

static constexpr uint32_t LOOP_FREQ_HZ = 500;
static constexpr float DT_SEC = 1.0f / static_cast<float>(LOOP_FREQ_HZ);
static constexpr uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_FREQ_HZ;

static constexpr bool SWAP_MOTORS_LR = false;
static constexpr bool INVERT_LEFT_MOTOR = false;
static constexpr bool INVERT_RIGHT_MOTOR = false;
static constexpr float RIGHT_MOTOR_TRIM = 1.00f;

// PURE FAST-DECAY (A=duty, B=0) @ 2 kHz: Mat cau H, khong sut ap 4x AA.
static constexpr bool ENABLE_SLOW_DECAY_BRAKE = false;
static constexpr bool ENABLE_ZERO_CMD_STATIC_BRAKE = false;

static constexpr uint32_t PWM_FREQ_HZ = 2000;
static constexpr uint8_t PWM_RES_BITS = 8;
static constexpr uint32_t PWM_HW_MAX = 255;
static constexpr float PWM_MAX = 255.0f;

static constexpr bool IR_WHITE_IS_HIGH = true;
static constexpr float INIT_WHITE_ADC = 4095.0f;
static constexpr float INIT_BLACK_ADC = 1200.0f;
static constexpr bool MIRROR_SENSORS_LR = false;

// ============================================================================
// 2. LINEARIZED 4-IR TRAPEZOID SENSOR GEOMETRY
// ============================================================================

static constexpr float SENSOR_POS_NORM[4] = {-1.0000f, -0.5625f, 0.5625f, 1.0000f};
static constexpr float SENSOR_DENOM_FLOOR = 0.95f;
static constexpr float SENSOR_OUTER_DENOM_FLOOR = 0.15f;
static constexpr float SENSOR_INNER_POS = 0.5625f;
static constexpr float SENSOR_CENTER_SLOPE = 0.65f;
static constexpr float SENSOR_OUTER_EXT = 0.35f;
static constexpr float SENSOR_MAX_POS = 1.35f;

// ============================================================================
// 3. PER-SENSOR ONLINE CALIBRATION & OBSTACLE THRESHOLDS
// ============================================================================

static constexpr float MIN_CAL_SPAN = 450.0f;
static constexpr float CAL_WHITE_UP_ALPHA = 0.16f;
static constexpr float CAL_WHITE_TRIM_ALPHA = 0.010f;
static constexpr float CAL_BLACK_DN_ALPHA = 0.14f;
static constexpr uint32_t CAL_WHITE_GLARE_TICKS = 2;

static constexpr float BLACK_NOISE_FLOOR = 0.05f;
static constexpr float BLACK_SAT_CEIL = 0.94f;
static constexpr float BG_SUBTRACT_LO = 0.45f;
static constexpr float BG_SUBTRACT_HI = 0.75f;
static constexpr float LINE_PRESENT_SUM = 0.18f;

static constexpr float CROSSING_MIN_ALL_BLACK = 0.74f;
static constexpr float CROSSING_MIN_SUM = 3.25f;
static constexpr float CROSSING_MAX_SPREAD_RATIO = 0.22f;
static constexpr float CROSSING_LATCH_MIN_BLACK = 0.76f;
static constexpr float CROSSING_LATCH_MIN_SUM = 3.30f;
static constexpr float CROSSING_LATCH_MAX_SPREAD = 0.21f;
static constexpr float CROSSING_LATCH_CURR_SUM = 2.38f;
static constexpr uint32_t CROSSING_LATCH_WINDOW_MS = 24;
static constexpr uint32_t CROSSING_LATCH_MIN_TICKS = 3;

static constexpr float AIR_MIN_ALL_BLACK = 0.82f;
static constexpr float AIR_MAX_SPREAD_RATIO = 0.14f;
static constexpr float AIR_MAX_RAW_ADC = 1200.0f;

static constexpr uint32_t CROSS_CONFIRM_MS = 6;
static constexpr uint32_t CROSS_COOLDOWN_MS = 450;
static constexpr uint32_t MAX_CROSS_LOCK_MS = 120;

enum class CrossingAction : uint8_t {
    Straight,
    ShortcutRight,
    ShortcutLeft,
};

static constexpr CrossingAction CROSSING_PLAN[6] = {
    CrossingAction::Straight,
    CrossingAction::Straight,
    CrossingAction::ShortcutRight,
    CrossingAction::Straight,
    CrossingAction::Straight,
    CrossingAction::Straight,
};

// Re nga 4 (Shortcut): cho mat trong bat line -> dap thang giua tam khong lac
static constexpr float SHORTCUT_BRAKE_OUTER_PWM = 185.0f;
static constexpr uint32_t SHORTCUT_BRAKE_MS = 24;
static constexpr float SHORTCUT_OUTER_PWM = 220.0f;
static constexpr float SHORTCUT_INNER_REV_PWM = 125.0f;
static constexpr float SHORTCUT_COUNTER_BRAKE_PWM = 240.0f;
static constexpr uint32_t SHORTCUT_COUNTER_BRAKE_MIN_MS = 20;
static constexpr uint32_t SHORTCUT_COUNTER_BRAKE_MAX_MS = 70;
static constexpr float SHORTCUT_ANTI_PLUG_TH = 120.0f;
static constexpr float SHORTCUT_CURRENT_BUDGET = 195.0f;
static constexpr uint32_t SHORTCUT_LEAVE_MIN_MS = 30;
static constexpr uint32_t SHORTCUT_LEAVE_MAX_MS = 340;
static constexpr uint32_t SHORTCUT_ACQUIRE_MAX_MS = 420;

// Fix qua net dut (dashed line): cho phep Straddle giu thang muot ma thay vi nhay LST
static constexpr uint32_t LIFTED_STOP_MS = 240;
static constexpr float STRADDLE_POS_LIMIT = 0.58f;
static constexpr float STRADDLE_VEL_LIMIT = 16.0f;
static constexpr uint32_t OUTER_RECENT_LOCK_MS = 35;
static constexpr uint32_t MAX_STRADDLE_MS = 500;
static constexpr uint32_t LOST_SAFETY_STOP_MS = 2000;

// ============================================================================
// 4. 2ND-ORDER ESO LADRC PROFILES (FIXED Z3 HEADROOM -> NO FAKE Z2 IN CURVES)
// ============================================================================

struct TuningProfile {
    float v_max;
    float v_curve_min;
    float max_wheel_pwm;
    float v_crossing;
    float v_straddle;
    float wc;
    float wo;
    float b0;
    float zeta;
    float max_z3_pwm;
    float wing_boost_slope;
    float curve_entry_boost;
    float max_steer_pwm;
    float max_reverse_tracking;
    float recovery_outer_pwm;
    float recovery_inner_pwm;
};

static constexpr TuningProfile PROFILE_SAFE = {
    245.0f, 225.0f, 255.0f, 245.0f, 245.0f,
    23.0f,  45.0f,  8.8f,   1.45f,  95.0f,
    0.30f,  0.08f,  255.0f, 0.0f,   235.0f, -35.0f,
};

static constexpr TuningProfile PROFILE_RACE = {
    255.0f, 255.0f, 255.0f, 255.0f, 255.0f,
    26.0f,  50.0f,  8.2f,   1.42f,  115.0f,
    0.35f,  0.10f,  255.0f, 20.0f,  250.0f, -45.0f,
};

// max_z3_pwm = 135.0f (giai phong tran z3 -> xoa bo loi z2 sai dau trong cua dai!)
static constexpr TuningProfile PROFILE_TURBO = {
    255.0f, 255.0f, 255.0f, 255.0f, 255.0f,
    27.5f,  50.0f,  7.7f,   1.48f,  92.0f,  // Điểm ngọt: wo=50, zeta=1.48, max_z3=92 -> cua cực mượt
    0.36f,  0.06f,  255.0f, 0.0f,   255.0f, -60.0f, // max_reverse_tracking=0.0f -> triệt tiêu 100% cú giật lùi trong cua!
};

static constexpr TuningProfile ACTIVE_PROFILE = PROFILE_TURBO;

static constexpr float LADRC_E_DEADBAND = 0.045f;
static constexpr float LADRC_DE_DEADBAND = 2.5f;

// ============================================================================
// 5. 4x AA ZERO-BROWNOUT POWER SHIELD (SAFE FOR H-BRIDGE)
// ============================================================================

static constexpr uint32_t SOFT_START_MS = 140;
static constexpr float MOTOR_TAU_CCM_S = 0.048f;
static constexpr float MOTOR_TAU_DRIVE_S = 0.055f;
static constexpr float MOTOR_TAU_COAST_S = 0.095f;

static constexpr float COMMON_ACCEL_PWM_PER_SEC = 2600.0f;
static constexpr float COMMON_BRAKE_PWM_PER_SEC = 5500.0f;

static constexpr float BATTERY_CURRENT_BUDGET = 380.0f;
static constexpr float MAX_TOTAL_PWM = 510.0f;
static constexpr float ANTI_PLUG_OMEGA_TH = 45.0f;
static constexpr float LOST_ANTI_PLUG_TH = 50.0f;
static constexpr float MOTOR_DEADZONE_PWM = 18.0f;

// ============================================================================
// 6. STARTUP AUDIO, CALIBRATION, TELEMETRY & BLACKBOX CONFIG
// ============================================================================

static constexpr uint32_t START_DELAY_MS = 5000;
static constexpr bool ENABLE_START_BEEPS = false;
static constexpr uint32_t BEEP_DUTY_PCT = 58;

static constexpr uint32_t STARTUP_SETTLE_MS = 120;
static constexpr bool ENABLE_BOOT_SWEEP_CALIB = false;

static constexpr bool ENABLE_TELEMETRY = false;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

static constexpr bool ENABLE_RUN_STATS = true;
static constexpr uint32_t RUN_STATS_PRINT_MS = 1000;

// Blackbox: 100 Hz (moi 5 tick = 10ms), 2500 mau = 25 giay (~70 KB RAM & Flash)
static constexpr bool ENABLE_BLACKBOX = true;
static constexpr uint32_t BB_DIVIDER = 5;
static constexpr uint32_t BB_MAX_FRAMES = 2500;
static constexpr uint32_t BB_MIN_SAVE_FRAMES = 50; // Bo qua luot < 0.5s (cam tren khong luc cam USB)
static constexpr uint32_t BB_SAVE_DELAY_MS = 300;
static constexpr const char* BB_FILE_PATH = "/blackbox.bin";

// ============================================================================
// HELPERS (Macro-Safe Inline Math)
// ============================================================================

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint32_t clamp_u32(uint32_t x, uint32_t lo, uint32_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline int16_t clamp_i16(float x) {
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return static_cast<int16_t>(x);
}

static inline float smoothstep(float edge0, float edge1, float x) {
    float t = clampf((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static inline void delayUsBusy(uint32_t us) {
    uint32_t t0 = micros();
    while (static_cast<uint32_t>(micros() - t0) < us) {
    }
}

static inline void delayMsBusy(uint32_t ms) {
    uint32_t t0 = millis();
    while (static_cast<uint32_t>(millis() - t0) < ms) {
    }
}

// ============================================================================
// MOTOR-COIL AUDIO (In-Phase Bipolar Micro-Pulses, 0 Net Torque)
// ============================================================================

struct NoteStep {
    uint32_t freq_hz;
    uint32_t play_ms;
    uint32_t rest_ms;
};

static constexpr NoteStep ASTRONOMIA_SONG[] = {
    {1480, 315, 95}, {1480, 155, 50}, {2217, 155, 50}, {1976, 315, 95}, {1760, 315, 95},
    {1661, 315, 95}, {1661, 155, 50}, {1661, 155, 50}, {1976, 315, 95}, {1760, 155, 50}, {1661, 155, 50},
    {1480, 315, 95}, {1480, 155, 50}, {3520, 155, 50}, {3322, 155, 50}, {3520, 155, 50}, {3322, 155, 50}, {3520, 155, 50},
};

static void motorBeep(uint32_t freq_hz, uint32_t duration_ms) {
    if (freq_hz == 0 || duration_ms == 0) return;

    uint32_t period_us = 1000000UL / freq_hz;
    if (period_us < 160) period_us = 160;
    uint32_t half_period_us = period_us / 2;
    uint32_t dead_us = 22;
    uint32_t duty_pct = clamp_u32(BEEP_DUTY_PCT, 20, 78);
    uint32_t max_pulse = (half_period_us > dead_us) ? (half_period_us - dead_us) : 0;
    uint32_t pulse_us = (half_period_us * duty_pct) / 100;
    if (pulse_us > max_pulse) pulse_us = max_pulse;
    if (pulse_us < 20) pulse_us = 20;
    uint32_t off_us = (half_period_us > pulse_us) ? (half_period_us - pulse_us) : 0;
    uint32_t cycles = static_cast<uint32_t>((static_cast<uint64_t>(duration_ms) * 1000ULL) / period_us);

    for (uint32_t c = 0; c < cycles; ++c) {
        digitalWrite(PIN_L_A, HIGH);
        digitalWrite(PIN_L_B, LOW);
        digitalWrite(PIN_R_A, HIGH);
        digitalWrite(PIN_R_B, LOW);
        delayUsBusy(pulse_us);

        digitalWrite(PIN_L_A, LOW);
        digitalWrite(PIN_R_A, LOW);
        delayUsBusy(off_us);

        digitalWrite(PIN_L_B, HIGH);
        digitalWrite(PIN_R_B, HIGH);
        delayUsBusy(pulse_us);

        digitalWrite(PIN_L_B, LOW);
        digitalWrite(PIN_R_B, LOW);
        delayUsBusy(off_us);
    }

    digitalWrite(PIN_L_A, LOW);
    digitalWrite(PIN_L_B, LOW);
    digitalWrite(PIN_R_A, LOW);
    digitalWrite(PIN_R_B, LOW);
}

static void playMelody(const struct NoteStep* notes, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (notes[i].freq_hz > 0 && notes[i].play_ms > 0) {
            motorBeep(notes[i].freq_hz, notes[i].play_ms);
        }
        if (notes[i].rest_ms > 0) {
            delayMsBusy(notes[i].rest_ms);
        }
    }
}

// ============================================================================
// SENSOR PROCESSOR (4-IR Trapezoid Geometry & Obstacle Classifier)
// ============================================================================

enum class LineKind : uint8_t {
    Tracking,
    Crossing,
    CenterStraddle,
    Lost,
    Lifted,
};

struct LineState {
    LineKind kind = LineKind::Tracking;
    float pos = 0.0f;
    float strength = 0.0f;
    float dir = 1.0f;
    uint32_t elapsed_ms = 0;
    bool is_new_event = false;
    bool wings_clear = false;
};

static inline void readIrMedian3(uint16_t out[4]) {
    uint16_t s[4][3];
    for (int k = 0; k < 3; ++k) {
        s[0][k] = static_cast<uint16_t>(analogRead(PIN_IR_S1));
        s[1][k] = static_cast<uint16_t>(analogRead(PIN_IR_S2));
        s[2][k] = static_cast<uint16_t>(analogRead(PIN_IR_S3));
        s[3][k] = static_cast<uint16_t>(analogRead(PIN_IR_S4));
    }
    for (int i = 0; i < 4; ++i) {
        uint16_t a = s[i][0], b = s[i][1], c = s[i][2];
        uint16_t max_ab = (a > b) ? a : b;
        uint16_t min_ab = (a < b) ? a : b;
        out[i] = (c > max_ab) ? max_ab : ((c < min_ab) ? min_ab : c);
    }
}

class SensorProcessor {
public:
    SensorProcessor() { resetAll(); }

    void resetAll() {
        for (int i = 0; i < 4; ++i) {
            white_ref_[i] = INIT_WHITE_ADC;
            black_ref_[i] = INIT_BLACK_ADC;
            white_glare_streak_[i] = 0;
            weights_[i] = 0.0f;
            raw_black_glob_[i] = 0.0f;
            last_raw_[i] = static_cast<uint16_t>(INIT_WHITE_ADC);
            cross_dark_streak_[i] = 0;
            cross_latch_ticks_[i] = 0;
            cross_latch_peak_[i] = 0.0f;
        }
        arena_seeded_ = false;
        early_run_ticks_ = 0;
        early_min_black_ = 4095.0f;
        early_black_locked_ = false;
        last_valid_pos_ = 0.0f;
        last_side_ = 1.0f;
        line_width_ema_ = 1.0f;
        all_dark_ticks_ = 0;
        cross_confirm_ticks_ = 0;
        cross_cooldown_ticks_ = 0;
        outer_recent_ticks_ = 0;
        straddle_ticks_ = 0;
        lost_ticks_ = 0;
    }

    static float computePos(const float w[4], float last_valid_pos) {
        float sum_w = w[0] + w[1] + w[2] + w[3];
        float weighted_sum = w[0] * SENSOR_POS_NORM[0] + w[1] * SENSOR_POS_NORM[1] +
                             w[2] * SENSOR_POS_NORM[2] + w[3] * SENSOR_POS_NORM[3];

        float outer_w = fmaxf(w[0], w[3]);
        float outer_active = smoothstep(0.08f, 0.45f, outer_w);
        float denom_floor =
            SENSOR_DENOM_FLOOR * (1.0f - outer_active) + SENSOR_OUTER_DENOM_FLOOR * outer_active;
        float pos = weighted_sum / fmaxf(sum_w, denom_floor);

        float abs_p = fabsf(pos);
        if (abs_p < SENSOR_INNER_POS) {
            float u = abs_p / SENSOR_INNER_POS;
            float s = (pos >= 0.0f) ? 1.0f : -1.0f;
            float center_pos =
                s * SENSOR_INNER_POS * (SENSOR_CENTER_SLOPE * u + (1.0f - SENSOR_CENTER_SLOPE) * u * u);
            pos = center_pos * (1.0f - outer_active) + pos * outer_active;
        }

        float r_mem = smoothstep(0.35f, 0.70f, fmaxf(last_valid_pos, pos));
        float r_clr = clampf((0.28f - w[2]) / 0.28f, 0.0f, 1.0f) *
                      clampf((0.12f - w[1]) / 0.12f, 0.0f, 1.0f) *
                      clampf((0.12f - w[0]) / 0.12f, 0.0f, 1.0f);
        if (w[3] > 0.03f && w[3] > w[0]) {
            pos = fmaxf(pos, r_clr + pos * (1.0f - r_clr));
            pos += r_mem * r_clr * clampf(1.0f - w[3], 0.0f, 1.0f) * SENSOR_OUTER_EXT;
        }

        float l_mem = smoothstep(0.35f, 0.70f, fmaxf(-last_valid_pos, -pos));
        float l_clr = clampf((0.28f - w[1]) / 0.28f, 0.0f, 1.0f) *
                      clampf((0.12f - w[2]) / 0.12f, 0.0f, 1.0f) *
                      clampf((0.12f - w[3]) / 0.12f, 0.0f, 1.0f);
        if (w[0] > 0.03f && w[0] > w[3]) {
            pos = fminf(pos, -l_clr + pos * (1.0f - l_clr));
            pos -= l_mem * l_clr * clampf(1.0f - w[0], 0.0f, 1.0f) * SENSOR_OUTER_EXT;
        }

        return clampf(pos, -SENSOR_MAX_POS, SENSOR_MAX_POS);
    }

    void seedStartupSample(const uint16_t raw_gpio[4]) {
        uint16_t ordered[4];
        orderChannels(raw_gpio, ordered);
        for (int i = 0; i < 4; ++i) last_raw_[i] = ordered[i];

        uint16_t brightest = 0;
        uint16_t darkest = 4095;
        for (int i = 0; i < 4; ++i) {
            if (ordered[i] > brightest) brightest = ordered[i];
            if (ordered[i] < darkest) darkest = ordered[i];
        }
        float b_f = static_cast<float>(brightest);
        float d_f = static_cast<float>(darkest);

        if (IR_WHITE_IS_HIGH && b_f > 1400.0f) {
            bool has_span = (b_f - d_f) > MIN_CAL_SPAN;
            if (!arena_seeded_) {
                float init_black = has_span ? d_f : INIT_BLACK_ADC;
                for (int i = 0; i < 4; ++i) {
                    float v = static_cast<float>(ordered[i]);
                    white_ref_[i] = ((b_f - v) < 360.0f) ? v : (b_f - 80.0f);
                    black_ref_[i] = init_black;
                }
                early_black_locked_ = has_span;
                arena_seeded_ = true;
            } else {
                for (int i = 0; i < 4; ++i) {
                    float v = static_cast<float>(ordered[i]);
                    if (v > white_ref_[i]) {
                        white_ref_[i] = 0.74f * white_ref_[i] + 0.26f * v;
                    } else if ((b_f - v) < 320.0f) {
                        white_ref_[i] = 0.90f * white_ref_[i] + 0.10f * v;
                    }
                    if (has_span) {
                        float target_black = (v < b_f - 0.55f * (b_f - d_f)) ? v : d_f;
                        black_ref_[i] = 0.80f * black_ref_[i] + 0.20f * target_black;
                        early_black_locked_ = true;
                    }
                }
            }
        }
    }

    void sweepCalibrateReset() {
        for (int i = 0; i < 4; ++i) {
            white_ref_[i] = 0.0f;
            black_ref_[i] = 4095.0f;
        }
    }

    void sweepCalibrateSample(const uint16_t raw_gpio[4]) {
        uint16_t ordered[4];
        orderChannels(raw_gpio, ordered);
        for (int i = 0; i < 4; ++i) {
            last_raw_[i] = ordered[i];
            float v = static_cast<float>(ordered[i]);
            if (IR_WHITE_IS_HIGH) {
                if (v > white_ref_[i]) white_ref_[i] = v;
                if (v < black_ref_[i]) black_ref_[i] = v;
            } else {
                if (v < white_ref_[i]) white_ref_[i] = v;
                if (v > black_ref_[i]) black_ref_[i] = v;
            }
        }
    }

    void sweepCalibrateFinish() {
        for (int i = 0; i < 4; ++i) {
            if (IR_WHITE_IS_HIGH) {
                if (white_ref_[i] - black_ref_[i] < MIN_CAL_SPAN) {
                    white_ref_[i] = INIT_WHITE_ADC;
                    black_ref_[i] = INIT_BLACK_ADC;
                }
            } else if (black_ref_[i] - white_ref_[i] < MIN_CAL_SPAN) {
                white_ref_[i] = INIT_BLACK_ADC;
                black_ref_[i] = INIT_WHITE_ADC;
            }
        }
        arena_seeded_ = true;
        early_black_locked_ = true;
    }

    LineState update(const uint16_t raw_gpio[4], float vel_hint) {
        uint16_t ordered[4];
        orderChannels(raw_gpio, ordered);
        for (int i = 0; i < 4; ++i) last_raw_[i] = ordered[i];

        uint16_t frame_max = ordered[0];
        uint16_t frame_min = ordered[0];
        for (int i = 1; i < 4; ++i) {
            if (ordered[i] > frame_max) frame_max = ordered[i];
            if (ordered[i] < frame_min) frame_min = ordered[i];
        }
        float fmax = static_cast<float>(frame_max);
        float fmin = static_cast<float>(frame_min);
        float spread = fmax - fmin;

        if (IR_WHITE_IS_HIGH && !arena_seeded_ && fmax > 2200.0f && spread > MIN_CAL_SPAN) {
            for (int i = 0; i < 4; ++i) {
                float v = static_cast<float>(ordered[i]);
                white_ref_[i] = ((fmax - v) < 360.0f) ? v : (fmax - 80.0f);
                black_ref_[i] = fmin;
            }
            arena_seeded_ = true;
            early_black_locked_ = true;
        }

        bool has_valid_surface_contrast = IR_WHITE_IS_HIGH && fmax > 2200.0f && spread > MIN_CAL_SPAN;
        float avg_white_prev = 0.25f * (white_ref_[0] + white_ref_[1] + white_ref_[2] + white_ref_[3]);

        float raw_black_glob[4];
        float avg_span = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float v = static_cast<float>(ordered[i]);
            if (IR_WHITE_IS_HIGH) {
                if (v > white_ref_[i]) {
                    if (white_glare_streak_[i] < UINT32_MAX) white_glare_streak_[i]++;
                    if (white_glare_streak_[i] >= CAL_WHITE_GLARE_TICKS) {
                        white_ref_[i] += CAL_WHITE_UP_ALPHA * (v - white_ref_[i]);
                    }
                } else {
                    white_glare_streak_[i] = 0;
                    if (has_valid_surface_contrast && fmax >= 0.90f * avg_white_prev &&
                        v > fmin + 0.78f * spread && v > 2600.0f &&
                        (white_ref_[i] - black_ref_[i]) > 650.0f) {
                        white_ref_[i] += CAL_WHITE_TRIM_ALPHA * (v - white_ref_[i]);
                    }
                }

                if (has_valid_surface_contrast) {
                    bool is_dark_ch = v < (fmax - 0.65f * spread);
                    if (is_dark_ch && v < black_ref_[i] && v >= 5.0f) {
                        black_ref_[i] += CAL_BLACK_DN_ALPHA * (v - black_ref_[i]);
                    }
                }

                float span = fmaxf(white_ref_[i] - black_ref_[i], MIN_CAL_SPAN);
                avg_span += span;
                raw_black_glob[i] = clampf((white_ref_[i] - v) / span, 0.0f, 1.0f);
            } else {
                if (v < white_ref_[i]) white_ref_[i] += CAL_WHITE_UP_ALPHA * (v - white_ref_[i]);
                if (v > black_ref_[i]) black_ref_[i] += CAL_BLACK_DN_ALPHA * (v - black_ref_[i]);
                float span = fmaxf(black_ref_[i] - white_ref_[i], MIN_CAL_SPAN);
                avg_span += span;
                raw_black_glob[i] = clampf((v - white_ref_[i]) / span, 0.0f, 1.0f);
            }
            raw_black_glob_[i] = raw_black_glob[i];
        }
        avg_span *= 0.25f;

        uint32_t early_lock_ticks = (120 * LOOP_FREQ_HZ) / 1000;
        if (IR_WHITE_IS_HIGH && !early_black_locked_ && has_valid_surface_contrast &&
            fmax >= 0.88f * avg_white_prev) {
            if (fmin < early_min_black_) early_min_black_ = fmin;
            if (early_run_ticks_ < UINT32_MAX) early_run_ticks_++;
            if (early_run_ticks_ >= early_lock_ticks) {
                if (early_min_black_ < avg_white_prev - MIN_CAL_SPAN) {
                    for (int i = 0; i < 4; ++i) {
                        if (early_min_black_ > black_ref_[i] + 60.0f) {
                            black_ref_[i] = early_min_black_;
                        }
                    }
                }
                early_black_locked_ = true;
            }
        }

        uint32_t tick_ms = 1000 / LOOP_FREQ_HZ;
        if (tick_ms < 1) tick_ms = 1;
        uint32_t confirm_needed = CROSS_CONFIRM_MS / tick_ms;
        if (confirm_needed < 1) confirm_needed = 1;
        uint32_t cooldown_ticks_max = CROSS_COOLDOWN_MS / tick_ms;
        if (cooldown_ticks_max < 1) cooldown_ticks_max = 1;
        uint32_t latch_window_ticks = CROSSING_LATCH_WINDOW_MS / tick_ms;
        if (latch_window_ticks < 1) latch_window_ticks = 1;

        float raw_sum_glob = raw_black_glob[0] + raw_black_glob[1] + raw_black_glob[2] + raw_black_glob[3];
        if (cross_cooldown_ticks_ > 0) cross_cooldown_ticks_--;

        float min_b_glob = fminf(fminf(raw_black_glob[0], raw_black_glob[1]),
                                 fminf(raw_black_glob[2], raw_black_glob[3]));
        float max_b_glob = fmaxf(fmaxf(raw_black_glob[0], raw_black_glob[1]),
                                 fmaxf(raw_black_glob[2], raw_black_glob[3]));

        for (int i = 0; i < 4; ++i) {
            if (raw_black_glob[i] >= CROSSING_LATCH_MIN_BLACK) {
                if (cross_dark_streak_[i] < UINT32_MAX) cross_dark_streak_[i]++;
                if (cross_dark_streak_[i] >= CROSSING_LATCH_MIN_TICKS) {
                    cross_latch_ticks_[i] = latch_window_ticks;
                    if (raw_black_glob[i] > cross_latch_peak_[i]) {
                        cross_latch_peak_[i] = raw_black_glob[i];
                    }
                }
            } else {
                cross_dark_streak_[i] = 0;
                if (cross_latch_ticks_[i] > 0) {
                    cross_latch_ticks_[i]--;
                    if (cross_latch_ticks_[i] == 0) cross_latch_peak_[i] = 0.0f;
                }
            }
        }

        bool all_four_latched = cross_latch_ticks_[0] > 0 && cross_latch_ticks_[1] > 0 &&
                                cross_latch_ticks_[2] > 0 && cross_latch_ticks_[3] > 0;
        bool is_crossing_staggered = false;
        if (all_four_latched && raw_sum_glob >= CROSSING_LATCH_CURR_SUM) {
            float min_pk = fminf(fminf(cross_latch_peak_[0], cross_latch_peak_[1]),
                                 fminf(cross_latch_peak_[2], cross_latch_peak_[3]));
            float max_pk = fmaxf(fmaxf(cross_latch_peak_[0], cross_latch_peak_[1]),
                                 fmaxf(cross_latch_peak_[2], cross_latch_peak_[3]));
            float sum_pk = cross_latch_peak_[0] + cross_latch_peak_[1] + cross_latch_peak_[2] + cross_latch_peak_[3];
            is_crossing_staggered = (min_pk >= CROSSING_LATCH_MIN_BLACK) &&
                                    (sum_pk >= CROSSING_LATCH_MIN_SUM) &&
                                    ((max_pk - min_pk) <= CROSSING_LATCH_MAX_SPREAD);
        }

        bool is_air_dark = (min_b_glob >= AIR_MIN_ALL_BLACK) &&
                           ((max_b_glob - min_b_glob) <= AIR_MAX_SPREAD_RATIO) &&
                           (spread <= (AIR_MAX_SPREAD_RATIO + 0.08f) * avg_span) &&
                           (fmax < AIR_MAX_RAW_ADC);

        bool is_crossing_raw = (min_b_glob >= CROSSING_MIN_ALL_BLACK) &&
                               (raw_sum_glob >= CROSSING_MIN_SUM) &&
                               ((max_b_glob - min_b_glob) <= CROSSING_MAX_SPREAD_RATIO);

        float smooth_keep = 1.0f - smoothstep(BG_SUBTRACT_LO, BG_SUBTRACT_HI, min_b_glob);
        float bg_floor = min_b_glob * smooth_keep;
        float denom_clean = fmaxf(BLACK_SAT_CEIL - bg_floor, 0.35f) - BLACK_NOISE_FLOOR;
        float inv_clean = 1.0f / fmaxf(denom_clean, 0.10f);
        float sum_w = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float clean = clampf((raw_black_glob[i] - bg_floor - BLACK_NOISE_FLOOR) * inv_clean, 0.0f, 1.0f);
            weights_[i] = clean;
            sum_w += clean;
        }

        uint32_t outer_lock_ticks = OUTER_RECENT_LOCK_MS / tick_ms;
        if (outer_lock_ticks < 1) outer_lock_ticks = 1;
        if (weights_[0] > 0.25f || weights_[3] > 0.25f) {
            outer_recent_ticks_ = outer_lock_ticks;
        } else if (outer_recent_ticks_ > 0) {
            outer_recent_ticks_--;
        }

        if (is_air_dark) {
            if (all_dark_ticks_ < UINT32_MAX) all_dark_ticks_++;
            if (all_dark_ticks_ * tick_ms >= LIFTED_STOP_MS) {
                cross_confirm_ticks_ = 0;
                cross_cooldown_ticks_ = 0;
                outer_recent_ticks_ = 0;
                straddle_ticks_ = 0;
                lost_ticks_ = 0;
                arena_seeded_ = false;
                early_black_locked_ = false;
                early_run_ticks_ = 0;
                early_min_black_ = 4095.0f;
                for (int i = 0; i < 4; ++i) {
                    white_glare_streak_[i] = 0;
                    cross_dark_streak_[i] = 0;
                    cross_latch_ticks_[i] = 0;
                    cross_latch_peak_[i] = 0.0f;
                }
                LineState st;
                st.kind = LineKind::Lifted;
                return st;
            }
        } else {
            all_dark_ticks_ = 0;
        }

        if (is_crossing_raw) {
            if (cross_confirm_ticks_ < UINT32_MAX) cross_confirm_ticks_++;
        } else {
            cross_confirm_ticks_ = 0;
        }

        bool crossing_confirmed = (cross_confirm_ticks_ >= confirm_needed) || is_crossing_staggered;
        if (crossing_confirmed) {
            straddle_ticks_ = 0;
            lost_ticks_ = 0;
            bool is_new = false;
            if (cross_cooldown_ticks_ == 0) {
                cross_cooldown_ticks_ = cooldown_ticks_max;
                for (int i = 0; i < 4; ++i) {
                    cross_dark_streak_[i] = 0;
                    cross_latch_ticks_[i] = 0;
                    cross_latch_peak_[i] = 0.0f;
                }
                is_new = true;
            }
            uint32_t cross_ms = (cross_confirm_ticks_ > 0) ? (cross_confirm_ticks_ * tick_ms) : CROSS_CONFIRM_MS;
            bool wings_clear = (weights_[0] < 0.25f) && (weights_[3] < 0.25f);
            LineState st;
            st.kind = LineKind::Crossing;
            st.elapsed_ms = cross_ms;
            st.is_new_event = is_new;
            st.wings_clear = wings_clear;
            return st;
        }

        if (sum_w >= LINE_PRESENT_SUM) {
            straddle_ticks_ = 0;
            lost_ticks_ = 0;
            float pos = computePos(weights_, last_valid_pos_);
            line_width_ema_ = 0.96f * line_width_ema_ + 0.04f * sum_w;
            last_valid_pos_ = pos;
            if (pos > 0.20f) last_side_ = 1.0f;
            else if (pos < -0.20f) last_side_ = -1.0f;

            LineState st;
            st.kind = LineKind::Tracking;
            st.pos = pos;
            st.strength = sum_w;
            return st;
        }

        if (outer_recent_ticks_ == 0 && fabsf(last_valid_pos_) < STRADDLE_POS_LIMIT &&
            fabsf(vel_hint) < STRADDLE_VEL_LIMIT && straddle_ticks_ * tick_ms < MAX_STRADDLE_MS) {
            if (straddle_ticks_ < UINT32_MAX) straddle_ticks_++;
            LineState st;
            st.kind = LineKind::CenterStraddle;
            st.pos = last_valid_pos_ * 0.65f;
            st.elapsed_ms = straddle_ticks_ * tick_ms;
            return st;
        }

        if (lost_ticks_ < UINT32_MAX) lost_ticks_++;
        float dir;
        if (outer_recent_ticks_ > 0 || fabsf(last_valid_pos_) >= STRADDLE_POS_LIMIT) {
            dir = (last_valid_pos_ >= 0.0f) ? 1.0f : -1.0f;
        } else if (fabsf(vel_hint) > 0.30f) {
            dir = (vel_hint >= 0.0f) ? 1.0f : -1.0f;
        } else {
            dir = last_side_;
        }

        LineState st;
        st.kind = LineKind::Lost;
        st.dir = dir;
        st.elapsed_ms = lost_ticks_ * tick_ms;
        return st;
    }

    const float* weights() const { return weights_; }
    const uint16_t* lastRaw() const { return last_raw_; }

private:
    static void orderChannels(const uint16_t raw_gpio[4], uint16_t out[4]) {
        if (MIRROR_SENSORS_LR) {
            out[0] = raw_gpio[3];
            out[1] = raw_gpio[2];
            out[2] = raw_gpio[1];
            out[3] = raw_gpio[0];
        } else {
            for (int i = 0; i < 4; ++i) out[i] = raw_gpio[i];
        }
    }

    float white_ref_[4];
    float black_ref_[4];
    uint32_t white_glare_streak_[4];
    bool arena_seeded_;
    uint32_t early_run_ticks_;
    float early_min_black_;
    bool early_black_locked_;
    float weights_[4];
    float raw_black_glob_[4];
    uint16_t last_raw_[4];
    float last_valid_pos_;
    float last_side_;
    float line_width_ema_;
    uint32_t all_dark_ticks_;
    uint32_t cross_confirm_ticks_;
    uint32_t cross_cooldown_ticks_;
    uint32_t cross_dark_streak_[4];
    uint32_t cross_latch_ticks_[4];
    float cross_latch_peak_[4];
    uint32_t outer_recent_ticks_;
    uint32_t straddle_ticks_;
    uint32_t lost_ticks_;
};

// ============================================================================
// 2ND-ORDER DISCRETE ESO LADRC CONTROLLER (FAST-DECAY OPTIMIZED)
// ============================================================================

struct LadrcOutput {
    float steer_pwm;
    float e_f;
    float de_f;
    float z3_pwm;
};

class LadrcController {
public:
    explicit LadrcController(const TuningProfile& profile) : profile_(profile) {
        float beta = expf(-profile.wo * DT_SEC);
        float q = 1.0f - beta;
        l1_ = 1.0f - beta * beta * beta;
        l2_ = (1.5f / DT_SEC) * q * q * (1.0f + beta);
        l3_ = (1.0f / (DT_SEC * DT_SEC)) * q * q * q;
        reset();
    }

    void reset() {
        z1_ = 0.0f;
        z2_ = 0.0f;
        z3_ = 0.0f;
        u_cmd_ = 0.0f;
        u_applied_ = 0.0f;
        saturated_dir_ = 0.0f;
    }

    void resetToPos(float pos) {
        z1_ = pos;
        z2_ = 0.0f;
        z3_ = 0.0f;
        u_cmd_ = 0.0f;
        u_applied_ = 0.0f;
        saturated_dir_ = 0.0f;
    }

    void feedbackAppliedSteer(float applied_steer_pwm) {
        u_applied_ = applied_steer_pwm;
        float sat_err = u_cmd_ - applied_steer_pwm;
        if (sat_err > 5.0f) saturated_dir_ = 1.0f;
        else if (sat_err < -5.0f) saturated_dir_ = -1.0f;
        else saturated_dir_ = 0.0f;
    }

    LadrcOutput step(float e_meas, float est_forward_speed) {
        float h = DT_SEC;
        float b0_eff = effectiveB0(est_forward_speed);

        float acc = z3_ - b0_eff * u_applied_;
        float z1_p = z1_ + h * z2_ + 0.5f * h * h * acc;
        float z2_p = z2_ + h * acc;
        float z3_p = z3_;

        float e_o = e_meas - z1_p;
        z1_ = z1_p + l1_ * e_o;
        z2_ = clampf(z2_p + l2_ * e_o, -45.0f, 45.0f);

        float dz3 = l3_ * e_o;
        if (saturated_dir_ * dz3 <= 0.0f) {
            z3_ = z3_p * 0.9965f + dz3;
        } else {
            z3_ = z3_p * 0.985f;
        }

        float z3_lim = b0_eff * profile_.max_z3_pwm;
        z3_ = clampf(z3_, -z3_lim, z3_lim);

        return computeLsef(b0_eff);
    }

    LadrcOutput stepHoldDisturbance(float e_target, float est_forward_speed, float z3_decay) {
        float b0_eff = effectiveB0(est_forward_speed);
        z1_ += 0.25f * (e_target - z1_);
        z2_ *= 0.80f;
        z3_ *= z3_decay;
        return computeLsef(b0_eff);
    }

    void coastDuringLost(float lost_dir) {
        z1_ = 0.88f * z1_ + 0.12f * (lost_dir * 1.30f);
        z2_ *= 0.85f;
        z3_ *= 0.88f;
        u_cmd_ = 0.0f;
        saturated_dir_ = 0.0f;
    }

    float velocityHint() const { return z2_; }
    float z3PwmEst() const { return z3_ / profile_.b0; }

private:
    float effectiveB0(float est_forward_speed) const {
        float v_norm = clampf(est_forward_speed / PWM_MAX, 0.40f, 1.0f);
        return profile_.b0 * (0.72f + 0.28f * v_norm);
    }

    LadrcOutput computeLsef(float b0_eff) {
        float abs_e = fabsf(z1_);
        float abs_de = fabsf(z2_);

        float entering_curve = (z1_ * z2_ > 0.0f)
            ? smoothstep(0.40f, 0.95f, abs_e) * smoothstep(1.2f, 6.5f, abs_de)
            : 0.0f;

        float wing_excess = fmaxf(abs_e - 0.60f, 0.0f);
        float wc_mult = 1.0f + profile_.wing_boost_slope * wing_excess +
                        profile_.curve_entry_boost * entering_curve;
        float wc_eff = profile_.wc * fminf(wc_mult, 1.48f);

        float straight = 1.0f - smoothstep(0.04f, 0.14f, abs_e);
        float g_e = smoothstep(0.5f * LADRC_E_DEADBAND, 1.5f * LADRC_E_DEADBAND, abs_e);
        float g_de = smoothstep(0.5f * LADRC_DE_DEADBAND, 1.5f * LADRC_DE_DEADBAND, abs_de);
        float z1_use = z1_ * (1.0f - straight * (1.0f - g_e));
        float z2_use = z2_ * (1.0f - straight * (1.0f - g_de));

        float u0 = wc_eff * wc_eff * z1_use + 2.0f * profile_.zeta * wc_eff * z2_use;
        float u_raw = (u0 + z3_) / b0_eff;
        u_cmd_ = u_raw;
        float steer_pwm = clampf(u_raw, -profile_.max_steer_pwm, profile_.max_steer_pwm);

        return {steer_pwm, z1_, z2_, z3_ / b0_eff};
    }

    float z1_, z2_, z3_;
    float u_cmd_, u_applied_, saturated_dir_;
    float l1_, l2_, l3_;
    TuningProfile profile_;
};

// ============================================================================
// MOTION PLANNER (Velocity & X-Crossing Shortcut State Machine)
// ============================================================================

struct DriveCommand {
    float base_speed;
    float steer_cmd;
    float max_reverse_pwm;
    bool instant_snap;
    float anti_plug_th;
    bool safety_stopped;
};

class MotionPlanner {
public:
    explicit MotionPlanner(const TuningProfile& profile) : profile_(profile) { reset(); }

    void reset() {
        base_speed_lpf_ = profile_.v_curve_min;
        recovery_ramp_ = 0.0f;
        cross_count_ = 0;
        cross_mode_ = CrossMode::None;
        cross_dir_sign_ = 1;
        cross_ticks_ = 0;
        cross_white_ticks_ = 0;
    }

    size_t crossCount() const { return cross_count_; }

    bool isInShortcut() const {
        return cross_mode_ == CrossMode::ShortcutLeave ||
               cross_mode_ == CrossMode::ShortcutAcquire ||
               cross_mode_ == CrossMode::ShortcutBrake;
    }

    DriveCommand compute(
        const LineState& line_state,
        const float sensor_weights[4],
        LadrcController& ctrl,
        float est_forward_speed
    ) {
        uint32_t tick_ms = 1000 / LOOP_FREQ_HZ;
        if (tick_ms < 1) tick_ms = 1;

        if (line_state.kind == LineKind::Lifted) {
            reset();
            ctrl.reset();
            return {0.0f, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, true};
        }

        if (cross_mode_ == CrossMode::None &&
            line_state.kind == LineKind::Crossing &&
            line_state.is_new_event) {
            size_t idx = cross_count_;
            cross_count_++;
            CrossingAction action = (idx < 6) ? CROSSING_PLAN[idx] : CrossingAction::Straight;
            ctrl.reset();
            recovery_ramp_ = 0.0f;
            cross_ticks_ = 0;
            cross_white_ticks_ = 0;
            if (action == CrossingAction::Straight) {
                cross_mode_ = CrossMode::StraightLock;
            } else if (action == CrossingAction::ShortcutRight) {
                cross_mode_ = CrossMode::ShortcutLeave;
                cross_dir_sign_ = 1;
            } else {
                cross_mode_ = CrossMode::ShortcutLeave;
                cross_dir_sign_ = -1;
            }
        }

        switch (cross_mode_) {
            case CrossMode::StraightLock: {
                cross_ticks_++;
                uint32_t elapsed = cross_ticks_ * tick_ms;
                bool wings_clear = (sensor_weights[0] < 0.25f) && (sensor_weights[3] < 0.25f);
                bool still_crossing = (line_state.kind == LineKind::Crossing);
                if ((elapsed >= 40 && !still_crossing && wings_clear) || elapsed >= MAX_CROSS_LOCK_MS) {
                    cross_mode_ = CrossMode::None;
                } else {
                    base_speed_lpf_ += 0.28f * (profile_.v_crossing - base_speed_lpf_);
                    return {base_speed_lpf_, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, false};
                }
                break;
            }

            case CrossMode::ShortcutLeave: {
                cross_ticks_++;
                uint32_t elapsed = cross_ticks_ * tick_ms;
                bool old_line_cleared = (cross_dir_sign_ > 0)
                    ? (sensor_weights[0] < 0.25f && sensor_weights[1] < 0.25f && sensor_weights[2] < 0.25f)
                    : (sensor_weights[1] < 0.25f && sensor_weights[2] < 0.25f && sensor_weights[3] < 0.25f);

                if (elapsed >= SHORTCUT_LEAVE_MIN_MS && old_line_cleared) {
                    cross_white_ticks_++;
                } else {
                    cross_white_ticks_ = 0;
                }

                if (cross_white_ticks_ >= 3 || elapsed >= SHORTCUT_LEAVE_MAX_MS) {
                    cross_mode_ = CrossMode::ShortcutAcquire;
                    cross_ticks_ = 0;
                }

                float outer = (elapsed < SHORTCUT_BRAKE_MS) ? SHORTCUT_BRAKE_OUTER_PWM : SHORTCUT_OUTER_PWM;
                float inner_rev = (elapsed < SHORTCUT_BRAKE_MS) ? 0.0f : SHORTCUT_INNER_REV_PWM;
                float plug_th = (elapsed < SHORTCUT_BRAKE_MS) ? ANTI_PLUG_OMEGA_TH : SHORTCUT_ANTI_PLUG_TH;
                float base = 0.5f * (outer - inner_rev);
                float steer = static_cast<float>(cross_dir_sign_) * 0.5f * (outer + inner_rev);
                base_speed_lpf_ = base;
                return {base, steer, inner_rev, true, plug_th, false};
            }

            case CrossMode::ShortcutAcquire: {
                cross_ticks_++;
                uint32_t elapsed = cross_ticks_ * tick_ms;
                // Cho mat trong (S3 khi re phai, S2 khi re trai) cham vach de dap dung giua tam
                float inner_turn_w = (cross_dir_sign_ > 0) ? sensor_weights[2] : sensor_weights[1];
                float far_inner_w = (cross_dir_sign_ > 0) ? sensor_weights[1] : sensor_weights[2];
                bool branch_hit = (inner_turn_w >= 0.48f) || (far_inner_w >= 0.42f);

                if (branch_hit || elapsed >= SHORTCUT_ACQUIRE_MAX_MS) {
                    cross_mode_ = CrossMode::ShortcutBrake;
                    cross_ticks_ = 0;
                    float base = 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
                    float steer = -static_cast<float>(cross_dir_sign_) * 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
                    base_speed_lpf_ = base;
                    return {base, steer, 0.0f, true, SHORTCUT_ANTI_PLUG_TH, false};
                } else {
                    float base = 0.5f * (SHORTCUT_OUTER_PWM - SHORTCUT_INNER_REV_PWM);
                    float steer = static_cast<float>(cross_dir_sign_) * 0.5f *
                                  (SHORTCUT_OUTER_PWM + SHORTCUT_INNER_REV_PWM);
                    base_speed_lpf_ = base;
                    return {base, steer, SHORTCUT_INNER_REV_PWM, true, SHORTCUT_ANTI_PLUG_TH, false};
                }
            }

            case CrossMode::ShortcutBrake: {
                cross_ticks_++;
                uint32_t elapsed = cross_ticks_ * tick_ms;
                bool centered = (cross_dir_sign_ > 0)
                    ? ((sensor_weights[1] >= 0.30f && sensor_weights[2] >= 0.30f) || sensor_weights[1] >= 0.45f)
                    : ((sensor_weights[1] >= 0.30f && sensor_weights[2] >= 0.30f) || sensor_weights[2] >= 0.45f);

                if ((elapsed >= SHORTCUT_COUNTER_BRAKE_MIN_MS && centered) ||
                    elapsed >= SHORTCUT_COUNTER_BRAKE_MAX_MS) {
                    cross_mode_ = CrossMode::None;
                    float sum_w = sensor_weights[0] + sensor_weights[1] + sensor_weights[2] + sensor_weights[3];
                    float pos = (sum_w >= LINE_PRESENT_SUM) ? SensorProcessor::computePos(sensor_weights, 0.0f) : 0.0f;
                    ctrl.resetToPos(pos);
                    base_speed_lpf_ = profile_.v_curve_min;
                } else {
                    float base = 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
                    float steer = -static_cast<float>(cross_dir_sign_) * 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
                    base_speed_lpf_ = base;
                    return {base, steer, 0.0f, true, SHORTCUT_ANTI_PLUG_TH, false};
                }
                break;
            }

            case CrossMode::None:
                break;
        }

        switch (line_state.kind) {
            case LineKind::Tracking: {
                recovery_ramp_ = 0.0f;
                LadrcOutput out = ctrl.step(line_state.pos, est_forward_speed);
                float curve_frac = smoothstep(0.62f, 1.18f, fabsf(out.e_f));
                float target_base = profile_.v_max - (profile_.v_max - profile_.v_curve_min) * curve_frac;
                base_speed_lpf_ += 0.25f * (target_base - base_speed_lpf_);
                return {base_speed_lpf_, out.steer_pwm, profile_.max_reverse_tracking, false, ANTI_PLUG_OMEGA_TH, false};
            }

            case LineKind::Crossing: {
                recovery_ramp_ = 0.0f;
                LadrcOutput out = ctrl.stepHoldDisturbance(0.0f, est_forward_speed, 0.90f);
                base_speed_lpf_ += 0.28f * (profile_.v_crossing - base_speed_lpf_);
                return {base_speed_lpf_, clampf(out.steer_pwm * 0.15f, -22.0f, 22.0f), 0.0f, false, ANTI_PLUG_OMEGA_TH, false};
            }

            case LineKind::CenterStraddle: {
                recovery_ramp_ = 0.0f;
                LadrcOutput out = ctrl.stepHoldDisturbance(line_state.pos, est_forward_speed, 0.96f);
                base_speed_lpf_ += 0.28f * (profile_.v_straddle - base_speed_lpf_);
                return {base_speed_lpf_, clampf(out.steer_pwm, -80.0f, 80.0f), 0.0f, false, ANTI_PLUG_OMEGA_TH, false};
            }

            case LineKind::Lost: {
                if (line_state.elapsed_ms >= LOST_SAFETY_STOP_MS) {
                    base_speed_lpf_ = 0.0f;
                    recovery_ramp_ = 0.0f;
                    return {0.0f, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, true};
                }
                ctrl.coastDuringLost(line_state.dir);
                recovery_ramp_ = fminf(recovery_ramp_ + 0.16f, 1.0f);
                float outer = profile_.v_curve_min +
                              recovery_ramp_ * (profile_.recovery_outer_pwm - profile_.v_curve_min);
                float inner = (line_state.elapsed_ms < 24)
                    ? 0.0f
                    : fminf(static_cast<float>(line_state.elapsed_ms - 24) / 36.0f, 1.0f) *
                          profile_.recovery_inner_pwm;
                float base = 0.5f * (outer + inner);
                float steer = line_state.dir * 0.5f * (outer - inner);
                base_speed_lpf_ = profile_.v_curve_min;
                return {base, steer, fabsf(profile_.recovery_inner_pwm), false, LOST_ANTI_PLUG_TH, false};
            }

            case LineKind::Lifted:
                return {0.0f, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, true};
        }

        return {0.0f, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, true};
    }

private:
    enum class CrossMode : uint8_t {
        None,
        StraightLock,
        ShortcutLeave,
        ShortcutAcquire,
        ShortcutBrake,
    };

    TuningProfile profile_;
    float base_speed_lpf_;
    float recovery_ramp_;
    size_t cross_count_;
    CrossMode cross_mode_;
    int8_t cross_dir_sign_;
    uint32_t cross_ticks_;
    uint32_t cross_white_ticks_;
};

// ============================================================================
// MOTOR POWER CONTROLLER (Differential-Preserving 1:1 Mixer & 4x AA Shield)
// ============================================================================

struct AppliedDrive {
    float left_pwm;
    float right_pwm;
    float applied_steer;
    float est_forward_speed;
    bool current_limited;
};

class MotorPowerController {
public:
    MotorPowerController() { reset(); }

    void begin() {
        ledcAttach(PIN_L_A, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttach(PIN_L_B, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttach(PIN_R_A, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttach(PIN_R_B, PWM_FREQ_HZ, PWM_RES_BITS);
        coastStop();
    }

    void reset() {
        run_ticks_ = 0;
        applied_base_ = 0.0f;
        omega_hat_l_ = 0.0f;
        omega_hat_r_ = 0.0f;
        coastStop();
    }

    void coastStop() {
        ledcWrite(PIN_L_A, 0);
        ledcWrite(PIN_L_B, 0);
        ledcWrite(PIN_R_A, 0);
        ledcWrite(PIN_R_B, 0);
        applied_base_ = 0.0f;
        float alpha_c = DT_SEC / (MOTOR_TAU_COAST_S + DT_SEC);
        omega_hat_l_ += alpha_c * (0.0f - omega_hat_l_);
        omega_hat_r_ += alpha_c * (0.0f - omega_hat_r_);
    }

    void driveCalibration(float left_target, float right_target) {
        run_ticks_ = (SOFT_START_MS * LOOP_FREQ_HZ) / 1000 + 1;
        DriveCommand cmd = {
            0.5f * (left_target + right_target),
            0.5f * (left_target - right_target),
            90.0f,
            false,
            ANTI_PLUG_OMEGA_TH,
            false,
        };
        (void)apply(cmd);
    }

    AppliedDrive apply(const DriveCommand& cmd) {
        uint32_t soft_ticks = (SOFT_START_MS * LOOP_FREQ_HZ) / 1000;
        if (soft_ticks < 1) soft_ticks = 1;
        if (run_ticks_ < UINT32_MAX) run_ticks_++;

        float soft_scale = 1.0f;
        if (run_ticks_ < soft_ticks) {
            float t = static_cast<float>(run_ticks_) / static_cast<float>(soft_ticks);
            soft_scale = t * t * (3.0f - 2.0f * t);
        }

        float target_base = clampf(cmd.base_speed * soft_scale, -PWM_MAX, PWM_MAX);
        float steer = clampf(cmd.steer_cmd * soft_scale, -PWM_MAX, PWM_MAX);

        float base;
        if (cmd.instant_snap) {
            base = target_base;
        } else {
            float max_base_up = COMMON_ACCEL_PWM_PER_SEC * DT_SEC;
            float max_base_dn = COMMON_BRAKE_PWM_PER_SEC * DT_SEC;
            float base_diff = target_base - applied_base_;
            float base_step = (base_diff > 0.0f) ? fminf(base_diff, max_base_up) : fmaxf(base_diff, -max_base_dn);
            base = applied_base_ + base_step;
        }
        applied_base_ = base;

        float max_w = ACTIVE_PROFILE.max_wheel_pwm;
        float min_pwm = -cmd.max_reverse_pwm;
        float l = base + steer;
        float r = base - steer;
        if (l > max_w) {
            float over = l - max_w;
            l = max_w;
            r -= over;
        } else if (r > max_w) {
            float over = r - max_w;
            r = max_w;
            l -= over;
        }
        l = clampf(l, min_pwm, max_w);
        r = clampf(r, min_pwm, max_w);

        float sum_abs = fabsf(l) + fabsf(r);
        if (sum_abs > MAX_TOTAL_PWM) {
            float scale = MAX_TOTAL_PWM / sum_abs;
            l *= scale;
            r *= scale;
        }

        l = antiPlugGuard(l, omega_hat_l_, cmd.anti_plug_th);
        r = antiPlugGuard(r, omega_hat_r_, cmd.anti_plug_th);

        float current_budget = cmd.instant_snap ? SHORTCUT_CURRENT_BUDGET : BATTERY_CURRENT_BUDGET;
        float draw_l = batteryDraw(l, omega_hat_l_);
        float draw_r = batteryDraw(r, omega_hat_r_);
        float total_draw = draw_l + draw_r;
        bool current_limited = (total_draw > current_budget);
        if (current_limited) {
            float scale = current_budget / total_draw;
            if (draw_l > 0.0f) l = scaleMotorDraw(l, omega_hat_l_, draw_l * scale);
            if (draw_r > 0.0f) r = scaleMotorDraw(r, omega_hat_r_, draw_r * scale);
            l = clampf(l, min_pwm, max_w);
            r = clampf(r, min_pwm, max_w);
        }

        float tau_l, tau_r;
        if (ENABLE_SLOW_DECAY_BRAKE) {
            tau_l = MOTOR_TAU_CCM_S;
            tau_r = MOTOR_TAU_CCM_S;
        } else {
            tau_l = (ENABLE_ZERO_CMD_STATIC_BRAKE && fabsf(l) <= 0.5f)
                ? MOTOR_TAU_CCM_S
                : (isCoasting(l, omega_hat_l_) ? MOTOR_TAU_COAST_S : MOTOR_TAU_DRIVE_S);
            tau_r = (ENABLE_ZERO_CMD_STATIC_BRAKE && fabsf(r) <= 0.5f)
                ? MOTOR_TAU_CCM_S
                : (isCoasting(r, omega_hat_r_) ? MOTOR_TAU_COAST_S : MOTOR_TAU_DRIVE_S);
        }
        omega_hat_l_ += (DT_SEC / (tau_l + DT_SEC)) * (l - omega_hat_l_);
        omega_hat_r_ += (DT_SEC / (tau_r + DT_SEC)) * (r - omega_hat_r_);

        writeHardware(l, r, true);

        return {
            l,
            r,
            0.5f * (l - r),
            fmaxf(0.5f * (omega_hat_l_ + omega_hat_r_), 0.0f),
            current_limited,
        };
    }

private:
    static bool isCoasting(float cmd, float omega_hat) {
        return fabsf(cmd) < 1.0f || (cmd * omega_hat >= 0.0f && fabsf(cmd) < fabsf(omega_hat));
    }

    static float batteryDraw(float cmd, float omega_hat) {
        float abs_cmd = fabsf(cmd);
        if (abs_cmd < 1.0f) return 0.0f;
        if (cmd * omega_hat >= 0.0f) return fmaxf(abs_cmd - fabsf(omega_hat), 0.0f);
        return abs_cmd + fabsf(omega_hat);
    }

    static float scaleMotorDraw(float cmd, float omega_hat, float allowed_draw) {
        if (cmd < 0.0f && omega_hat > 0.0f) return -fmaxf(allowed_draw - omega_hat, 0.0f);
        if (cmd > 0.0f && omega_hat < 0.0f) return fmaxf(allowed_draw + omega_hat, 0.0f);
        if (cmd >= 0.0f) return fmaxf(omega_hat, 0.0f) + allowed_draw;
        return fminf(omega_hat, 0.0f) - allowed_draw;
    }

    static float antiPlugGuard(float cmd, float omega_hat, float thresh) {
        if ((omega_hat > thresh && cmd < -10.0f) || (omega_hat < -thresh && cmd > 10.0f)) {
            return 0.0f;
        }
        return cmd;
    }

    static float compensateDeadzone(float pwm) {
        float abs_v = fabsf(pwm);
        if (abs_v < 1.0f) return 0.0f;
        float sign = (pwm >= 0.0f) ? 1.0f : -1.0f;
        float dz = MOTOR_DEADZONE_PWM * fminf(abs_v / 16.0f, 1.0f);
        float mapped = dz + (abs_v / PWM_MAX) * (PWM_MAX - MOTOR_DEADZONE_PWM);
        return sign * clampf(mapped, 0.0f, PWM_MAX);
    }

    void writeHardware(float logical_l, float logical_r, bool allow_static_brake) {
        float l = compensateDeadzone(logical_l);
        float r = compensateDeadzone(logical_r * RIGHT_MOTOR_TRIM);
        if (SWAP_MOTORS_LR) {
            float tmp = l;
            l = r;
            r = tmp;
        }
        if (INVERT_LEFT_MOTOR) l = -l;
        if (INVERT_RIGHT_MOTOR) r = -r;
        setMotorPair(PIN_L_A, PIN_L_B, l, allow_static_brake);
        setMotorPair(PIN_R_A, PIN_R_B, r, allow_static_brake);
    }

    static void setMotorPair(uint8_t pin_a, uint8_t pin_b, float signed_pwm, bool allow_static_brake) {
        float abs_v = fabsf(signed_pwm);
        uint32_t duty = (abs_v >= 254.5f) ? PWM_HW_MAX : static_cast<uint32_t>(abs_v + 0.5f);
        if (duty > PWM_HW_MAX) duty = PWM_HW_MAX;

        if (ENABLE_SLOW_DECAY_BRAKE) {
            if (signed_pwm > 0.5f) {
                ledcWrite(pin_a, PWM_HW_MAX);
                ledcWrite(pin_b, PWM_HW_MAX - duty);
            } else if (signed_pwm < -0.5f) {
                ledcWrite(pin_a, PWM_HW_MAX - duty);
                ledcWrite(pin_b, PWM_HW_MAX);
            } else {
                ledcWrite(pin_a, PWM_HW_MAX);
                ledcWrite(pin_b, PWM_HW_MAX);
            }
        } else if (signed_pwm > 0.5f) {
            ledcWrite(pin_b, 0);
            ledcWrite(pin_a, duty);
        } else if (signed_pwm < -0.5f) {
            ledcWrite(pin_a, 0);
            ledcWrite(pin_b, duty);
        } else if (allow_static_brake && ENABLE_ZERO_CMD_STATIC_BRAKE) {
            ledcWrite(pin_a, PWM_HW_MAX);
            ledcWrite(pin_b, PWM_HW_MAX);
        } else {
            ledcWrite(pin_a, 0);
            ledcWrite(pin_b, 0);
        }
    }

    float applied_base_;
    float omega_hat_l_;
    float omega_hat_r_;
    uint32_t run_ticks_;
};

// ============================================================================
// FLIGHT BLACKBOX (100Hz RAM Ring Buffer + Auto-Save to LittleFS Flash)
// ============================================================================

struct BlackboxFrame {
    uint16_t t_ms;
    uint16_t raw[4];
    int16_t pos_x100;
    int16_t z2_x10;
    int16_t z3_pwm;
    int16_t steer;
    int16_t l_pwm;
    int16_t r_pwm;
    uint8_t w[4];
    uint8_t state;
    uint8_t flags;
};

struct BlackboxHeader {
    uint32_t magic;
    uint32_t count;
    uint32_t sample_ms;
    uint32_t total_ms;
};

static constexpr uint32_t BB_MAGIC = 0xBB10C301UL;

class BlackboxRecorder {
public:
    void begin() {
        fs_ready_ = false;
        if (!ENABLE_BLACKBOX) return;
        fs_ready_ = LittleFS.begin(true);
        if (fs_ready_) {
            loadFromFlash();
        }
    }

    void beginRun() {
        if (!ENABLE_BLACKBOX) return;
        head_ = 0;
        count_ = 0;
        tick_div_ = 0;
        run_start_ms_ = millis();
        recording_ = true;
        unsaved_ = false;
    }

    bool isRecording() const { return recording_; }
    bool hasUnsavedRun() const { return unsaved_ && count_ >= BB_MIN_SAVE_FRAMES; }
    uint32_t frameCount() const { return count_; }

    void record(
        const LineState& ls,
        const uint16_t raw[4],
        const float w[4],
        const AppliedDrive& app,
        float z2,
        float z3_pwm,
        uint8_t st,
        size_t cross_cnt
    ) {
        if (!ENABLE_BLACKBOX) return;
        if (!recording_) beginRun();

        if ((tick_div_++ % BB_DIVIDER) != 0) return;

        uint32_t dt = static_cast<uint32_t>(millis() - run_start_ms_);
        BlackboxFrame fr;
        fr.t_ms = (dt > 65535UL) ? 65535 : static_cast<uint16_t>(dt);
        for (int i = 0; i < 4; ++i) {
            fr.raw[i] = raw[i];
            fr.w[i] = static_cast<uint8_t>(clampf(w[i] * 100.0f + 0.5f, 0.0f, 100.0f));
        }
        float pos_val = (ls.kind == LineKind::Lost) ? (ls.dir * 1.35f) : ls.pos;
        fr.pos_x100 = clamp_i16(pos_val * 100.0f);
        fr.z2_x10 = clamp_i16(z2 * 10.0f);
        fr.z3_pwm = clamp_i16(z3_pwm);
        fr.steer = clamp_i16(app.applied_steer);
        fr.l_pwm = clamp_i16(app.left_pwm);
        fr.r_pwm = clamp_i16(app.right_pwm);
        fr.state = st;
        uint8_t crs_clamped = (cross_cnt > 127) ? 127 : static_cast<uint8_t>(cross_cnt);
        fr.flags = static_cast<uint8_t>((crs_clamped << 1) | (app.current_limited ? 1U : 0U));

        frames_[head_] = fr;
        head_ = (head_ + 1) % BB_MAX_FRAMES;
        if (count_ < BB_MAX_FRAMES) count_++;
        unsaved_ = true;
    }

    void finishRunAndSave() {
        recording_ = false;
        if (!ENABLE_BLACKBOX) {
            unsaved_ = false;
            return;
        }
        if (!unsaved_ || count_ < BB_MIN_SAVE_FRAMES) {
            unsaved_ = false;
            if (fs_ready_) {
                count_ = 0;
                head_ = 0;
                loadFromFlash();
            }
            return;
        }
        unsaved_ = false;
        if (!fs_ready_) {
            Serial.printf("[BB] Ghi RAM xong: %lu mau (Khong co Flash LittleFS)\n",
                          static_cast<unsigned long>(count_));
            return;
        }

        uint32_t t0 = millis();
        File f = LittleFS.open(BB_FILE_PATH, FILE_WRITE);
        if (!f) {
            Serial.println("[BB] Loi mo file Flash!");
            return;
        }

        BlackboxHeader hdr;
        hdr.magic = BB_MAGIC;
        hdr.count = count_;
        hdr.sample_ms = (1000UL * BB_DIVIDER) / LOOP_FREQ_HZ;
        hdr.total_ms = (count_ > 0) ? frameAt(count_ - 1).t_ms : 0;

        f.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
        if (count_ < BB_MAX_FRAMES || head_ == 0) {
            f.write(reinterpret_cast<const uint8_t*>(frames_), count_ * sizeof(BlackboxFrame));
        } else {
            uint32_t tail_n = BB_MAX_FRAMES - head_;
            f.write(reinterpret_cast<const uint8_t*>(&frames_[head_]), tail_n * sizeof(BlackboxFrame));
            f.write(reinterpret_cast<const uint8_t*>(&frames_[0]), head_ * sizeof(BlackboxFrame));
        }
        f.close();

        Serial.printf(
            "[BB] Da luu %lu mau (%.1fs) vao Flash trong %lums! Gui 's'=tom tat, 'b'=log 10Hz, 'd'=full CSV\n",
            static_cast<unsigned long>(count_),
            static_cast<double>(hdr.total_ms) / 1000.0,
            static_cast<unsigned long>(millis() - t0)
        );
    }

    void printStartupHint() const {
        if (!ENABLE_BLACKBOX) return;
        if (count_ > 0) {
            uint32_t dur_ms = frameAt(count_ - 1).t_ms;
            Serial.printf(
                "[BB] Dang co du lieu luot truoc: %lu mau (%.1fs) | Gui 's'=tom tat, 'b'=log 10Hz, 'd'=CSV 100Hz, 'p'=giu xe dung yen\n",
                static_cast<unsigned long>(count_),
                static_cast<double>(dur_ms) / 1000.0
            );
        } else {
            Serial.println("[BB] San sang ghi 100Hz (25s). Gui 'p' de giu xe dung yen.");
        }
    }

    void pollSerial(bool& inspect_mode) {
        while (Serial.available() > 0) {
            char c = static_cast<char>(Serial.read());
            if (c == '\r' || c == '\n' || c == ' ') continue;
            if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');

            if (c == 's') {
                inspect_mode = true;
                printSummary();
            } else if (c == 'b') {
                inspect_mode = true;
                dumpCsv(10);
            } else if (c == 'd') {
                inspect_mode = true;
                dumpCsv(1);
            } else if (c == 'c') {
                inspect_mode = true;
                clearAll();
            } else if (c == 'p') {
                inspect_mode = true;
                Serial.println("[BB] Che do dung yen (Inspect Mode). Gui 's'/'b'/'d' de xem, 'g' de chay.");
            } else if (c == 'g') {
                inspect_mode = false;
                Serial.println("[BB] Thoat Inspect Mode -> Chuan bi chay!");
            }
        }
    }

    void printSummary() const {
        if (count_ == 0) {
            Serial.println("[BB] Chua co du lieu nao trong Blackbox!");
            return;
        }

        double sum_l = 0, sum_r = 0, sum_steer = 0, sum_abs_steer = 0;
        double sum_z3 = 0, sum_abs_z3 = 0, sum_pos = 0, sum_abs_pos = 0;
        uint32_t raw_min[4] = {4095, 4095, 4095, 4095};
        uint32_t raw_max[4] = {0, 0, 0, 0};
        uint64_t raw_sum[4] = {0, 0, 0, 0};
        uint32_t both250 = 0, ilim = 0;
        uint32_t st_cnt[5] = {0, 0, 0, 0, 0};

        for (uint32_t i = 0; i < count_; ++i) {
            const BlackboxFrame& fr = frameAt(i);
            sum_l += fr.l_pwm;
            sum_r += fr.r_pwm;
            sum_steer += fr.steer;
            sum_abs_steer += fabs(static_cast<double>(fr.steer));
            sum_z3 += fr.z3_pwm;
            sum_abs_z3 += fabs(static_cast<double>(fr.z3_pwm));
            sum_pos += fr.pos_x100;
            sum_abs_pos += fabs(static_cast<double>(fr.pos_x100));
            if (fr.l_pwm >= 250 && fr.r_pwm >= 250) both250++;
            if (fr.flags & 1U) ilim++;
            if (fr.state < 5) st_cnt[fr.state]++;
            for (int k = 0; k < 4; ++k) {
                if (fr.raw[k] < raw_min[k]) raw_min[k] = fr.raw[k];
                if (fr.raw[k] > raw_max[k]) raw_max[k] = fr.raw[k];
                raw_sum[k] += fr.raw[k];
            }
        }

        double n = static_cast<double>(count_);
        auto pct = [n](uint32_t v) -> int { return static_cast<int>((static_cast<double>(v) * 100.0) / n); };
        uint32_t dur_ms = frameAt(count_ - 1).t_ms;

        Serial.println("================ BLACKBOX SUMMARY ================");
        Serial.printf("Frames=%lu (%.2fs) | L_avg=%d R_avg=%d avgPWM=%d | both>=250=%d%% | Ilimit=%d%%\n",
                      static_cast<unsigned long>(count_),
                      static_cast<double>(dur_ms) / 1000.0,
                      static_cast<int>(sum_l / n),
                      static_cast<int>(sum_r / n),
                      static_cast<int>(0.5 * (sum_l + sum_r) / n),
                      pct(both250),
                      pct(ilim));
        Serial.printf("steer_avg=%+d (|steer|=%d) | z3_avg=%+d (|z3|=%d) | pos_avg=%+d (|pos|=%d)\n",
                      static_cast<int>(sum_steer / n),
                      static_cast<int>(sum_abs_steer / n),
                      static_cast<int>(sum_z3 / n),
                      static_cast<int>(sum_abs_z3 / n),
                      static_cast<int>(sum_pos / n),
                      static_cast<int>(sum_abs_pos / n));
        Serial.printf("State: TRK=%d%% CRS=%d%% STR=%d%% LST=%d%% SHRT=%d%%\n",
                      pct(st_cnt[0]), pct(st_cnt[1]), pct(st_cnt[2]), pct(st_cnt[3]), pct(st_cnt[4]));
        for (int k = 0; k < 4; ++k) {
            Serial.printf("IR S%d: min=%4lu avg=%4lu max=%4lu (span=%4lu)\n",
                          k + 1,
                          static_cast<unsigned long>(raw_min[k]),
                          static_cast<unsigned long>(raw_sum[k] / count_),
                          static_cast<unsigned long>(raw_max[k]),
                          static_cast<unsigned long>(raw_max[k] - raw_min[k]));
        }
        Serial.println("==================================================");
    }

    void dumpCsv(uint32_t step) const {
        if (count_ == 0) {
            Serial.println("[BB] Chua co du lieu!");
            return;
        }
        if (step < 1) step = 1;
        static const char* ST_NAME[5] = {"TRK", "CRS", "STR", "LST", "SHRT"};
        Serial.println("t_ms,st,crs,s1,s2,s3,s4,w1,w2,w3,w4,pos,z2,z3,steer,L,R,ilim");
        uint32_t printed = 0;
        for (uint32_t i = 0; i < count_; i += step) {
            const BlackboxFrame& fr = frameAt(i);
            const char* st = (fr.state < 5) ? ST_NAME[fr.state] : "AIR";
            uint8_t crs = fr.flags >> 1;
            uint8_t ilim = fr.flags & 1U;
            Serial.printf(
                "%u,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%u\n",
                fr.t_ms, st, crs,
                fr.raw[0], fr.raw[1], fr.raw[2], fr.raw[3],
                fr.w[0], fr.w[1], fr.w[2], fr.w[3],
                fr.pos_x100, fr.z2_x10, fr.z3_pwm, fr.steer, fr.l_pwm, fr.r_pwm, ilim
            );
            if ((++printed & 31U) == 0) {
                delay(2);
            }
        }
        Serial.printf("--- END BLACKBOX (%lu lines, step=%lu) ---\n",
                      static_cast<unsigned long>(printed),
                      static_cast<unsigned long>(step));
    }

    void clearAll() {
        head_ = 0;
        count_ = 0;
        unsaved_ = false;
        recording_ = false;
        if (fs_ready_ && LittleFS.exists(BB_FILE_PATH)) {
            LittleFS.remove(BB_FILE_PATH);
        }
        Serial.println("[BB] Da xoa sach du lieu Blackbox!");
    }

private:
    const BlackboxFrame& frameAt(uint32_t logical_idx) const {
        if (count_ < BB_MAX_FRAMES) return frames_[logical_idx];
        return frames_[(head_ + logical_idx) % BB_MAX_FRAMES];
    }

    void loadFromFlash() {
        if (!LittleFS.exists(BB_FILE_PATH)) return;
        File f = LittleFS.open(BB_FILE_PATH, FILE_READ);
        if (!f) return;

        BlackboxHeader hdr;
        if (f.read(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr)) == sizeof(hdr) &&
            hdr.magic == BB_MAGIC && hdr.count > 0 && hdr.count <= BB_MAX_FRAMES) {
            size_t need = hdr.count * sizeof(BlackboxFrame);
            if (f.read(reinterpret_cast<uint8_t*>(frames_), need) == need) {
                count_ = hdr.count;
                head_ = count_ % BB_MAX_FRAMES;
            }
        }
        f.close();
    }

    BlackboxFrame frames_[BB_MAX_FRAMES];
    uint32_t head_ = 0;
    uint32_t count_ = 0;
    uint32_t tick_div_ = 0;
    uint32_t run_start_ms_ = 0;
    bool recording_ = false;
    bool unsaved_ = false;
    bool fs_ready_ = false;
};

// ============================================================================
// RUN STATS & GLOBAL INSTANCES
// ============================================================================

struct RunStats {
    uint32_t ticks = 0;
    float sum_l = 0.0f;
    float sum_r = 0.0f;
    float sum_abs_steer = 0.0f;
    float sum_abs_z3 = 0.0f;
    float sum_speed = 0.0f;
    uint32_t full_both = 0;
    uint32_t limited = 0;
    uint32_t state[5] = {0, 0, 0, 0, 0};
    uint64_t loop_sum_us = 0;
    uint32_t loop_max_us = 0;
    uint32_t overruns = 0;

    void reset() { *this = RunStats(); }

    void add(const AppliedDrive& a, size_t st, float z3_pwm) {
        ticks++;
        sum_l += a.left_pwm;
        sum_r += a.right_pwm;
        sum_abs_steer += fabsf(a.applied_steer);
        sum_abs_z3 += fabsf(z3_pwm);
        sum_speed += 0.5f * (a.left_pwm + a.right_pwm);
        if (a.left_pwm >= 250.0f && a.right_pwm >= 250.0f) full_both++;
        if (a.current_limited) limited++;
        state[st]++;
    }

    void addLoopTime(uint32_t us) {
        loop_sum_us += us;
        if (us > loop_max_us) loop_max_us = us;
        if (us > LOOP_PERIOD_US) overruns++;
    }

    void print() const {
        float n = static_cast<float>(ticks > 0 ? ticks : 1);
        auto pct = [n](uint32_t c) -> int { return static_cast<int>((static_cast<float>(c) * 100.0f) / n); };
        Serial.printf(
            "[STATS] ticks=%lu (%lu ms) | loop avg=%luus max=%luus overrun=%d%% | L=%d R=%d avgPWM=%d |steer|=%d |z3|=%d | both>=250: %d%% | Ilimit: %d%% | TRK=%d%% CRS=%d%% STR=%d%% LST=%d%% SHRT=%d%%\n",
            static_cast<unsigned long>(ticks),
            static_cast<unsigned long>(ticks * (1000 / LOOP_FREQ_HZ)),
            static_cast<unsigned long>(loop_sum_us / (ticks > 0 ? ticks : 1)),
            static_cast<unsigned long>(loop_max_us),
            pct(overruns),
            static_cast<int>(sum_l / n),
            static_cast<int>(sum_r / n),
            static_cast<int>(sum_speed / n),
            static_cast<int>(sum_abs_steer / n),
            static_cast<int>(sum_abs_z3 / n),
            pct(full_both),
            pct(limited),
            pct(state[0]),
            pct(state[1]),
            pct(state[2]),
            pct(state[3]),
            pct(state[4])
        );
    }
};

static SensorProcessor g_sensors;
static LadrcController g_ctrl(ACTIVE_PROFILE);
static MotionPlanner g_planner(ACTIVE_PROFILE);
static MotorPowerController g_motors;
static RunStats g_stats;
static BlackboxRecorder g_bb;

static uint32_t g_tick_counter = 0;
static float g_est_forward_speed = 0.0f;
static uint32_t g_stopped_ticks = 0;
static bool g_inspect_mode = false;

// ============================================================================
// ARDUINO SETUP & 500 HZ REAL-TIME LOOP
// ============================================================================

void setup() {
    setCpuFrequencyMhz(80);
    Serial.begin(115200);

    pinMode(PIN_L_A, OUTPUT);
    pinMode(PIN_L_B, OUTPUT);
    pinMode(PIN_R_A, OUTPUT);
    pinMode(PIN_R_B, OUTPUT);
    digitalWrite(PIN_L_A, LOW);
    digitalWrite(PIN_L_B, LOW);
    digitalWrite(PIN_R_A, LOW);
    digitalWrite(PIN_R_B, LOW);

    pinMode(PIN_IR_ON, OUTPUT);
    digitalWrite(PIN_IR_ON, HIGH);
    pinMode(PIN_UNUSED_LED, OUTPUT);
    digitalWrite(PIN_UNUSED_LED, LOW);

    pinMode(PIN_BOOT_BTN, INPUT_PULLUP);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    g_bb.begin();
    g_bb.printStartupHint();

    g_sensors.resetAll();
    uint32_t settle_ticks = (STARTUP_SETTLE_MS * LOOP_FREQ_HZ) / 1000;
    uint32_t boot_low_count = 0;
    uint32_t countdown_start_ms = millis();

    if (START_DELAY_MS >= 1500) {
        uint32_t pre_settle_ms = (START_DELAY_MS > (STARTUP_SETTLE_MS + 250))
            ? (START_DELAY_MS - STARTUP_SETTLE_MS - 250)
            : 0;
        if (ENABLE_START_BEEPS) {
            if (START_DELAY_MS >= 6000) {
                playMelody(ASTRONOMIA_SONG, sizeof(ASTRONOMIA_SONG) / sizeof(ASTRONOMIA_SONG[0]));
            }
            const uint32_t ready_t[3] = {
                (START_DELAY_MS > 3000) ? (START_DELAY_MS - 3000) : 0,
                (START_DELAY_MS > 2000) ? (START_DELAY_MS - 2000) : 0,
                (START_DELAY_MS > 1000) ? (START_DELAY_MS - 1000) : 0,
            };
            const uint32_t ready_f[3] = {1480, 1760, 2217};
            for (int i = 0; i < 3; ++i) {
                uint32_t elapsed_now = static_cast<uint32_t>(millis() - countdown_start_ms);
                if (ready_t[i] < pre_settle_ms && elapsed_now <= ready_t[i]) {
                    while (static_cast<uint32_t>(millis() - countdown_start_ms) < ready_t[i]) {
                        g_bb.pollSerial(g_inspect_mode);
                        if (g_inspect_mode) break;
                    }
                    if (g_inspect_mode) break;
                    motorBeep(ready_f[i], 160);
                }
            }
        }

        while (!g_inspect_mode && static_cast<uint32_t>(millis() - countdown_start_ms) < pre_settle_ms) {
            g_bb.pollSerial(g_inspect_mode);
        }

        while (g_inspect_mode) {
            g_bb.pollSerial(g_inspect_mode);
            delay(5);
        }

        for (uint32_t t = 0; t < settle_ticks; ++t) {
            uint32_t tick_start = micros();
            uint16_t raw[4];
            readIrMedian3(raw);
            g_sensors.seedStartupSample(raw);
            if (ENABLE_BOOT_SWEEP_CALIB && digitalRead(PIN_BOOT_BTN) == LOW) {
                boot_low_count++;
            }
            while (static_cast<uint32_t>(micros() - tick_start) < LOOP_PERIOD_US) {
            }
        }

        if (ENABLE_START_BEEPS) {
            motorBeep(2960, 240);
            delayMsBusy(10);
        }
    } else {
        for (uint32_t t = 0; t < settle_ticks; ++t) {
            uint32_t tick_start = micros();
            uint16_t raw[4];
            readIrMedian3(raw);
            g_sensors.seedStartupSample(raw);
            if (ENABLE_BOOT_SWEEP_CALIB && digitalRead(PIN_BOOT_BTN) == LOW) {
                boot_low_count++;
            }
            while (static_cast<uint32_t>(micros() - tick_start) < LOOP_PERIOD_US) {
            }
        }
        while (!g_inspect_mode && static_cast<uint32_t>(millis() - countdown_start_ms) < START_DELAY_MS) {
            g_bb.pollSerial(g_inspect_mode);
        }
        while (g_inspect_mode) {
            g_bb.pollSerial(g_inspect_mode);
            delay(5);
        }
        if (ENABLE_START_BEEPS) {
            motorBeep(2093, 80);
        }
    }

    g_motors.begin();

    if (ENABLE_TELEMETRY) {
        Serial.println("=== ESP32-C3 Line Follower (LADRC Fast-Decay + Blackbox / 500Hz) ===");
        Serial.printf(
            "v_max=%d | v_curve=%d | wc=%d | wo=%d | b0=%d | max_z3=%d\n",
            static_cast<int>(ACTIVE_PROFILE.v_max),
            static_cast<int>(ACTIVE_PROFILE.v_curve_min),
            static_cast<int>(ACTIVE_PROFILE.wc * 10.0f),
            static_cast<int>(ACTIVE_PROFILE.wo * 10.0f),
            static_cast<int>(ACTIVE_PROFILE.b0 * 10.0f),
            static_cast<int>(ACTIVE_PROFILE.max_z3_pwm)
        );
    }

    if (ENABLE_BOOT_SWEEP_CALIB && boot_low_count > (settle_ticks / 3)) {
        g_sensors.sweepCalibrateReset();
        uint32_t wait_rel = millis();
        while (digitalRead(PIN_BOOT_BTN) == LOW && static_cast<uint32_t>(millis() - wait_rel) < 800) {
        }

        uint32_t sweep_ticks = (2400 * LOOP_FREQ_HZ) / 1000;
        for (uint32_t t = 0; t < sweep_ticks; ++t) {
            uint32_t tick_start = micros();
            uint16_t raw[4];
            readIrMedian3(raw);
            g_sensors.sweepCalibrateSample(raw);

            uint32_t phase = (t * 4) / sweep_ticks;
            float l_cmd = (phase == 1 || phase == 2) ? 72.0f : -72.0f;
            float r_cmd = (phase == 1 || phase == 2) ? -72.0f : 72.0f;
            g_motors.driveCalibration(l_cmd, r_cmd);
            while (static_cast<uint32_t>(micros() - tick_start) < LOOP_PERIOD_US) {
            }
        }

        g_motors.coastStop();
        g_sensors.sweepCalibrateFinish();

        uint32_t pause_start = millis();
        while (static_cast<uint32_t>(millis() - pause_start) < 1200) {
            if (digitalRead(PIN_BOOT_BTN) == LOW) break;
        }
        g_motors.reset();
    }

    g_ctrl.reset();
    g_planner.reset();
}

void loop() {
    if (g_inspect_mode) {
        g_motors.coastStop();
        g_bb.pollSerial(g_inspect_mode);
        delay(5);
        return;
    }

    uint32_t tick_start = micros();

    uint16_t raw[4];
    readIrMedian3(raw);

    LineState line_state = g_sensors.update(raw, g_ctrl.velocityHint());
    DriveCommand drive_cmd = g_planner.compute(line_state, g_sensors.weights(), g_ctrl, g_est_forward_speed);

    bool has_applied = false;
    AppliedDrive applied = {0.0f, 0.0f, 0.0f, 0.0f, false};

    if (drive_cmd.safety_stopped) {
        if (line_state.kind == LineKind::Lifted) {
            g_motors.reset();
        } else {
            g_motors.coastStop();
        }
        g_ctrl.feedbackAppliedSteer(0.0f);
        g_est_forward_speed = 0.0f;
    } else {
        applied = g_motors.apply(drive_cmd);
        g_ctrl.feedbackAppliedSteer(applied.applied_steer);
        g_est_forward_speed = applied.est_forward_speed;
        has_applied = true;
    }

    size_t st = 0;
    if (g_planner.isInShortcut()) {
        st = 4;
    } else {
        switch (line_state.kind) {
            case LineKind::Tracking: st = 0; break;
            case LineKind::Crossing: st = 1; break;
            case LineKind::CenterStraddle: st = 2; break;
            case LineKind::Lost:
            case LineKind::Lifted: st = 3; break;
        }
    }

    if (has_applied) {
        if (g_stopped_ticks > 0) {
            g_stats.reset();
            g_bb.beginRun();
            g_stopped_ticks = 0;
        }
        if (ENABLE_RUN_STATS) {
            g_stats.add(applied, st, g_ctrl.z3PwmEst());
        }
        if (ENABLE_BLACKBOX) {
            g_bb.record(
                line_state,
                g_sensors.lastRaw(),
                g_sensors.weights(),
                applied,
                g_ctrl.velocityHint(),
                g_ctrl.z3PwmEst(),
                static_cast<uint8_t>(st),
                g_planner.crossCount()
            );
        }
    } else {
        if (g_stopped_ticks < UINT32_MAX) g_stopped_ticks++;

        uint32_t save_ticks = (BB_SAVE_DELAY_MS * LOOP_FREQ_HZ) / 1000;
        if (ENABLE_BLACKBOX && g_stopped_ticks == save_ticks && g_bb.isRecording()) {
            g_bb.finishRunAndSave();
        }

        if (ENABLE_BLACKBOX && (g_stopped_ticks % 10) == 0) {
            g_bb.pollSerial(g_inspect_mode);
        }

        if (ENABLE_RUN_STATS) {
            uint32_t stats_print_ticks = (RUN_STATS_PRINT_MS * LOOP_FREQ_HZ) / 1000;
            if (stats_print_ticks < 1) stats_print_ticks = 1;
            if (g_stats.ticks > 0 && (g_stopped_ticks % stats_print_ticks) == 0) {
                g_stats.print();
            }
        }
    }

    g_tick_counter++;
    uint32_t telemetry_ticks = (TELEMETRY_INTERVAL_MS * LOOP_FREQ_HZ) / 1000;
    if (telemetry_ticks < 1) telemetry_ticks = 1;
    if (ENABLE_TELEMETRY && (g_tick_counter % telemetry_ticks) == 0) {
        const uint16_t* r = g_sensors.lastRaw();
        const float* w = g_sensors.weights();
        const char* tag = "TRK";
        int pos_x100 = 0;
        if (g_planner.isInShortcut()) {
            tag = "SHRT";
        } else {
            switch (line_state.kind) {
                case LineKind::Tracking:
                    tag = "TRK";
                    pos_x100 = static_cast<int>(line_state.pos * 100.0f);
                    break;
                case LineKind::Crossing:
                    tag = "CRS";
                    break;
                case LineKind::CenterStraddle:
                    tag = "STR";
                    pos_x100 = static_cast<int>(line_state.pos * 100.0f);
                    break;
                case LineKind::Lost:
                    tag = "LST";
                    pos_x100 = static_cast<int>(line_state.dir * 135.0f);
                    break;
                case LineKind::Lifted:
                    tag = "AIR";
                    break;
            }
        }
        int al = has_applied ? static_cast<int>(applied.left_pwm) : 0;
        int ar = has_applied ? static_cast<int>(applied.right_pwm) : 0;
        Serial.printf(
            "[%s#%lu] raw=[%u,%u,%u,%u] w=[%d,%d,%d,%d] pos=%d z3=%d L=%d R=%d v=%d\n",
            tag,
            static_cast<unsigned long>(g_planner.crossCount()),
            r[0], r[1], r[2], r[3],
            static_cast<int>(w[0] * 100.0f),
            static_cast<int>(w[1] * 100.0f),
            static_cast<int>(w[2] * 100.0f),
            static_cast<int>(w[3] * 100.0f),
            pos_x100,
            static_cast<int>(g_ctrl.z3PwmEst()),
            al,
            ar,
            static_cast<int>(g_est_forward_speed)
        );
    }

    if (ENABLE_RUN_STATS && g_stopped_ticks == 0) {
        g_stats.addLoopTime(static_cast<uint32_t>(micros() - tick_start));
    }
    while (static_cast<uint32_t>(micros() - tick_start) < LOOP_PERIOD_US) {
    }
}
