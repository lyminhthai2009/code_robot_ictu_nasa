/**
 * ============================================================================
 *  LADRC Line Follower (ESP32-C3 / 500 Hz / 4x AA Zero-Brownout)
 *  1-to-1 C++ Port from Verified Rust Firmware (`ladrc_robot`)
 * ============================================================================
 *  Hardware (Verified on Real COBOT ESP32-C3 Kit):
 *    - MCU:        ESP32-C3 @ 80 MHz (Wi-Fi/BLE OFF => ~40% lower 3.3V draw)
 *    - Power:      4x AA (max 7.4V) -> AMS1117-3.3 LDO
 *    - Motors:     2x GA12-N20 5V 300 RPM (no encoder), LEDC 5 kHz 8-bit (0..255)
 *                    Left  Motor: GPIO 2  (Ch0 = A), GPIO 10 (Ch1 = B)
 *                    Right Motor: GPIO 6  (Ch2 = A), GPIO 5  (Ch3 = B)
 *                    Forward: A=pwm, B=0 | Reverse: A=0, B=pwm | Coast: A=0, B=0
 *    - IR Array:   4-IR Isosceles Trapezoid (S1..S4 = 40.0 mm, S2..S3 = 22.5 mm)
 *                    S1 (Outer Left):  GPIO 0  (ADC1, 12-bit, 11 dB)
 *                    S2 (Inner Left):  GPIO 1  (ADC1, 12-bit, 11 dB)
 *                    S3 (Inner Right): GPIO 3  (ADC1, 12-bit, 11 dB)
 *                    S4 (Outer Right): GPIO 4  (ADC1, 12-bit, 11 dB)
 *                    IR_LED_ON:        GPIO 21 (HIGH = ON)
 *                    BOOT Button:      GPIO 9  (covered on chassis -> disabled)
 *
 *  Compatibility:
 *    - ESP32 Arduino Core 3.x (tested with 3.3.12) AND Core 2.x
 *    - PlatformIO (`framework = arduino`) or copy-paste directly into Arduino IDE `.ino`
 * ============================================================================
 */

#include <Arduino.h>
#include <cmath>
#include <cstdint>
#include <cstddef>

// ============================================================================
//  0. SMALL MATH HELPERS (ZERO DEPENDENCY, C++11/17/20 SAFE)
// ============================================================================

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline float minf(float a, float b) {
  return (a < b) ? a : b;
}

static inline float maxf(float a, float b) {
  return (a > b) ? a : b;
}

static inline uint32_t sat_add_u32(uint32_t a, uint32_t b) {
  uint32_t res = a + b;
  return (res < a) ? UINT32_MAX : res;
}

// ============================================================================
//  1. CENTRAL CONFIGURATION (1-to-1 FROM `src/config.rs`)
// ============================================================================

/// Control loop frequency (Hz). 500 Hz (Ts = 2.0 ms).
static constexpr uint32_t LOOP_FREQ_HZ   = 500;
static constexpr float    DT_SEC         = 1.0f / static_cast<float>(LOOP_FREQ_HZ);
static constexpr uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_FREQ_HZ;

// --- Hardware Pin Map (Verified) ---
static constexpr uint8_t PIN_MOTOR_L_A = 2;   // LEDC Ch0
static constexpr uint8_t PIN_MOTOR_L_B = 10;  // LEDC Ch1
static constexpr uint8_t PIN_MOTOR_R_A = 6;   // LEDC Ch2
static constexpr uint8_t PIN_MOTOR_R_B = 5;   // LEDC Ch3

static constexpr uint8_t LEDC_CH_L_A   = 0;
static constexpr uint8_t LEDC_CH_L_B   = 1;
static constexpr uint8_t LEDC_CH_R_A   = 2;
static constexpr uint8_t LEDC_CH_R_B   = 3;

static constexpr uint8_t PIN_IR_S1     = 0;   // Outer Left  (L2)
static constexpr uint8_t PIN_IR_S2     = 1;   // Inner Left  (L1)
static constexpr uint8_t PIN_IR_S3     = 3;   // Inner Right (R1)
static constexpr uint8_t PIN_IR_S4     = 4;   // Outer Right (R2)
static constexpr uint8_t PIN_IR_LED_ON = 21;  // IR Emitter Enable (HIGH = ON)
static constexpr uint8_t PIN_BOOT_BTN  = 9;   // BOOT Button (active LOW)

// --- Verified Motor & Sensor Polarity / Orientation ---
static constexpr bool  SWAP_MOTORS_LR     = false;
static constexpr bool  INVERT_LEFT_MOTOR  = false;
static constexpr bool  INVERT_RIGHT_MOTOR = false;
static constexpr float RIGHT_MOTOR_TRIM   = 1.00f;

/// LEDC PWM specification: 5 kHz, 8-bit (0..255).
static constexpr uint32_t PWM_FREQ_HZ  = 5000;
static constexpr uint8_t  PWM_RES_BITS = 8;
static constexpr float    PWM_MAX      = 255.0f;

/// Sensor polarity: White = HIGH ADC (~4095), Black = LOW ADC (~1900-2050).
static constexpr bool  IR_WHITE_IS_HIGH  = true;
static constexpr float INIT_WHITE_ADC    = 4095.0f;
static constexpr float INIT_BLACK_ADC    = 1950.0f;

/// Verified on hardware: GPIO0=S1(L2), GPIO1=S2(L1), GPIO3=S3(R1), GPIO4=S4(R2).
static constexpr bool  MIRROR_SENSORS_LR = false;

// --- Trapezoid Sensor Geometry (from cautruccambien.png) ---
// Long base  (S1 <-> S4): 40.0 mm => x = +-20.00 mm (normalized +-1.0000)
// Short base (S2 <-> S3): 22.5 mm => x = +-11.25 mm (normalized +-0.5625)
// Forward lookahead of S2, S3 = 4.84 mm ahead of S1, S4.
static constexpr float SENSOR_POS_NORM[4] = { -1.0000f, -0.5625f, 0.5625f, 1.0000f };

/// Regularized denominator `max(sum_w, SENSOR_DENOM_FLOOR)` eliminates the
/// discontinuous -0.56 <-> +0.56 relay jump across the 22.5 mm S2-S3 center gap!
static constexpr float SENSOR_DENOM_FLOOR = 1.05f;

/// Continuous extrapolation when ONLY outer S1/S4 sees a fading line edge in a tight curve.
static constexpr float OUTER_EDGE_EXTRAP  = 0.25f;

// --- Online Arena Adaptation & Variable Line Width Thresholds ---
static constexpr float MIN_CAL_SPAN          = 450.0f;
static constexpr float CAL_EXPAND_ALPHA      = 0.14f;
/// CRITICAL: Keep at 0.0! Relaxing black_ref upward on partial line edges shrinks span!
static constexpr float CAL_BLACK_RELAX_ALPHA = 0.0f;

static constexpr float BLACK_NOISE_FLOOR  = 0.06f;
static constexpr float BLACK_SAT_CEIL     = 0.92f;
static constexpr float COMMON_MODE_REJECT = 0.85f;

static constexpr float LINE_PRESENT_SUM   = 0.10f;

/// Bridge-proof, 5mm-bump-proof & arena-invariant X-Crossing detection:
static constexpr float    CROSSING_MIN_ALL_BLACK    = 0.77f;
static constexpr float    CROSSING_MIN_SUM          = 3.55f;
static constexpr float    CROSSING_MAX_SPREAD_RATIO = 0.225f;
static constexpr float    LOCAL_CONTRAST_SPAN_MIN   = 420.0f;

/// Lifted-in-air thresholds (stricter uniformity <= 14% span so dark Bridge NEVER triggers AIR!).
static constexpr float    AIR_MIN_ALL_BLACK         = 0.82f;
static constexpr float    AIR_MAX_SPREAD_RATIO      = 0.14f;

/// 10 ms confirmation (5 ticks): rejects 5 mm speed-bump glitches while catching X-crossings.
static constexpr uint32_t CROSS_CONFIRM_MS          = 10;
static constexpr uint32_t CROSS_COOLDOWN_MS         = 650;
static constexpr uint32_t MAX_CROSS_LOCK_MS         = 130;

/// Action to take at each X-intersection encountered along the track.
enum class CrossingAction : uint8_t {
  Straight,       ///< Go arrow-straight across the X-intersection
  ShortcutRight,  ///< Decisive, zero-overshoot, zero-brownout RIGHT shortcut
  ShortcutLeft,   ///< Symmetric LEFT shortcut turn
};

/// Competition Crossing Plan (indexed by 1st, 2nd, 3rd... X-intersection):
/// Vong bang & Vong chung ket: [Straight, Straight, ShortcutRight, Straight, Straight, Straight]
static constexpr size_t CROSSING_PLAN_LEN = 6;
static constexpr CrossingAction CROSSING_PLAN[CROSSING_PLAN_LEN] = {
  CrossingAction::Straight,
  CrossingAction::Straight,
  CrossingAction::ShortcutRight,
  CrossingAction::Straight,
  CrossingAction::Straight,
  CrossingAction::Straight,
};

/// 3-Phase Shortcut Snap-Pivot + Counter-Brake parameters:
static constexpr float    SHORTCUT_BRAKE_OUTER_PWM      = 225.0f;
static constexpr uint32_t SHORTCUT_BRAKE_MS             = 28;
static constexpr float    SHORTCUT_OUTER_PWM            = 210.0f;
static constexpr float    SHORTCUT_INNER_REV_PWM        = 160.0f;
static constexpr float    SHORTCUT_COUNTER_BRAKE_PWM    = 195.0f;
static constexpr uint32_t SHORTCUT_COUNTER_BRAKE_MIN_MS = 24;
static constexpr uint32_t SHORTCUT_COUNTER_BRAKE_MAX_MS = 75;
static constexpr float    SHORTCUT_ANTI_PLUG_TH         = 185.0f;
static constexpr uint32_t SHORTCUT_LEAVE_MIN_MS         = 22;
static constexpr uint32_t SHORTCUT_LEAVE_MAX_MS         = 320;
static constexpr uint32_t SHORTCUT_ACQUIRE_MAX_MS       = 400;

/// Lifted-in-air safety detection (all 4 sensors continuously dark > 240 ms => auto-stop!).
static constexpr uint32_t LIFTED_STOP_MS = 240;

/// Thin-line center-straddle & red START/FINISH bar bridging.
static constexpr float    STRADDLE_POS_LIMIT = 0.42f;
static constexpr uint32_t MAX_STRADDLE_MS    = 140;

/// Safety auto-stop if completely off the track (all white) for > 2.0 seconds.
static constexpr uint32_t LOST_SAFETY_STOP_MS = 2000;

// --- Speed & LADRC Profiles ---
struct TuningProfile {
  float v_max;
  float v_curve_min;
  float max_wheel_pwm;
  float v_crossing;
  float v_straddle;
  float omega_c;
  float zeta;
  float omega_o_calm;
  float omega_o_agile;
  float b0_base;
  float b0_speed_gain;
  float max_steer_pwm;
  float max_reverse_tracking;
  float recovery_outer_pwm;
  float recovery_inner_pwm;
  float slow_k_err;
  float slow_k_vel;
  float slow_k_curv;
};

/// Safe AA profile: fast & extra gentle on older AA batteries.
static constexpr TuningProfile PROFILE_SAFE = {
  225.0f, 135.0f, 235.0f, 195.0f, 190.0f,
  14.0f, 1.65f, 32.0f, 44.0f,
  3.70f, 2.20f, 205.0f, 0.0f,
  165.0f, 0.0f,
  0.36f, 0.028f, 0.36f
};

/// Race 255 MAX profile: smooth 255 straight + 185 curve floor.
static constexpr TuningProfile PROFILE_RACE = {
  255.0f, 185.0f, 255.0f, 250.0f, 248.0f,
  16.2f, 1.52f, 38.0f, 52.0f,
  3.45f, 2.05f, 240.0f, 0.0f,
  205.0f, 0.0f,
  0.34f, 0.025f, 0.32f
};

/// Turbo MAX-255 profile (ACTIVE): pinned 255 on straights & sweeps, 215+ in hairpins!
static constexpr TuningProfile PROFILE_TURBO = {
  255.0f, 215.0f, 255.0f, 255.0f, 255.0f,
  17.0f, 1.50f, 40.0f, 56.0f,
  3.40f, 2.05f, 255.0f, 0.0f,
  225.0f, 0.0f,
  0.28f, 0.020f, 0.26f
};

/// Select the active driving profile here! (PROFILE_SAFE / PROFILE_RACE / PROFILE_TURBO)
static constexpr TuningProfile ACTIVE_PROFILE = PROFILE_TURBO;

// --- 4x AA Zero-Brownout Power Shield (MAX-255 Tuned) ---
static constexpr uint32_t SOFT_START_MS            = 450;
static constexpr float    MOTOR_MECH_TAU_S         = 0.105f;
static constexpr float    COMMON_ACCEL_PWM_PER_SEC = 620.0f;
static constexpr float    COMMON_BRAKE_PWM_PER_SEC = 3400.0f;
static constexpr float    STEER_SLEW_PWM_PER_SEC   = 4800.0f;
static constexpr float    BATTERY_CURRENT_BUDGET   = 135.0f;
static constexpr float    MAX_TOTAL_PWM            = 510.0f;
static constexpr float    ANTI_PLUG_OMEGA_TH       = 30.0f;
static constexpr float    MOTOR_DEADZONE_PWM       = 12.0f;

// --- Startup, Calibration & Telemetry ---
static constexpr uint32_t STARTUP_SETTLE_MS       = 280;
/// BOOT button is covered on this chassis -> disabled; arena adaptation is 100% automatic!
static constexpr bool     ENABLE_BOOT_SWEEP_CALIB = false;

/// Race-ready: `false` saves CPU & 3.3V power (set `true` for 10 Hz Serial debug).
static constexpr bool     ENABLE_TELEMETRY        = false;
static constexpr uint32_t TELEMETRY_INTERVAL_MS   = 100;

// ============================================================================
//  2. 4-IR TRAPEZOID SENSOR PROCESSOR (1-to-1 FROM `src/sensor.rs`)
// ============================================================================

enum class LineKind : uint8_t {
  Tracking,
  Crossing,
  CenterStraddle,
  Lost,
  Lifted,
};

struct LineState {
  LineKind kind;
  float    pos;           // Tracking: pos | CenterStraddle: pos_hold | Lost: dir
  float    strength;      // Tracking: sum_w
  uint32_t elapsed_ms;    // Crossing / CenterStraddle / Lost
  bool     is_new_event;  // Crossing
  bool     wings_clear;   // Crossing

  static LineState makeTracking(float pos, float strength) {
    return { LineKind::Tracking, pos, strength, 0, false, false };
  }
  static LineState makeCrossing(uint32_t elapsed_ms, bool is_new_event, bool wings_clear) {
    return { LineKind::Crossing, 0.0f, 0.0f, elapsed_ms, is_new_event, wings_clear };
  }
  static LineState makeCenterStraddle(float pos_hold, uint32_t elapsed_ms) {
    return { LineKind::CenterStraddle, pos_hold, 0.0f, elapsed_ms, false, false };
  }
  static LineState makeLost(float dir, uint32_t elapsed_ms) {
    return { LineKind::Lost, dir, 0.0f, elapsed_ms, false, false };
  }
  static LineState makeLifted() {
    return { LineKind::Lifted, 0.0f, 0.0f, 0, false, false };
  }
};

class SensorProcessor {
public:
  SensorProcessor()
    : arena_seeded_(false),
      early_run_ticks_(0),
      early_min_black_(4095.0f),
      early_black_locked_(false),
      last_valid_pos_(0.0f),
      last_side_(1.0f),
      line_width_ema_(1.0f),
      all_dark_ticks_(0),
      cross_confirm_ticks_(0),
      cross_cooldown_ticks_(0),
      straddle_ticks_(0),
      lost_ticks_(0) {
    for (int i = 0; i < 4; ++i) {
      white_ref_[i] = INIT_WHITE_ADC;
      black_ref_[i] = INIT_BLACK_ADC;
      weights_[i]   = 0.0f;
      last_raw_[i]  = static_cast<uint16_t>(INIT_WHITE_ADC);
    }
  }

  /// Median of 3 raw ADC readings: completely eliminates PWM switching & optical glitches.
  static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    uint16_t max_ab = (a > b) ? a : b;
    uint16_t min_ab = (a < b) ? a : b;
    if (c > max_ab) return max_ab;
    if (c < min_ab) return min_ab;
    return c;
  }

  static inline void orderChannels(const uint16_t raw_gpio[4], uint16_t out[4]) {
    if (MIRROR_SENSORS_LR) {
      out[0] = raw_gpio[3];
      out[1] = raw_gpio[2];
      out[2] = raw_gpio[1];
      out[3] = raw_gpio[0];
    } else {
      out[0] = raw_gpio[0];
      out[1] = raw_gpio[1];
      out[2] = raw_gpio[2];
      out[3] = raw_gpio[3];
    }
  }

  /// Buttonless instant arena-baseline seeding during startup settle.
  void seedStartupSample(const uint16_t raw_gpio[4]) {
    uint16_t ordered[4];
    orderChannels(raw_gpio, ordered);
    for (int i = 0; i < 4; ++i) last_raw_[i] = ordered[i];

    uint16_t brightest = 0;
    uint16_t darkest   = 4095;
    for (int i = 0; i < 4; ++i) {
      if (ordered[i] > brightest) brightest = ordered[i];
      if (ordered[i] < darkest)   darkest   = ordered[i];
    }
    float b_f = static_cast<float>(brightest);
    float d_f = static_cast<float>(darkest);

    if (IR_WHITE_IS_HIGH && b_f > 1400.0f) {
      if (!arena_seeded_) {
        for (int i = 0; i < 4; ++i) white_ref_[i] = b_f;
        if (b_f - d_f > MIN_CAL_SPAN) {
          for (int i = 0; i < 4; ++i) black_ref_[i] = d_f;
          early_black_locked_ = true;
        } else {
          float b_est = minf(b_f * 0.45f, INIT_BLACK_ADC);
          for (int i = 0; i < 4; ++i) black_ref_[i] = b_est;
        }
        arena_seeded_ = true;
      } else {
        for (int i = 0; i < 4; ++i) {
          float v = static_cast<float>(ordered[i]);
          if (b_f > white_ref_[i]) {
            white_ref_[i] = 0.70f * white_ref_[i] + 0.30f * b_f;
          } else {
            white_ref_[i] = 0.92f * white_ref_[i] + 0.08f * maxf(b_f, v);
          }
          if (b_f - d_f > MIN_CAL_SPAN) {
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
      } else {
        if (black_ref_[i] - white_ref_[i] < MIN_CAL_SPAN) {
          white_ref_[i] = INIT_BLACK_ADC;
          black_ref_[i] = INIT_WHITE_ADC;
        }
      }
    }
  }

  /// Process one 500 Hz sensor frame and return the classified `LineState`.
  LineState update(const uint16_t raw_gpio[4], float eso_vel_hint) {
    uint16_t ordered[4];
    orderChannels(raw_gpio, ordered);
    for (int i = 0; i < 4; ++i) last_raw_[i] = ordered[i];

    uint16_t frame_max = ordered[0];
    uint16_t frame_min = ordered[0];
    for (int i = 1; i < 4; ++i) {
      if (ordered[i] > frame_max) frame_max = ordered[i];
      if (ordered[i] < frame_min) frame_min = ordered[i];
    }
    float fmax   = static_cast<float>(frame_max);
    float fmin   = static_cast<float>(frame_min);
    float spread = fmax - fmin;

    // Instant arena seeding on first floor contact after power-on in air or pick-up/put-down
    if (IR_WHITE_IS_HIGH && !arena_seeded_ && fmax > 2200.0f && spread > MIN_CAL_SPAN) {
      for (int i = 0; i < 4; ++i) {
        white_ref_[i] = fmax;
        black_ref_[i] = fmin;
      }
      arena_seeded_       = true;
      early_black_locked_ = true;
    }

    bool has_valid_surface_contrast =
      IR_WHITE_IS_HIGH && (fmax > 2200.0f) && (spread > MIN_CAL_SPAN);

    // 1. Safe Online Arena Adaptation (Buttonless, zero upward drift in curves)
    float raw_black_glob[4];
    float avg_span = 0.0f;
    for (int i = 0; i < 4; ++i) {
      float v = static_cast<float>(ordered[i]);
      if (IR_WHITE_IS_HIGH) {
        if (v > white_ref_[i]) {
          white_ref_[i] += CAL_EXPAND_ALPHA * (v - white_ref_[i]);
        }
        if (has_valid_surface_contrast) {
          bool is_dark_ch = (v < fmax - 0.65f * spread);
          if (is_dark_ch && v < black_ref_[i] && v > 250.0f) {
            black_ref_[i] += CAL_EXPAND_ALPHA * (v - black_ref_[i]);
          }
        }
        float span = maxf(white_ref_[i] - black_ref_[i], MIN_CAL_SPAN);
        avg_span += span;
        raw_black_glob[i] = clampf((white_ref_[i] - v) / span, 0.0f, 1.0f);
      } else {
        if (v < white_ref_[i]) {
          white_ref_[i] += CAL_EXPAND_ALPHA * (v - white_ref_[i]);
        }
        if (v > black_ref_[i]) {
          black_ref_[i] += CAL_EXPAND_ALPHA * (v - black_ref_[i]);
        }
        float span = maxf(black_ref_[i] - white_ref_[i], MIN_CAL_SPAN);
        avg_span += span;
        raw_black_glob[i] = clampf((v - white_ref_[i]) / span, 0.0f, 1.0f);
      }
    }
    avg_span *= 0.25f;

    // One-shot early-run glossy-black lock (runs ONLY once in first 300 ms on main white mat)
    if (IR_WHITE_IS_HIGH && !early_black_locked_ && has_valid_surface_contrast) {
      float avg_white = 0.25f * (white_ref_[0] + white_ref_[1] + white_ref_[2] + white_ref_[3]);
      if (fmax >= 0.88f * avg_white) {
        if (fmin < early_min_black_) {
          early_min_black_ = fmin;
        }
        early_run_ticks_ = sat_add_u32(early_run_ticks_, 1);
        if (early_run_ticks_ >= 150) {
          if (early_min_black_ < avg_white - MIN_CAL_SPAN) {
            for (int i = 0; i < 4; ++i) {
              if (early_min_black_ > black_ref_[i] + 120.0f) {
                black_ref_[i] = early_min_black_;
              }
            }
          }
          early_black_locked_ = true;
        }
      }
    }

    // 2. Bridge-Proof, 5mm-Bump-Proof & Arena-Invariant X-Crossing & Air Detector
    uint32_t tick_ms            = 1000 / LOOP_FREQ_HZ;
    uint32_t confirm_needed     = (CROSS_CONFIRM_MS / tick_ms > 1) ? (CROSS_CONFIRM_MS / tick_ms) : 1;
    uint32_t cooldown_ticks_max = (CROSS_COOLDOWN_MS / tick_ms > 1) ? (CROSS_COOLDOWN_MS / tick_ms) : 1;

    float raw_sum_glob =
      raw_black_glob[0] + raw_black_glob[1] + raw_black_glob[2] + raw_black_glob[3];
    if (cross_cooldown_ticks_ > 0) {
      cross_cooldown_ticks_ -= 1;
    }

    float min_b_glob = minf(
      minf(raw_black_glob[0], raw_black_glob[1]),
      minf(raw_black_glob[2], raw_black_glob[3])
    );

    bool is_air_dark =
      (min_b_glob >= AIR_MIN_ALL_BLACK) && (spread <= AIR_MAX_SPREAD_RATIO * avg_span);

    bool is_crossing_raw =
      (min_b_glob >= CROSSING_MIN_ALL_BLACK) &&
      (raw_sum_glob >= CROSSING_MIN_SUM) &&
      (spread <= CROSSING_MAX_SPREAD_RATIO * avg_span);

    // 3. LOCAL-CONTRAST TRACKING NORMALIZATION (100% Immune to Bridge Bg & 5mm Speed Bumps!)
    float inv_scale = 1.0f / (BLACK_SAT_CEIL - BLACK_NOISE_FLOOR);
    float sum_w = 0.0f;
    for (int i = 0; i < 4; ++i) {
      float v = static_cast<float>(ordered[i]);
      float lb;
      if (IR_WHITE_IS_HIGH) {
        float local_span = maxf(fmax - black_ref_[i], LOCAL_CONTRAST_SPAN_MIN);
        lb = clampf((fmax - v) / local_span, 0.0f, 1.0f);
      } else {
        float local_span = maxf(black_ref_[i] - fmin, LOCAL_CONTRAST_SPAN_MIN);
        lb = clampf((v - fmin) / local_span, 0.0f, 1.0f);
      }
      float clean = clampf((lb - BLACK_NOISE_FLOOR) * inv_scale, 0.0f, 1.0f);
      weights_[i] = clean;
      sum_w += clean;
    }

    // 4. Lifted-In-Air Auto-Stop vs Confirmed X-Crossing Event
    if (is_air_dark) {
      all_dark_ticks_ = sat_add_u32(all_dark_ticks_, 1);
      if (all_dark_ticks_ * tick_ms >= LIFTED_STOP_MS) {
        cross_confirm_ticks_  = 0;
        cross_cooldown_ticks_ = 0;
        straddle_ticks_       = 0;
        lost_ticks_           = 0;
        // Re-arm buttonless arena seeding for next put-down!
        arena_seeded_         = false;
        early_black_locked_   = false;
        early_run_ticks_      = 0;
        early_min_black_      = 4095.0f;
        return LineState::makeLifted();
      }
    } else {
      all_dark_ticks_ = 0;
    }

    if (is_crossing_raw) {
      cross_confirm_ticks_ = sat_add_u32(cross_confirm_ticks_, 1);
      if (cross_confirm_ticks_ >= confirm_needed) {
        straddle_ticks_ = 0;
        lost_ticks_     = 0;
        bool is_new_event = false;
        if (cross_cooldown_ticks_ == 0) {
          cross_cooldown_ticks_ = cooldown_ticks_max;
          is_new_event = true;
        }
        bool wings_clear = (weights_[0] < 0.25f) && (weights_[3] < 0.25f);
        return LineState::makeCrossing(cross_confirm_ticks_ * tick_ms, is_new_event, wings_clear);
      }
    } else {
      cross_confirm_ticks_ = 0;
    }

    // 5. Normal Tracking State
    if (sum_w >= LINE_PRESENT_SUM) {
      straddle_ticks_ = 0;
      lost_ticks_     = 0;

      float weighted_sum = weights_[0] * SENSOR_POS_NORM[0]
                         + weights_[1] * SENSOR_POS_NORM[1]
                         + weights_[2] * SENSOR_POS_NORM[2]
                         + weights_[3] * SENSOR_POS_NORM[3];
      float pos = weighted_sum / maxf(sum_w, SENSOR_DENOM_FLOOR);

      bool only_left_outer  = (weights_[0] > 0.05f) && (weights_[1] < 0.08f)
                           && (weights_[2] < 0.06f) && (weights_[3] < 0.06f);
      bool only_right_outer = (weights_[3] > 0.05f) && (weights_[2] < 0.08f)
                           && (weights_[1] < 0.06f) && (weights_[0] < 0.06f);

      if (only_left_outer && last_valid_pos_ < -0.65f) {
        float fade = clampf(1.0f - weights_[0], 0.0f, 1.0f);
        pos = -1.0f - OUTER_EDGE_EXTRAP * fade;
      } else if (only_right_outer && last_valid_pos_ > 0.65f) {
        float fade = clampf(1.0f - weights_[3], 0.0f, 1.0f);
        pos = 1.0f + OUTER_EDGE_EXTRAP * fade;
      }

      line_width_ema_ = 0.96f * line_width_ema_ + 0.04f * sum_w;
      last_valid_pos_ = pos;
      if (pos > 0.25f)       last_side_ =  1.0f;
      else if (pos < -0.25f) last_side_ = -1.0f;

      return LineState::makeTracking(pos, sum_w);
    }

    // 6. All-White: Thin-Line Center Straddle vs True Hairpin Exit
    if (fabsf(last_valid_pos_) < STRADDLE_POS_LIMIT && (straddle_ticks_ * tick_ms < MAX_STRADDLE_MS)) {
      straddle_ticks_ = sat_add_u32(straddle_ticks_, 1);
      float pos_hold  = last_valid_pos_ * 0.30f;
      return LineState::makeCenterStraddle(pos_hold, straddle_ticks_ * tick_ms);
    }

    // 7. True Line Loss (Sharp Curve Exit)
    lost_ticks_ = sat_add_u32(lost_ticks_, 1);
    float dir;
    if (fabsf(last_valid_pos_) >= STRADDLE_POS_LIMIT) {
      dir = (last_valid_pos_ >= 0.0f) ? 1.0f : -1.0f;
    } else if (fabsf(eso_vel_hint) > 0.25f) {
      dir = (eso_vel_hint >= 0.0f) ? 1.0f : -1.0f;
    } else {
      dir = last_side_;
    }

    return LineState::makeLost(dir, lost_ticks_ * tick_ms);
  }

  const float*    weights()   const { return weights_; }
  const uint16_t* lastRaw()   const { return last_raw_; }
  const float*    whiteRef()  const { return white_ref_; }
  const float*    blackRef()  const { return black_ref_; }
  float           lineWidth() const { return line_width_ema_; }

private:
  float    white_ref_[4];
  float    black_ref_[4];
  bool     arena_seeded_;
  uint32_t early_run_ticks_;
  float    early_min_black_;
  bool     early_black_locked_;
  float    weights_[4];
  uint16_t last_raw_[4];
  float    last_valid_pos_;
  float    last_side_;
  float    line_width_ema_;
  uint32_t all_dark_ticks_;
  uint32_t cross_confirm_ticks_;
  uint32_t cross_cooldown_ticks_;
  uint32_t straddle_ticks_;
  uint32_t lost_ticks_;
};

// ============================================================================
//  3. DISCRETE 2ND-ORDER LADRC CONTROLLER (1-to-1 FROM `src/ladrc.rs`)
// ============================================================================

struct LadrcOutput {
  float steer_pwm;
  float z1;
  float z2;
  float z3;
  float curvature_effort;
};

class LadrcController {
public:
  explicit LadrcController(const TuningProfile& profile)
    : z1_(0.0f),
      z2_(0.0f),
      z3_(0.0f),
      u_prev_eff_(0.0f),
      omega_o_smooth_(profile.omega_o_calm),
      curv_effort_lpf_(0.0f),
      kp_(profile.omega_c * profile.omega_c),
      kd_(2.0f * profile.zeta * profile.omega_c),
      profile_(profile) {}

  void reset() {
    z1_ = 0.0f;
    z2_ = 0.0f;
    z3_ = 0.0f;
    u_prev_eff_ = 0.0f;
    omega_o_smooth_ = profile_.omega_o_calm;
    curv_effort_lpf_ = 0.0f;
  }

  void resetToPos(float pos) {
    z1_ = pos;
    z2_ = 0.0f;
    z3_ = 0.0f;
    u_prev_eff_ = 0.0f;
    omega_o_smooth_ = profile_.omega_o_calm;
    curv_effort_lpf_ = 0.0f;
  }

  inline void feedbackAppliedSteer(float applied_steer_pwm) {
    u_prev_eff_ = applied_steer_pwm;
  }

  LadrcOutput step(float e_meas, float forward_speed_est, float disturbance_leak) {
    const float ts = DT_SEC;
    const float b0 = effectiveB0(forward_speed_est);

    // 1. Adaptive Observer Bandwidth omega_o (calm on straight -> agile in curves)
    float activity = clampf(fabsf(e_meas) * 1.1f + fabsf(z2_) * 0.12f, 0.0f, 1.0f);
    float omega_o_target = profile_.omega_o_calm
                         + activity * (profile_.omega_o_agile - profile_.omega_o_calm);
    omega_o_smooth_ += 0.08f * (omega_o_target - omega_o_smooth_);

    // 2. Exact Pole-Placement Discrete Current-Observer Gains (Miklosovic & Gao, 2006)
    float beta       = expf(-omega_o_smooth_ * ts);
    float one_m_beta = 1.0f - beta;
    float l1 = 1.0f - beta * beta * beta;
    float l2 = (1.5f / ts) * one_m_beta * one_m_beta * (1.0f + beta);
    float l3 = (1.0f / (ts * ts)) * one_m_beta * one_m_beta * one_m_beta;

    // 3. Exact ZOH State Prediction using previous ACTUAL applied steering u_prev_eff
    float accel_pred = z3_ - b0 * u_prev_eff_;
    float z1_pred = z1_ + ts * z2_ + 0.5f * ts * ts * accel_pred;
    float z2_pred = z2_ + ts * accel_pred;
    float z3_pred = z3_ * disturbance_leak;

    // 4. Current-Observer Measurement Update (zero phase lag!)
    float innov = e_meas - z1_pred;
    z1_ = z1_pred + l1 * innov;
    z2_ = z2_pred + l2 * innov;
    z3_ = z3_pred + l3 * innov;

    // Fast sign-reversal release on z3 when exiting curves onto straightaways
    if ((z1_ * z3_ < 0.0f) && (fabsf(z1_) > 0.12f)) {
      z3_ *= 0.90f;
    }

    // Clamp disturbance <= 35% of max steering so PD damping ALWAYS dominates
    float max_z3 = b0 * profile_.max_steer_pwm * 0.35f;
    z3_ = clampf(z3_, -max_z3, max_z3);

    // 5. Linear State Error Feedback (LSEF) + Total Disturbance Cancellation
    float z1_shaped = shapeError(z1_);
    float u_raw     = (kp_ * z1_shaped + kd_ * z2_ + z3_) / b0;
    float steer_pwm = clampf(u_raw, -profile_.max_steer_pwm, profile_.max_steer_pwm);

    // 6. Filtered Curvature/Disturbance Effort for velocity planner
    float raw_curv   = clampf(fabsf(z3_ / (b0 * profile_.max_steer_pwm)), 0.0f, 1.5f);
    float curv_alpha = (raw_curv > curv_effort_lpf_) ? 0.12f : 0.035f;
    curv_effort_lpf_ += curv_alpha * (raw_curv - curv_effort_lpf_);

    return { steer_pwm, z1_, z2_, z3_, curv_effort_lpf_ };
  }

  void coastDuringLost(float lost_dir, float forward_speed_est) {
    float b0 = effectiveB0(forward_speed_est);
    z1_ = 0.90f * z1_ + 0.10f * (lost_dir * 1.15f);
    z2_ *= 0.92f;
    float target_z3 = lost_dir * b0 * profile_.max_steer_pwm * 0.35f;
    z3_ = 0.95f * z3_ + 0.05f * target_z3;
    curv_effort_lpf_ = 0.95f * curv_effort_lpf_ + 0.05f * 1.0f;
  }

  inline float velocityHint() const { return z2_; }

private:
  inline float effectiveB0(float forward_speed_est) const {
    float v_norm = clampf(forward_speed_est / PWM_MAX, 0.0f, 1.0f);
    return profile_.b0_base + profile_.b0_speed_gain * v_norm;
  }

  static inline float shapeError(float e) {
    float abs_e = fabsf(e);
    if (abs_e < 0.10f) {
      float t    = abs_e / 0.10f;
      float gain = 0.70f + 0.30f * t * t;
      return e * gain;
    }
    return e;
  }

  float         z1_;
  float         z2_;
  float         z3_;
  float         u_prev_eff_;
  float         omega_o_smooth_;
  float         curv_effort_lpf_;
  float         kp_;
  float         kd_;
  TuningProfile profile_;
};

// ============================================================================
//  4. VELOCITY PLANNER & 3-PHASE SHORTCUT MACHINE (1-to-1 FROM `src/planner.rs`)
// ============================================================================

struct DriveCommand {
  float base_speed;
  float steer_cmd;
  float max_reverse_pwm;
  bool  instant_snap;
  float anti_plug_th;
  bool  safety_stopped;
};

enum class CrossingModeKind : uint8_t {
  None,
  StraightLock,
  ShortcutLeave,
  ShortcutAcquire,
  ShortcutBrake,
};

class MotionPlanner {
public:
  explicit MotionPlanner(const TuningProfile& profile)
    : profile_(profile),
      base_speed_lpf_(0.0f),
      recovery_ramp_(0.0f),
      cross_count_(0),
      cross_mode_(CrossingModeKind::None),
      mode_dir_sign_(1),
      mode_ticks_(0),
      mode_white_ticks_(0) {}

  void reset() {
    base_speed_lpf_   = 0.0f;
    recovery_ramp_    = 0.0f;
    cross_count_      = 0;
    cross_mode_       = CrossingModeKind::None;
    mode_dir_sign_    = 1;
    mode_ticks_       = 0;
    mode_white_ticks_ = 0;
  }

  size_t crossCount() const { return cross_count_; }

  bool isInShortcut() const {
    return (cross_mode_ == CrossingModeKind::ShortcutLeave) ||
           (cross_mode_ == CrossingModeKind::ShortcutAcquire) ||
           (cross_mode_ == CrossingModeKind::ShortcutBrake);
  }

  DriveCommand compute(
    const LineState& line_state,
    const float sensor_weights[4],
    LadrcController& ladrc,
    float est_forward_speed
  ) {
    const uint32_t tick_ms = 1000 / LOOP_FREQ_HZ;

    // 0. Lifted-in-air ALWAYS overrides everything: stop & reset crossing counter
    if (line_state.kind == LineKind::Lifted) {
      reset();
      ladrc.reset();
      return { 0.0f, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, true };
    }

    // 1. Trigger a new crossing maneuver ONLY when idle
    if ((cross_mode_ == CrossingModeKind::None) &&
        (line_state.kind == LineKind::Crossing) &&
        line_state.is_new_event) {
      size_t idx = cross_count_;
      if (cross_count_ < SIZE_MAX) cross_count_ += 1;
      CrossingAction action = (idx < CROSSING_PLAN_LEN)
                                ? CROSSING_PLAN[idx]
                                : CrossingAction::Straight;

      ladrc.reset();
      recovery_ramp_ = 0.0f;
      switch (action) {
        case CrossingAction::Straight:
          cross_mode_ = CrossingModeKind::StraightLock;
          mode_ticks_ = 0;
          break;
        case CrossingAction::ShortcutRight:
          cross_mode_       = CrossingModeKind::ShortcutLeave;
          mode_dir_sign_    = 1;
          mode_ticks_       = 0;
          mode_white_ticks_ = 0;
          break;
        case CrossingAction::ShortcutLeft:
          cross_mode_       = CrossingModeKind::ShortcutLeave;
          mode_dir_sign_    = -1;
          mode_ticks_       = 0;
          mode_white_ticks_ = 0;
          break;
      }
    }

    // 2. Execute Active Crossing Maneuver
    switch (cross_mode_) {
      case CrossingModeKind::StraightLock: {
        uint32_t next_ticks     = mode_ticks_ + 1;
        uint32_t elapsed        = next_ticks * tick_ms;
        bool     wings_clear    = (sensor_weights[0] < 0.25f) && (sensor_weights[3] < 0.25f);
        bool     still_crossing = (line_state.kind == LineKind::Crossing);

        if ((elapsed >= 44 && !still_crossing && wings_clear) || (elapsed >= MAX_CROSS_LOCK_MS)) {
          cross_mode_ = CrossingModeKind::None;
        } else {
          mode_ticks_ = next_ticks;
          float target_base = minf(
            profile_.v_crossing,
            maxf(base_speed_lpf_, profile_.v_curve_min)
          );
          base_speed_lpf_ += 0.10f * (target_base - base_speed_lpf_);
          return { base_speed_lpf_, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, false };
        }
        break;
      }

      case CrossingModeKind::ShortcutLeave: {
        uint32_t next_ticks = mode_ticks_ + 1;
        uint32_t elapsed    = next_ticks * tick_ms;

        bool old_line_cleared = (mode_dir_sign_ > 0)
          ? (sensor_weights[0] < 0.20f && sensor_weights[1] < 0.20f && sensor_weights[2] < 0.20f)
          : (sensor_weights[1] < 0.20f && sensor_weights[2] < 0.20f && sensor_weights[3] < 0.20f);

        uint32_t next_white = (elapsed >= SHORTCUT_LEAVE_MIN_MS && old_line_cleared)
                                ? (mode_white_ticks_ + 1)
                                : 0;

        if (next_white >= 3 || elapsed >= SHORTCUT_LEAVE_MAX_MS) {
          cross_mode_ = CrossingModeKind::ShortcutAcquire;
          mode_ticks_ = 0;
        } else {
          mode_ticks_       = next_ticks;
          mode_white_ticks_ = next_white;
        }

        float outer     = (elapsed < SHORTCUT_BRAKE_MS) ? SHORTCUT_BRAKE_OUTER_PWM : SHORTCUT_OUTER_PWM;
        float inner_rev = (elapsed < SHORTCUT_BRAKE_MS) ? 0.0f                     : SHORTCUT_INNER_REV_PWM;
        float plug_th   = (elapsed < SHORTCUT_BRAKE_MS) ? ANTI_PLUG_OMEGA_TH       : SHORTCUT_ANTI_PLUG_TH;

        float base  = 0.5f * (outer - inner_rev);
        float steer = static_cast<float>(mode_dir_sign_) * 0.5f * (outer + inner_rev);
        base_speed_lpf_ = base;
        return { base, steer, inner_rev, true, plug_th, false };
      }

      case CrossingModeKind::ShortcutAcquire: {
        uint32_t next_ticks = mode_ticks_ + 1;
        uint32_t elapsed    = next_ticks * tick_ms;

        float outer_w = (mode_dir_sign_ > 0) ? sensor_weights[3] : sensor_weights[0];
        bool  branch_hit = (outer_w >= 0.42f) ||
                           (sensor_weights[2] >= 0.35f) ||
                           (sensor_weights[1] >= 0.35f);

        if (branch_hit || elapsed >= SHORTCUT_ACQUIRE_MAX_MS) {
          cross_mode_ = CrossingModeKind::ShortcutBrake;
          mode_ticks_ = 0;
          float base  = 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
          float steer = -static_cast<float>(mode_dir_sign_) * 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
          base_speed_lpf_ = base;
          return { base, steer, 0.0f, true, SHORTCUT_ANTI_PLUG_TH, false };
        } else {
          mode_ticks_ = next_ticks;
          float base  = 0.5f * (SHORTCUT_OUTER_PWM - SHORTCUT_INNER_REV_PWM);
          float steer = static_cast<float>(mode_dir_sign_) * 0.5f * (SHORTCUT_OUTER_PWM + SHORTCUT_INNER_REV_PWM);
          base_speed_lpf_ = base;
          return { base, steer, SHORTCUT_INNER_REV_PWM, true, SHORTCUT_ANTI_PLUG_TH, false };
        }
      }

      case CrossingModeKind::ShortcutBrake: {
        uint32_t next_ticks = mode_ticks_ + 1;
        uint32_t elapsed    = next_ticks * tick_ms;

        bool centered = (mode_dir_sign_ > 0)
          ? ((sensor_weights[1] >= 0.30f && sensor_weights[2] >= 0.30f) || sensor_weights[1] >= 0.45f)
          : ((sensor_weights[1] >= 0.30f && sensor_weights[2] >= 0.30f) || sensor_weights[2] >= 0.45f);

        if ((elapsed >= SHORTCUT_COUNTER_BRAKE_MIN_MS && centered) ||
            (elapsed >= SHORTCUT_COUNTER_BRAKE_MAX_MS)) {
          cross_mode_ = CrossingModeKind::None;
          float sum_w = sensor_weights[0] + sensor_weights[1] + sensor_weights[2] + sensor_weights[3];
          float pos   = 0.0f;
          if (sum_w >= LINE_PRESENT_SUM) {
            float wsum = sensor_weights[0] * SENSOR_POS_NORM[0]
                       + sensor_weights[1] * SENSOR_POS_NORM[1]
                       + sensor_weights[2] * SENSOR_POS_NORM[2]
                       + sensor_weights[3] * SENSOR_POS_NORM[3];
            pos = wsum / maxf(sum_w, SENSOR_DENOM_FLOOR);
          }
          ladrc.resetToPos(pos);
          base_speed_lpf_ = 185.0f;
        } else {
          mode_ticks_ = next_ticks;
          float base  = 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
          float steer = -static_cast<float>(mode_dir_sign_) * 0.5f * SHORTCUT_COUNTER_BRAKE_PWM;
          base_speed_lpf_ = base;
          return { base, steer, 0.0f, true, SHORTCUT_ANTI_PLUG_TH, false };
        }
        break;
      }

      case CrossingModeKind::None:
        break;
    }

    // 3. Normal High-Speed 255 LADRC Line Tracking & Recovery
    switch (line_state.kind) {
      case LineKind::Tracking: {
        recovery_ramp_ = 0.0f;
        LadrcOutput out = ladrc.step(line_state.pos, est_forward_speed, 1.0f);

        // Wide 0.35 deadband so straights & gentle/medium sweeps stay LOCKED at 255/255!
        float err_term    = fabsf(out.z1) * profile_.slow_k_err;
        float vel_term    = fabsf(out.z2) * profile_.slow_k_vel;
        float curv_term   = out.curvature_effort * profile_.slow_k_curv;
        float slow_raw    = err_term + vel_term + curv_term;
        float slow_factor = clampf((slow_raw - 0.35f) / 0.65f, 0.0f, 1.0f);

        float target_base = profile_.v_max - (profile_.v_max - profile_.v_curve_min) * slow_factor;
        float alpha       = (target_base < base_speed_lpf_) ? 0.20f : 0.075f;
        base_speed_lpf_  += alpha * (target_base - base_speed_lpf_);

        return {
          base_speed_lpf_,
          out.steer_pwm,
          profile_.max_reverse_tracking,
          false,
          ANTI_PLUG_OMEGA_TH,
          false
        };
      }

      case LineKind::Crossing: {
        recovery_ramp_ = 0.0f;
        LadrcOutput out = ladrc.step(0.0f, est_forward_speed, 0.95f);
        float target_base = minf(
          profile_.v_crossing,
          maxf(base_speed_lpf_, profile_.v_curve_min)
        );
        base_speed_lpf_ += 0.08f * (target_base - base_speed_lpf_);

        return {
          base_speed_lpf_,
          clampf(out.steer_pwm * 0.15f, -25.0f, 25.0f),
          0.0f,
          false,
          ANTI_PLUG_OMEGA_TH,
          false
        };
      }

      case LineKind::CenterStraddle: {
        recovery_ramp_ = 0.0f;
        LadrcOutput out = ladrc.step(line_state.pos, est_forward_speed, 0.98f);
        float target_base = profile_.v_straddle;
        base_speed_lpf_  += 0.06f * (target_base - base_speed_lpf_);

        return {
          base_speed_lpf_,
          clampf(out.steer_pwm, -70.0f, 70.0f),
          0.0f,
          false,
          ANTI_PLUG_OMEGA_TH,
          false
        };
      }

      case LineKind::Lost: {
        if (line_state.elapsed_ms >= LOST_SAFETY_STOP_MS) {
          base_speed_lpf_ = 0.0f;
          recovery_ramp_  = 0.0f;
          return { 0.0f, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, true };
        }

        float dir = line_state.pos;
        ladrc.coastDuringLost(dir, est_forward_speed);

        recovery_ramp_ = minf(recovery_ramp_ + 0.035f, 1.0f);
        float outer = profile_.v_curve_min
                    + recovery_ramp_ * (profile_.recovery_outer_pwm - profile_.v_curve_min);
        float inner = 0.0f;
        if (line_state.elapsed_ms >= 70) {
          float rev_t = minf(static_cast<float>(line_state.elapsed_ms - 70) / 90.0f, 1.0f);
          inner = rev_t * profile_.recovery_inner_pwm;
        }

        float base  = 0.5f * (outer + inner);
        float steer = dir * 0.5f * (outer - inner);
        base_speed_lpf_ = base;

        return {
          base,
          steer,
          fabsf(profile_.recovery_inner_pwm),
          false,
          ANTI_PLUG_OMEGA_TH,
          false
        };
      }

      case LineKind::Lifted:
      default:
        return { 0.0f, 0.0f, 0.0f, false, ANTI_PLUG_OMEGA_TH, true };
    }
  }

private:
  TuningProfile    profile_;
  float            base_speed_lpf_;
  float            recovery_ramp_;
  size_t           cross_count_;
  CrossingModeKind cross_mode_;
  int8_t           mode_dir_sign_;
  uint32_t         mode_ticks_;
  uint32_t         mode_white_ticks_;
};

// ============================================================================
//  5. ZERO-LAG STEERING + 4x AA BATTERY-SAFE MOTOR CONTROLLER
//     (1-to-1 FROM `src/power_motor.rs`, Arduino Core 2.x & 3.x Compatible!)
// ============================================================================

struct AppliedDrive {
  float left_pwm;
  float right_pwm;
  float applied_steer;
  float est_forward_speed;
};

class MotorPowerController {
public:
  MotorPowerController()
    : applied_base_(0.0f),
      applied_steer_(0.0f),
      omega_hat_l_(0.0f),
      omega_hat_r_(0.0f),
      run_ticks_(0) {}

  void begin() {
    ledcInitChannel(PIN_MOTOR_L_A, LEDC_CH_L_A);
    ledcInitChannel(PIN_MOTOR_L_B, LEDC_CH_L_B);
    ledcInitChannel(PIN_MOTOR_R_A, LEDC_CH_R_A);
    ledcInitChannel(PIN_MOTOR_R_B, LEDC_CH_R_B);
    coastStop();
  }

  void reset() {
    run_ticks_     = 0;
    applied_base_  = 0.0f;
    applied_steer_ = 0.0f;
    omega_hat_l_   = 0.0f;
    omega_hat_r_   = 0.0f;
    coastStop();
  }

  void coastStop() {
    ledcSetDuty(LEDC_CH_L_A, 0);
    ledcSetDuty(LEDC_CH_L_B, 0);
    ledcSetDuty(LEDC_CH_R_A, 0);
    ledcSetDuty(LEDC_CH_R_B, 0);
    applied_base_  = 0.0f;
    applied_steer_ = 0.0f;
    float alpha_m  = DT_SEC / (MOTOR_MECH_TAU_S + DT_SEC);
    omega_hat_l_  += alpha_m * (0.0f - omega_hat_l_);
    omega_hat_r_  += alpha_m * (0.0f - omega_hat_r_);
  }

  void driveCalibration(float left_target, float right_target) {
    run_ticks_ = (SOFT_START_MS * LOOP_FREQ_HZ) / 1000 + 1;
    DriveCommand cmd = {
      0.5f * (left_target + right_target),
      0.5f * (left_target - right_target),
      90.0f,
      false,
      ANTI_PLUG_OMEGA_TH,
      false
    };
    (void)apply(cmd);
  }

  AppliedDrive apply(const DriveCommand& cmd) {
    // 1. Soft-start S-curve on launch (protects 4x AA on standing start)
    uint32_t soft_ticks = (SOFT_START_MS * LOOP_FREQ_HZ) / 1000;
    if (soft_ticks < 1) soft_ticks = 1;
    run_ticks_ = sat_add_u32(run_ticks_, 1);

    float soft_scale = 1.0f;
    if (run_ticks_ < soft_ticks) {
      float t = static_cast<float>(run_ticks_) / static_cast<float>(soft_ticks);
      soft_scale = t * t * (3.0f - 2.0f * t);
    }

    float target_base  = clampf(cmd.base_speed * soft_scale, -PWM_MAX, PWM_MAX);
    float target_steer = clampf(cmd.steer_cmd  * soft_scale, -PWM_MAX, PWM_MAX);

    // 2. Decoupled Slew Rate Limiting (bypassed on `instant_snap` during shortcut)
    float base, steer;
    if (cmd.instant_snap) {
      base  = target_base;
      steer = target_steer;
    } else {
      float max_base_up = COMMON_ACCEL_PWM_PER_SEC * DT_SEC;
      float max_base_dn = COMMON_BRAKE_PWM_PER_SEC * DT_SEC;
      float base_diff   = target_base - applied_base_;
      float base_step   = (base_diff > 0.0f) ? minf(base_diff, max_base_up)
                                             : maxf(base_diff, -max_base_dn);
      base = applied_base_ + base_step;

      float max_steer_step = STEER_SLEW_PWM_PER_SEC * DT_SEC;
      steer = applied_steer_ + clampf(target_steer - applied_steer_, -max_steer_step, max_steer_step);
    }

    applied_base_  = base;
    applied_steer_ = steer;

    // 3. Steering-Priority Differential Mixer with 4x AA Wheel Cap
    const float max_w = ACTIVE_PROFILE.max_wheel_pwm;
    float l = base + steer;
    float r = base - steer;
    float peak = maxf(l, r);
    if (peak > max_w) {
      float overflow = peak - max_w;
      l -= overflow;
      r -= overflow;
    }
    float min_pwm = -cmd.max_reverse_pwm;
    l = clampf(l, min_pwm, max_w);
    r = clampf(r, min_pwm, max_w);

    float sum_abs = fabsf(l) + fabsf(r);
    if (sum_abs > MAX_TOTAL_PWM) {
      float scale = MAX_TOTAL_PWM / sum_abs;
      l *= scale;
      r *= scale;
    }

    // 4. Anti-Plugging Coast Guard
    l = antiPlugGuard(l, omega_hat_l_, cmd.anti_plug_th);
    r = antiPlugGuard(r, omega_hat_r_, cmd.anti_plug_th);

    // 5. Joint Positive Battery-Draw Current Envelope Limiter (draw_L + draw_R <= 135.0)
    float draw_l     = batteryDraw(l, omega_hat_l_);
    float draw_r     = batteryDraw(r, omega_hat_r_);
    float total_draw = draw_l + draw_r;
    if (total_draw > BATTERY_CURRENT_BUDGET) {
      float scale = BATTERY_CURRENT_BUDGET / total_draw;
      if (draw_l > 0.0f) l = scaleMotorDraw(l, omega_hat_l_, draw_l * scale);
      if (draw_r > 0.0f) r = scaleMotorDraw(r, omega_hat_r_, draw_r * scale);
      l = clampf(l, min_pwm, max_w);
      r = clampf(r, min_pwm, max_w);
    }

    // 6. Update Back-EMF Wheel Velocity Estimator
    float alpha_m = DT_SEC / (MOTOR_MECH_TAU_S + DT_SEC);
    omega_hat_l_ += alpha_m * (l - omega_hat_l_);
    omega_hat_r_ += alpha_m * (r - omega_hat_r_);

    // 7. Output Hardware PWM (5 kHz, 8-bit 0..255 -> Arduino HAL maps 255 to 100% DC!)
    writeHardware(l, r);

    return {
      l,
      r,
      0.5f * (l - r),
      maxf(0.5f * (omega_hat_l_ + omega_hat_r_), 0.0f)
    };
  }

private:
  static inline void ledcInitChannel(uint8_t pin, uint8_t ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(pin, PWM_FREQ_HZ, PWM_RES_BITS, ch);
    ledcWriteChannel(ch, 0);
#else
    ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(pin, ch);
    ledcWrite(ch, 0);
#endif
  }

  static inline void ledcSetDuty(uint8_t ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(ch, duty);
#else
    ledcWrite(ch, duty);
#endif
  }

  static inline float batteryDraw(float cmd, float omega_hat) {
    float abs_cmd = fabsf(cmd);
    if (abs_cmd < 1.0f) return 0.0f;
    if (cmd * omega_hat >= 0.0f) {
      return maxf(abs_cmd - fabsf(omega_hat), 0.0f);
    }
    return abs_cmd + fabsf(omega_hat);
  }

  static inline float scaleMotorDraw(float cmd, float omega_hat, float allowed_draw) {
    if (cmd < 0.0f && omega_hat > 0.0f) {
      return -maxf(allowed_draw - omega_hat, 0.0f);
    } else if (cmd > 0.0f && omega_hat < 0.0f) {
      return maxf(allowed_draw + omega_hat, 0.0f);
    } else if (cmd >= 0.0f) {
      return maxf(omega_hat, 0.0f) + allowed_draw;
    } else {
      return minf(omega_hat, 0.0f) - allowed_draw;
    }
  }

  static inline float antiPlugGuard(float cmd, float omega_hat, float thresh) {
    if ((omega_hat > thresh && cmd < -10.0f) || (omega_hat < -thresh && cmd > 10.0f)) {
      return 0.0f;
    }
    return cmd;
  }

  static inline float compensateDeadzone(float pwm) {
    float abs_v = fabsf(pwm);
    if (abs_v < 2.0f) return 0.0f;
    float sign   = (pwm >= 0.0f) ? 1.0f : -1.0f;
    float mapped = MOTOR_DEADZONE_PWM + (abs_v / PWM_MAX) * (PWM_MAX - MOTOR_DEADZONE_PWM);
    return sign * clampf(mapped, 0.0f, PWM_MAX);
  }

  void writeHardware(float logical_l, float logical_r) const {
    float l = compensateDeadzone(logical_l);
    float r = compensateDeadzone(logical_r * RIGHT_MOTOR_TRIM);

    if (SWAP_MOTORS_LR) {
      float tmp = l;
      l = r;
      r = tmp;
    }
    if (INVERT_LEFT_MOTOR)  l = -l;
    if (INVERT_RIGHT_MOTOR) r = -r;

    setMotorPair(LEDC_CH_L_A, LEDC_CH_L_B, l);
    setMotorPair(LEDC_CH_R_A, LEDC_CH_R_B, r);
  }

  static inline void setMotorPair(uint8_t ch_a, uint8_t ch_b, float signed_pwm) {
    float abs_v = fabsf(signed_pwm);
    // In ESP32 Arduino HAL, duty == 255 on 8-bit LEDC automatically drives true 100% DC (256/256)!
    uint32_t duty = (abs_v >= 254.5f) ? 255U : static_cast<uint32_t>(abs_v + 0.5f);
    if (signed_pwm > 0.5f) {
      ledcSetDuty(ch_b, 0);
      ledcSetDuty(ch_a, duty);
    } else if (signed_pwm < -0.5f) {
      ledcSetDuty(ch_a, 0);
      ledcSetDuty(ch_b, duty);
    } else {
      ledcSetDuty(ch_a, 0);
      ledcSetDuty(ch_b, 0);
    }
  }

  float    applied_base_;
  float    applied_steer_;
  float    omega_hat_l_;
  float    omega_hat_r_;
  uint32_t run_ticks_;
};

// ============================================================================
//  6. GLOBAL SUBSYSTEMS & ADC MEDIAN-OF-3 READER
// ============================================================================

static MotorPowerController g_motors;
static SensorProcessor      g_sensors;
static LadrcController      g_ladrc(ACTIVE_PROFILE);
static MotionPlanner        g_planner(ACTIVE_PROFILE);

static uint32_t g_telemetry_ticks   = 50;
static uint32_t g_tick_counter      = 0;
static float    g_est_forward_speed = 0.0f;

/// Read all 4 IR ADC1 channels with Median-of-3 spike rejection.
/// Order: [GPIO0 (S1), GPIO1 (S2), GPIO3 (S3), GPIO4 (S4)].
static inline void readIrMedian3(uint16_t out[4]) {
  uint16_t s[4][3];
  for (int k = 0; k < 3; ++k) {
    s[0][k] = static_cast<uint16_t>(analogRead(PIN_IR_S1));
    s[1][k] = static_cast<uint16_t>(analogRead(PIN_IR_S2));
    s[2][k] = static_cast<uint16_t>(analogRead(PIN_IR_S3));
    s[3][k] = static_cast<uint16_t>(analogRead(PIN_IR_S4));
  }
  for (int i = 0; i < 4; ++i) {
    out[i] = SensorProcessor::median3(s[i][0], s[i][1], s[i][2]);
  }
}

// ============================================================================
//  7. ARDUINO `setup()` & DETERMINISTIC 500 Hz `loop()`
// ============================================================================

void setup() {
  // 1. Run ESP32-C3 at 80 MHz (NO Wi-Fi/BLE => ~40% lower 3.3V draw on AMS1117-3.3)
  setCpuFrequencyMhz(80);

  if (ENABLE_TELEMETRY) {
    Serial.begin(115200);
  }

  // 2. Turn ON 4x IR emitter LEDs (GPIO 21 = HIGH)
  pinMode(PIN_IR_LED_ON, OUTPUT);
  digitalWrite(PIN_IR_LED_ON, HIGH);

  // 3. Optional BOOT button on GPIO 9 (disabled by default since covered on chassis)
  if (ENABLE_BOOT_SWEEP_CALIB) {
    pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  }

  // 4. Configure ADC1: 12-bit (0..4095), 11 dB attenuation (~0..3.1V full IR range)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // 5. Configure LEDC PWM (5 kHz, 8-bit) & coast-stop motors
  g_motors.begin();

  if (ENABLE_TELEMETRY) {
    Serial.println("=== LADRC Line Follower (ESP32-C3 / 500Hz / 4xAA Safe / C++) ===");
    Serial.printf(
      "Profile: v_max=%d, v_curve=%d, wc=%d, wo=%d..%d\n",
      static_cast<int>(ACTIVE_PROFILE.v_max),
      static_cast<int>(ACTIVE_PROFILE.v_curve_min),
      static_cast<int>(ACTIVE_PROFILE.omega_c),
      static_cast<int>(ACTIVE_PROFILE.omega_o_calm),
      static_cast<int>(ACTIVE_PROFILE.omega_o_agile)
    );
  }

  // 6. Startup Settle Window (280 ms): stabilize AMS1117-3.3 & IR LEDs, seed arena baseline
  uint32_t settle_ticks   = (STARTUP_SETTLE_MS * LOOP_FREQ_HZ) / 1000;
  uint32_t boot_low_count = 0;
  for (uint32_t t = 0; t < settle_ticks; ++t) {
    uint32_t tick_start = micros();
    uint16_t raw[4];
    readIrMedian3(raw);
    g_sensors.seedStartupSample(raw);
    if (ENABLE_BOOT_SWEEP_CALIB && digitalRead(PIN_BOOT_BTN) == LOW) {
      boot_low_count += 1;
    }
    while (static_cast<uint32_t>(micros() - tick_start) < LOOP_PERIOD_US) {}
  }

  // 7. Optional Gentle In-Place Sweep Calibration (ONLY if ENABLE_BOOT_SWEEP_CALIB == true)
  if (ENABLE_BOOT_SWEEP_CALIB && (boot_low_count > settle_ticks / 3)) {
    if (ENABLE_TELEMETRY) {
      Serial.println("[CALIB] BOOT detected -> Starting 2.4s gentle sweep calibration...");
    }
    g_sensors.sweepCalibrateReset();

    uint32_t wait_release = millis();
    while ((digitalRead(PIN_BOOT_BTN) == LOW) &&
           (static_cast<uint32_t>(millis() - wait_release) < 800UL)) {}

    uint32_t sweep_ticks = (2400UL * LOOP_FREQ_HZ) / 1000UL;
    for (uint32_t t = 0; t < sweep_ticks; ++t) {
      uint32_t tick_start = micros();
      uint16_t raw[4];
      readIrMedian3(raw);
      g_sensors.sweepCalibrateSample(raw);

      uint32_t phase = (t * 4UL) / sweep_ticks;
      float l_cmd = (phase == 1 || phase == 2) ?  72.0f : -72.0f;
      float r_cmd = (phase == 1 || phase == 2) ? -72.0f :  72.0f;
      g_motors.driveCalibration(l_cmd, r_cmd);
      while (static_cast<uint32_t>(micros() - tick_start) < LOOP_PERIOD_US) {}
    }

    g_motors.coastStop();
    g_sensors.sweepCalibrateFinish();

    if (ENABLE_TELEMETRY) {
      const float* w = g_sensors.whiteRef();
      const float* b = g_sensors.blackRef();
      Serial.printf(
        "[CALIB] Done! White=[%d,%d,%d,%d] Black=[%d,%d,%d,%d]\n",
        static_cast<int>(w[0]), static_cast<int>(w[1]), static_cast<int>(w[2]), static_cast<int>(w[3]),
        static_cast<int>(b[0]), static_cast<int>(b[1]), static_cast<int>(b[2]), static_cast<int>(b[3])
      );
    }

    uint32_t pause_start = millis();
    while (static_cast<uint32_t>(millis() - pause_start) < 1200UL) {
      if (digitalRead(PIN_BOOT_BTN) == LOW) break;
    }
    g_motors.reset();
  }

  if (ENABLE_TELEMETRY) {
    Serial.println("[RUN] Soft-start launch!");
  }
  g_ladrc.reset();
  g_planner.reset();

  g_telemetry_ticks = (TELEMETRY_INTERVAL_MS * LOOP_FREQ_HZ) / 1000;
  if (g_telemetry_ticks < 1) g_telemetry_ticks = 1;
  g_tick_counter      = 0;
  g_est_forward_speed = 0.0f;
}

void loop() {
  uint32_t tick_start = micros();

  // A. Read 4 IR sensors (Median-of-3 spike rejection)
  uint16_t raw[4];
  readIrMedian3(raw);

  // B. Online auto-calibration + trapezoid local-contrast fusion & obstacle classifier
  LineState line_state = g_sensors.update(raw, g_ladrc.velocityHint());

  // C. LADRC + Curvature-Adaptive Velocity Planner & 3-Phase Shortcut State Machine
  DriveCommand drive_cmd = g_planner.compute(
    line_state,
    g_sensors.weights(),
    g_ladrc,
    g_est_forward_speed
  );

  // D. Apply Zero-Lag Steering + 4x AA Battery-Safe Motor Control
  bool         has_applied = false;
  AppliedDrive applied     = { 0.0f, 0.0f, 0.0f, 0.0f };

  if (drive_cmd.safety_stopped) {
    if (line_state.kind == LineKind::Lifted) {
      // Full reset on Lifted => smooth S-curve soft-start when placed down at START!
      g_motors.reset();
    } else {
      g_motors.coastStop();
    }
    g_ladrc.feedbackAppliedSteer(0.0f);
    g_est_forward_speed = 0.0f;
  } else {
    applied = g_motors.apply(drive_cmd);
    // Feed ACTUAL post-limiter steering back into LESO => exact bumpless anti-windup!
    g_ladrc.feedbackAppliedSteer(applied.applied_steer);
    g_est_forward_speed = applied.est_forward_speed;
    has_applied = true;
  }

  // E. Optional Throttled Serial Telemetry (10 Hz)
  g_tick_counter += 1;
  if (ENABLE_TELEMETRY && (g_tick_counter % g_telemetry_ticks == 0)) {
    const uint16_t* r = g_sensors.lastRaw();
    const float*    w = g_sensors.weights();
    const char* state_tag = "TRK";
    if (g_planner.isInShortcut()) {
      state_tag = "SHRT";
    } else {
      switch (line_state.kind) {
        case LineKind::Tracking:       state_tag = "TRK"; break;
        case LineKind::Crossing:       state_tag = "CRS"; break;
        case LineKind::CenterStraddle: state_tag = "STR"; break;
        case LineKind::Lost:           state_tag = "LST"; break;
        case LineKind::Lifted:         state_tag = "AIR"; break;
      }
    }
    int al = has_applied ? static_cast<int>(applied.left_pwm)  : 0;
    int ar = has_applied ? static_cast<int>(applied.right_pwm) : 0;
    Serial.printf(
      "[%s#%u] raw=[%u,%u,%u,%u] w=[%d,%d,%d,%d] L=%d R=%d v=%d\n",
      state_tag,
      static_cast<unsigned>(g_planner.crossCount()),
      static_cast<unsigned>(r[0]),
      static_cast<unsigned>(r[1]),
      static_cast<unsigned>(r[2]),
      static_cast<unsigned>(r[3]),
      static_cast<int>(w[0] * 100.0f),
      static_cast<int>(w[1] * 100.0f),
      static_cast<int>(w[2] * 100.0f),
      static_cast<int>(w[3] * 100.0f),
      al,
      ar,
      static_cast<int>(g_est_forward_speed)
    );
  }

  // F. Exact Fixed-Timestep Pace (2000 us = 500 Hz)
  while (static_cast<uint32_t>(micros() - tick_start) < LOOP_PERIOD_US) {}
}
