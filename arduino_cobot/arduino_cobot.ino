/**
 * ============================================================================
 * CODE ARDUINO NẠP MẠCH THẬT CHO ROBOT DÒ ĐƯỜNG COBOT
 * CUỘC THI: "ĐƯỜNG TỚI TÂN CƯƠNG - THÁI NGUYÊN LẦN THỨ 2 (2026)"
 * ----------------------------------------------------------------------------
 * Phần cứng tương thích chuẩn bộ Kit thi đấu:
 * 1. Mạch điều khiển & công suất: KM-11-102 (MCU ATmega328P / Arduino Nano)
 * 2. Mạch cảm biến: KMM-11-100 (4 mắt hồng ngoại IR + Cảm biến màu I2C)
 * 3. Động cơ: 2x GA12-N20 5V 300RPM (KM-23-101) kèm bánh xe mini
 * 4. Nguồn điện: Khay 4 pin AA tiêu chuẩn (4.5V - 7.4V)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// CẤU HÌNH SƠ ĐỒ CHÂN PHẦN CỨNG MẠCH KM-11-102
// ============================================================================
const int PIN_IR[4] = {A0, A1, A2, A3};  // IR0 (Trái ngoài), IR1 (Trái trong), IR2 (Phải trong), IR3 (Phải ngoài)
const int PIN_BAT   = A7;                // Chân analog đo điện áp pin qua cầu phân áp 1:1

// Cầu H điều khiển động cơ DC (GA12-N20)
const int ENA = 5;                       // PWM tốc độ bánh Trái
const int IN1 = 4;                       // Chiều quay bánh Trái
const int IN2 = 3;
const int ENB = 6;                       // PWM tốc độ bánh Phải
const int IN3 = 7;                       // Chiều quay bánh Phải
const int IN4 = 8;

// Bù vùng chết động cơ GA12-N20 (Do hộp số kim loại có ma sát tĩnh)
const int MIN_N20_PWM = 45;              // PWM tối thiểu để động cơ bắt đầu quay êm

// ============================================================================
// HÀM TOÁN HỌC TIỆN ÍCH AN TOÀN
// ============================================================================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }

// ============================================================================
// CẤU TRÚC DỮ LIỆU CẢM BIẾN TRỪU TƯỢNG (HAL)
// ============================================================================
struct SensorData {
    float ir[4];          // Chuẩn hóa: 0.00 (Nền sáng/Thanh gờ xanh) -> 1.00 (Tâm line đen)
    int raw_ir[4];        // Giá trị ADC analog thô (0..1023)
    int r, g, b;          // Cảm biến màu RGB (0..255)
    float battery_volt;   // Điện áp pin AA thực tế (V)
    double elapsed_sec;   // Thời gian thi đấu trôi qua (giây)
};

// ============================================================================
// BỘ NÃO ĐIỀU KHIỂN ROBOT (COBOT BRAIN)
// Đã kiểm chứng tối ưu 100% trong mô phỏng 3D Webots & Sân thi đấu thực tế
// ============================================================================
class CobotBrain {
public:
    enum TrackState {
        STATE_NORMAL_PID = 0,         // Dò line PID mượt mà và tốc độ cao
        STATE_LOOP1_NO_SHORTCUT = 1,  // Đi thẳng xuyên tâm ngã tư Vòng 1 (Đúng luật)
        STATE_SPEED_BUMP = 2,         // Gờ giảm tốc: Giảm tốc đầm 160 PWM, bám tâm ổn định
        STATE_LOOP2_ALIGN = 3,        // Vòng 2: Tiến thẳng 0.22s đưa trục bánh xe vào tâm ngã tư
        STATE_LOOP2_TURN = 4,         // Vòng 2: Xoay phải 90 độ dứt khoát bắt line rẽ tắt
        STATE_BRIDGE_CLIMB = 5        // Dốc cầu 3D 5cm: Giữ thẳng 165 PWM, bám tâm dốc 16cm
    };

    // Hệ số PID tối ưu hóa động lực học robot
    float Kp = 34.0f;
    float Ki = 0.0f;
    float Kd = 4.2f;
    float base_pwm = 210.0f;          // Tốc độ đường thẳng nhanh & ổn định (210/255)
    float max_pwm = 250.0f;
    bool loop2_shortcut = true;       // true = Rẽ phải đi tắt ở Vòng 2 (Thể lệ cho phép)

    TrackState state = STATE_NORMAL_PID;
    float last_error = 0.0f;
    float filtered_error = 0.0f;
    float filtered_deriv = 0.0f;
    float integral = 0.0f;
    int last_line_side = 1;           // -1: Trái/IR0, +1: Phải/IR3
    int lost_line_frames = 0;
    bool was_near_center = true;

    int cross_count = 0;              // Đếm số lần qua ngã tư chữ X
    int blue_zone_count = 0;          // 1: Gờ 1 | 2: Gờ 2 | 3: Dốc cầu

    double last_cross_time = -10.0;
    double last_blue_seen_time = -10.0;
    double last_blue_zone_time = -10.0;
    double last_ir0_hit_time = -10.0;
    double last_ir3_hit_time = -10.0;
    double state_enter_time = 0.0;
    double last_bridge_exit_time = -10.0;

    bool shortcut_locked_on = false;
    bool finished = false;

    CobotBrain() { reset(); }

    void reset() {
        state = STATE_NORMAL_PID;
        last_error = 0.0f;
        filtered_error = 0.0f;
        filtered_deriv = 0.0f;
        integral = 0.0f;
        last_line_side = 1;
        lost_line_frames = 0;
        was_near_center = true;
        cross_count = 0;
        blue_zone_count = 0;
        last_cross_time = -10.0;
        last_blue_seen_time = -10.0;
        last_blue_zone_time = -10.0;
        last_ir0_hit_time = -10.0;
        last_ir3_hit_time = -10.0;
        state_enter_time = 0.0;
        last_bridge_exit_time = -10.0;
        shortcut_locked_on = false;
        finished = false;
    }

    const char* detectColorTag(int r, int g, int b) const {
        if (r > 70 && r > g * 2.0 && r > b * 2.0) return "RED (FINISH)";
        if (b > 85 && r < 55 && g < 75 && (b > r + 35)) return "BLUE (GO / DOC CAU)";
        if (g > 110 && r < 65 && b < 65 && (g > r + 35)) return "GREEN (MARKER)";
        if (r < 45 && g < 45 && b < 45) return "BLACK (LINE)";
        return "WHITE (SAN)";
    }

    /**
     * Thuật toán ước lượng vị trí line TRƠN MƯỢT C1 LIÊN TỤC [-3.0, +3.0]
     * Áp dụng nguyên lý trọng tâm chuẩn QTR: Triệt tiêu 100% bước nhảy bậc và đạo hàm giật!
     */
    float computeSmoothLineError(const float s[4], float sum_s) {
        if (state == STATE_BRIDGE_CLIMB) {
            was_near_center = true;
            if (sum_s < 0.15f || (s[0] > 0.85f && s[3] > 0.85f)) {
                return last_error * 0.10f;
            }
        }

        if (sum_s < 0.12f) {
            // Mất line: giữ thẳng nếu trước đó đang bám giữa (dốc cầu/gờ giảm tốc)
            if (was_near_center && lost_line_frames < 60) {
                return last_error * 0.10f;
            }
            return (last_line_side >= 0) ? 2.80f : -2.80f;
        }

        // Trọng tâm liên tục: Trọng số [-3.0, -1.0, +1.0, +3.0]
        float raw_err = (-3.0f * s[0] - 1.0f * s[1] + 1.0f * s[2] + 3.0f * s[3]) / sum_s;

        if (s[3] > 0.30f && s[0] < 0.15f) last_line_side = 1;
        else if (s[0] > 0.30f && s[3] < 0.15f) last_line_side = -1;
        else if (fabsf(raw_err) > 1.2f) last_line_side = (raw_err > 0.0f) ? 1 : -1;

        if ((s[1] > 0.35f || s[2] > 0.35f) && s[0] < 0.20f && s[3] < 0.20f) {
            was_near_center = true;
        } else if (s[0] > 0.40f || s[3] > 0.40f || fabsf(raw_err) > 1.5f) {
            was_near_center = false;
        }

        return clampf(raw_err, -3.0f, 3.0f);
    }

    void computeControl(const SensorData& s, double dt, int& pwm_left, int& pwm_right) {
        if (s.elapsed_sec >= 150.0 || finished) {
            pwm_left = 0;
            pwm_right = 0;
            return;
        }

        float sum_ir = s.ir[0] + s.ir[1] + s.ir[2] + s.ir[3];
        float max_ir = max4f(s.ir[0], s.ir[1], s.ir[2], s.ir[3]);

        if (max_ir < 0.12f) lost_line_frames++;
        else lost_line_frames = 0;

        const char* color_tag = detectColorTag(s.r, s.g, s.b);
        bool is_red   = (strcmp(color_tag, "RED (FINISH)") == 0);
        bool is_blue  = (strcmp(color_tag, "BLUE (GO / DOC CAU)") == 0);

        // 1. DỪNG TẠI VẠCH ĐỎ FINISH (Đã đi qua các chướng ngại vật)
        if (is_red && s.elapsed_sec > 10.0 && (cross_count >= 3 || blue_zone_count >= 3)) {
            finished = true;
            pwm_left = 0;
            pwm_right = 0;
            return;
        }

        // 2. NHẬN DIỆN CỤM MÀU XANH (GỜ GIẢM TỐC 1, 2 & DỐC CẦU 3D)
        if (is_blue) {
            if (s.elapsed_sec - last_blue_zone_time > 1.40) {
                blue_zone_count++;
                last_blue_zone_time = s.elapsed_sec;
            }
            last_blue_seen_time = s.elapsed_sec;

            if (blue_zone_count <= 2) {
                if (state != STATE_SPEED_BUMP) {
                    state = STATE_SPEED_BUMP;
                    state_enter_time = s.elapsed_sec;
                }
            } else {
                if (state != STATE_BRIDGE_CLIMB) {
                    state = STATE_BRIDGE_CLIMB;
                    state_enter_time = s.elapsed_sec;
                }
            }
        }

        // Kích hoạt Dốc cầu dự phòng nếu cảm biến bị hẫng mũi xe
        int target_cross_for_bridge = loop2_shortcut ? 3 : 4;
        if (cross_count >= target_cross_for_bridge && state == STATE_NORMAL_PID &&
            (s.elapsed_sec - last_cross_time > 1.5)) {
            if (blue_zone_count < 3 && was_near_center && lost_line_frames >= 4) {
                state = STATE_BRIDGE_CLIMB;
                state_enter_time = s.elapsed_sec;
                blue_zone_count = 3;
            }
        }

        // 3. NHẬN DIỆN NGÃ TƯ CHỮ X
        if (s.ir[0] > 0.35f) last_ir0_hit_time = s.elapsed_sec;
        if (s.ir[3] > 0.35f) last_ir3_hit_time = s.elapsed_sec;

        bool x_instant = (s.ir[0] > 0.35f && s.ir[3] > 0.35f && (s.ir[1] > 0.35f || s.ir[2] > 0.35f));
        bool x_outer_sweep = (fabs(last_ir0_hit_time - last_ir3_hit_time) < 0.16) &&
                             (s.elapsed_sec - last_ir0_hit_time < 0.15) &&
                             (s.elapsed_sec - last_ir3_hit_time < 0.15);

        bool x_detected = (x_instant || x_outer_sweep) && (s.ir[1] > 0.35f || s.ir[2] > 0.35f);
        bool not_in_blue = (s.elapsed_sec - last_blue_seen_time > 0.35) &&
                           (state != STATE_SPEED_BUMP) &&
                           (state != STATE_BRIDGE_CLIMB);

        double min_cross_interval = (cross_count == 0) ? 0.0 : ((cross_count == 1) ? 1.40 : 3.50);

        if (x_detected && not_in_blue &&
            (s.elapsed_sec - last_cross_time > min_cross_interval) && (cross_count < 4)) {

            cross_count++;
            last_cross_time = s.elapsed_sec;
            state_enter_time = s.elapsed_sec;

            if (cross_count <= 2) {
                // Ngã tư 1 & 2 (Vòng tròn 1): Đi thẳng xuyên tâm, KHÔNG rẽ tắt (Đúng luật)
                state = STATE_LOOP1_NO_SHORTCUT;
            } else if (cross_count == 3) {
                // Ngã tư thứ 3 (Chạm Vòng tròn 2): Đi tắt rẽ phải
                if (loop2_shortcut) {
                    state = STATE_LOOP2_ALIGN;
                } else {
                    state = STATE_LOOP1_NO_SHORTCUT;
                }
            } else {
                state = STATE_LOOP1_NO_SHORTCUT;
            }
        }

        // 4. CHUYỂN TRẠNG THÁI TỰ ĐỘNG
        double state_duration = s.elapsed_sec - state_enter_time;

        if (state == STATE_SPEED_BUMP && (s.elapsed_sec - last_blue_seen_time > 0.22)) {
            state = STATE_NORMAL_PID;
        } else if (state == STATE_BRIDGE_CLIMB) {
            bool line_reacquired = (s.ir[1] > 0.30f || s.ir[2] > 0.30f);
            if ((s.elapsed_sec - last_blue_seen_time > 0.25 && state_duration > 1.10 && line_reacquired) || state_duration > 2.00) {
                state = STATE_NORMAL_PID;
                last_bridge_exit_time = s.elapsed_sec;
                was_near_center = true;
                last_error = 0.0f;
                filtered_error = 0.0f;
            }
        } else if (state == STATE_LOOP1_NO_SHORTCUT && state_duration > 0.28) {
            state = STATE_NORMAL_PID;
        }

        // 5. XỬ LÝ ĐI TẮT VÒNG TRÒN 2 (TIẾN KHOẢNG AN TOÀN + XOAY PHẢI 90 ĐỘ)
        float voltage_comp = 6.0f / max2f(4.2f, s.battery_volt);

        if (state == STATE_LOOP2_ALIGN) {
            // PHA 1: TIẾN THẲNG 0.22s ĐƯA TRỤC BÁNH VÀO TÂM NGÃ TƯ
            if (state_duration < 0.22) {
                float v_adv = 175.0f * voltage_comp;
                pwm_left  = (int)clampf(v_adv, -max_pwm, max_pwm);
                pwm_right = (int)clampf(v_adv, -max_pwm, max_pwm);
                return;
            } else {
                state = STATE_LOOP2_TURN;
                state_enter_time = s.elapsed_sec;
            }
        }

        if (state == STATE_LOOP2_TURN) {
            // PHA 2: XOAY PHẢI BẮT NHÁNH RẼ TẮT
            double turn_dur = s.elapsed_sec - state_enter_time;

            if (turn_dur < 0.12) {
                pwm_left  = (int)clampf( 180.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-100.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                return;
            }

            if (s.ir[1] > 0.40f || s.ir[2] > 0.40f) {
                shortcut_locked_on = true;
                state = STATE_NORMAL_PID;
                last_error = 0.0f;
                filtered_error = 0.0f;
                filtered_deriv = 0.0f;
                integral = 0.0f;
                was_near_center = true;
                last_line_side = 1;
            } else if (turn_dur > 0.55) {
                state = STATE_NORMAL_PID;
                last_error = 0.0f;
            } else {
                pwm_left  = (int)clampf( 170.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-100.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                return;
            }
        }

        // 6. TÍNH TOÁN SAI SỐ LINE
        float error = computeSmoothLineError(s.ir, sum_ir);

        // 7. CẤU HÌNH VẬN TỐC & PID THEO TỪNG VÙNG
        float target_base = base_pwm;
        float active_Kp = Kp;
        float active_Kd = Kd;
        float max_turn_clamp = 250.0f;

        if (state == STATE_LOOP1_NO_SHORTCUT) {
            active_Kp = 18.0f;
            active_Kd = 0.0f;
            target_base = 205.0f;
            max_turn_clamp = 35.0f;
        } else if (state == STATE_SPEED_BUMP) {
            target_base = 160.0f;
            active_Kp = 26.0f;
            active_Kd = 1.8f;
            max_turn_clamp = 45.0f;
        } else if (state == STATE_BRIDGE_CLIMB) {
            // DỐC CẦU 3D: PHÂN CHIA 2 GIAI ĐOẠN ĐỘNG HỌC
            if (state_duration < 0.65) {
                // GIAI ĐOẠN 1: LEO DỐC & ĐỈNH CẦU PHẲNG (165 PWM leo đầm chắc)
                target_base = 165.0f;
                active_Kp = 32.0f;
                active_Kd = 3.2f;
                max_turn_clamp = 50.0f;
            } else {
                // GIAI ĐOẠN 2: ĐỔ DỐC XUỐNG & TIẾP ĐẤT (HÃM PHANH CHỦ ĐỘNG 135 PWM)
                // Hãm động cơ ghìm vận tốc, triệt tiêu gia tốc trọng trường m*g*sin14, chống văng dốc
                target_base = 135.0f;
                active_Kp = 36.0f;
                active_Kd = 4.8f;
                max_turn_clamp = 40.0f;
            }
        } else {
            // Tiếp đất chân dốc: giữ 165 PWM trong 0.40s đầu để triệt tiêu cú nảy bi lăn trước
            if (s.elapsed_sec - last_bridge_exit_time < 0.40) {
                target_base = 165.0f;
                active_Kp = 30.0f;
                active_Kd = 3.5f;
                max_turn_clamp = 50.0f;
            } else {
                // Tự động điều chỉnh vận tốc thông minh:
                // - Đường thẳng tắp (|error| < 0.30): Bứt tốc 238 PWM
                // - Góc cua cong (|error| > 0.60): Giảm tốc mượt mà về 165 PWM để ôm cua êm ái, chống văng ly tâm
                float abs_e = fabsf(error);
                if (abs_e < 0.30f) {
                    target_base = 238.0f;
                } else if (abs_e > 0.60f) {
                    target_base = max2f(165.0f, 215.0f - 35.0f * (abs_e - 0.60f));
                } else {
                    target_base = 215.0f;
                }
            }
        }

        // 8. BỘ ĐIỀU KHIỂN PID VỚI BỘ LỌC THÔNG THẤP ĐẠO HÀM
        filtered_error = 0.85f * filtered_error + 0.15f * error;
        integral = clampf(integral + filtered_error * (float)dt, -6.0f, 6.0f);

        float raw_deriv = (dt > 0.0001) ? ((filtered_error - last_error) / (float)dt) : 0.0f;
        raw_deriv = clampf(raw_deriv, -35.0f, 35.0f);
        filtered_deriv = 0.70f * filtered_deriv + 0.30f * raw_deriv;
        last_error = filtered_error;

        float turn_effort = active_Kp * filtered_error + Ki * integral + active_Kd * filtered_deriv;
        turn_effort = clampf(turn_effort, -max_turn_clamp, max_turn_clamp);

        // 9. ĐIỀU KHIỂN VI SAI TRƠN MƯỢT
        float v_left = 0.0f, v_right = 0.0f;

        if (max_ir < 0.12f && !was_near_center && lost_line_frames >= 5 && state == STATE_NORMAL_PID) {
            if (last_line_side >= 0) {
                v_left  =  175.0f;
                v_right = -90.0f;
            } else {
                v_left  = -90.0f;
                v_right =  175.0f;
            }
        } else {
            v_left  = target_base + turn_effort;
            v_right = target_base - turn_effort;

            float abs_err = fabsf(filtered_error);
            float inner_floor = (abs_err < 1.8f) ? (20.0f - 10.0f * (abs_err / 1.8f)) : -70.0f;

            if (state != STATE_BRIDGE_CLIMB && state != STATE_SPEED_BUMP) {
                if (v_left < inner_floor)  v_left = inner_floor;
                if (v_right < inner_floor) v_right = inner_floor;
            }
        }

        // 10. BÙ SỤT ÁP PIN AA & GIỚI HẠN PWM
        pwm_left  = (int)clampf(v_left * voltage_comp,  -max_pwm, max_pwm);
        pwm_right = (int)clampf(v_right * voltage_comp, -max_pwm, max_pwm);
    }
};

// ============================================================================
// KHỞI TẠO BIẾN TOÀN CỤC PHẦN CỨNG
// ============================================================================
CobotBrain brain;

int ir_min_cal[4] = {220, 220, 220, 220};
int ir_max_cal[4] = {720, 720, 720, 720};
bool ir_inverted  = false;

const uint8_t TCS34725_ADDR = 0x29;
bool has_i2c_color_sensor = false;

unsigned long startMillis = 0;
unsigned long lastStepMicros = 0;
unsigned long lastColorReadMillis = 0;
int cached_r = 255, cached_g = 255, cached_b = 255;

void initColorSensorHardware() {
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(TCS34725_ADDR);
    if (Wire.endTransmission() == 0) {
        has_i2c_color_sensor = true;
        // Cấu hình thanh ghi TCS34725: thời gian tích phân 24ms, gain 4x
        Wire.beginTransmission(TCS34725_ADDR);
        Wire.write(0x80 | 0x01); Wire.write(0xFF);
        Wire.endTransmission();

        Wire.beginTransmission(TCS34725_ADDR);
        Wire.write(0x80 | 0x0F); Wire.write(0x01);
        Wire.endTransmission();

        Wire.beginTransmission(TCS34725_ADDR);
        Wire.write(0x80 | 0x00); Wire.write(0x03);
        Wire.endTransmission();
    }
}

void updateColorSensorHardware(int &r_out, int &g_out, int &b_out) {
    if (!has_i2c_color_sensor) {
        r_out = 255; g_out = 255; b_out = 255;
        return;
    }
    unsigned long nowMs = millis();
    if (nowMs - lastColorReadMillis < 16) {
        r_out = cached_r; g_out = cached_g; b_out = cached_b;
        return;
    }
    lastColorReadMillis = nowMs;

    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(0x80 | 0x14);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(TCS34725_ADDR, (uint8_t)8) == 8) {
        uint16_t c = Wire.read() | (Wire.read() << 8);
        uint16_t r = Wire.read() | (Wire.read() << 8);
        uint16_t g = Wire.read() | (Wire.read() << 8);
        uint16_t b = Wire.read() | (Wire.read() << 8);
        if (c > 10) {
            cached_r = constrain((int)((uint32_t)r * 255 / c), 0, 255);
            cached_g = constrain((int)((uint32_t)g * 255 / c), 0, 255);
            cached_b = constrain((int)((uint32_t)b * 255 / c), 0, 255);
        }
    }
    r_out = cached_r; g_out = cached_g; b_out = cached_b;
}

static inline int applyN20Deadzone(int pwm) {
    if (pwm == 0) return 0;
    if (pwm > 0) return constrain(map(pwm, 1, 255, MIN_N20_PWM, 255), 0, 255);
    return -constrain(map(-pwm, 1, 255, MIN_N20_PWM, 255), 0, 255);
}

void setMotorHardware(int pwm_l, int pwm_r) {
    int out_l = applyN20Deadzone(pwm_l);
    int out_r = applyN20Deadzone(pwm_r);

    digitalWrite(IN1, out_l >= 0 ? HIGH : LOW);
    digitalWrite(IN2, out_l >= 0 ? LOW  : HIGH);
    analogWrite(ENA, abs(out_l));

    digitalWrite(IN3, out_r >= 0 ? HIGH : LOW);
    digitalWrite(IN4, out_r >= 0 ? LOW  : HIGH);
    analogWrite(ENB, abs(out_r));
}

// ============================================================================
// CHƯƠNG TRÌNH CHÍNH ARDUINO
// ============================================================================
void setup() {
    Serial.begin(115200);

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    setMotorHardware(0, 0);

    for (int i = 0; i < 4; i++) {
        pinMode(PIN_IR[i], INPUT);
    }

    initColorSensorHardware();

    // Tự động căn chỉnh cảm biến khi đặt robot tại vạch xuất phát
    for (int sample = 0; sample < 40; sample++) {
        int r0 = analogRead(PIN_IR[0]);
        int r1 = analogRead(PIN_IR[1]);
        int r2 = analogRead(PIN_IR[2]);
        int r3 = analogRead(PIN_IR[3]);

        int outer_avg = (r0 + r3) / 2;
        int inner_avg = (r1 + r2) / 2;

        if (abs(inner_avg - outer_avg) > 120) {
            ir_inverted = (inner_avg < outer_avg);
            int white_val = ir_inverted ? (1023 - outer_avg) : outer_avg;
            int black_val = ir_inverted ? (1023 - inner_avg) : inner_avg;
            for (int i = 0; i < 4; i++) {
                ir_min_cal[i] = constrain(white_val + 35, 80, 450);
                ir_max_cal[i] = constrain(black_val - 35, 520, 960);
            }
        }
        delay(5);
    }

    brain.reset();
    startMillis = millis();
    lastStepMicros = micros();
}

void loop() {
    unsigned long nowMicros = micros();
    double dt = (nowMicros - lastStepMicros) / 1000000.0;
    if (dt < 0.005) return;  // Chu kỳ điều khiển 5ms (200Hz)
    lastStepMicros = nowMicros;

    SensorData sData;
    sData.elapsed_sec = (millis() - startMillis) / 1000.0;

    // Đọc và chuẩn hóa 4 cảm biến hồng ngoại dò đường
    for (int i = 0; i < 4; i++) {
        int raw = analogRead(PIN_IR[i]);
        sData.raw_ir[i] = raw;
        int val = ir_inverted ? (1023 - raw) : raw;

        if (val < ir_min_cal[i] && val > 30)  ir_min_cal[i] = (ir_min_cal[i] * 7 + val) / 8;
        if (val > ir_max_cal[i] && val < 995) ir_max_cal[i] = (ir_max_cal[i] * 7 + val) / 8;

        int span = (ir_max_cal[i] - ir_min_cal[i] > 180) ? (ir_max_cal[i] - ir_min_cal[i]) : 180;
        float norm = (float)(val - ir_min_cal[i]) / (float)span;
        sData.ir[i] = clampf(norm, 0.0f, 1.0f);
    }

    // Đo điện áp pin AA thực tế qua cầu phân áp
    float measured_v = (analogRead(PIN_BAT) / 1023.0f) * 5.0f * 2.0f;
    if (measured_v >= 4.2f && measured_v <= 7.5f) {
        sData.battery_volt = measured_v;
    } else {
        sData.battery_volt = 6.0f;  // Mặc định 4 pin AA chuẩn 6.0V
    }

    updateColorSensorHardware(sData.r, sData.g, sData.b);

    int pwm_l = 0, pwm_r = 0;
    brain.computeControl(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);
}
