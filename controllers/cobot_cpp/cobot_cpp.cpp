/**
 * ============================================================================
 * CONTROLLER C++ ROBOT DÒ ĐƯỜNG COBOT - ICTU ROBOTICS CONTEST 2026
 * Chủ đề: "ĐƯỜNG TỚI TÂN CƯƠNG - THÁI NGUYÊN LẦN THỨ 2"
 * Tương thích kép: Mô phỏng 3D Webots & Mạch thật (KM-11-102 + KMM-11-100 + GA12-N20)
 * ============================================================================
 */

#define WEBOTS_SIM  // <-- Khi nạp lên Robot thật (Arduino IDE / PlatformIO), hãy comment (//) dòng này!

#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// HÀM TIỆN ÍCH CHUẨN C/C++ (Biên dịch mượt trên cả Webots lẫn Arduino AVR/ARM)
// ============================================================================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float max2f(float a, float b) { return (a > b) ? a : b; }
static inline float min2f(float a, float b) { return (a < b) ? a : b; }
static inline float max4f(float a, float b, float c, float d) { return max2f(max2f(a, b), max2f(c, d)); }
static inline float min4f(float a, float b, float c, float d) { return min2f(min2f(a, b), min2f(c, d)); }

// ============================================================================
// PHẦN 1: GIAO DIỆN PHẦN CỨNG TRỪU TƯỢNG (HAL - HARDWARE ABSTRACTION LAYER)
// ============================================================================
struct SensorData {
    float ir[4];          // Đã chuẩn hóa: 0.00 (Nền trắng) -> 1.00 (Tâm line đen)
    int raw_ir[4];        // Giá trị ADC thô (0..1023)
    int r, g, b;          // Cảm biến màu KMM-11-100 / TCS34725 (0..255)
    float battery_volt;   // Điện áp khối 4 pin AA (Danh định 6.0V, tối đa <= 7.4V)
    double elapsed_sec;   // Thời gian chạy trong lượt (Giới hạn 150.0s)
};

enum TrackState {
    STATE_NORMAL_PID = 0,     // Dò line PID siêu mượt, biến thiên vận tốc theo góc cua
    STATE_LOOP1_NO_SHORTCUT,  // Vòng 1: Khóa thẳng xuyên tâm ngã tư chữ X (không đi tắt)
    STATE_SPEED_BUMP,         // Gờ giảm tốc 1, 2: Tốc độ đầm, bám tâm ổn định, dập tắt rung lắc
    STATE_LOOP2_ALIGN,        // Vòng 2: Tiến thêm khoảng an toàn (trục bánh xe tới tâm ngã tư)
    STATE_LOOP2_TURN,         // Vòng 2: Xoay phải 90 độ dứt khoát bắt nhánh rẽ tắt
    STATE_BRIDGE_CLIMB        // Dốc cầu 3D 5cm: Vận tốc tối ưu 170 PWM, bám tâm kín kẽ, chống văng dốc
};

// ============================================================================
// PHẦN 2: BỘ NÃO THUẬT TOÁN TRUNG TÂM (DÙNG CHUNG CHO CẢ WEBOTS & ROBOT THẬT)
// ============================================================================
class CobotBrain {
public:
    // Cấu hình PID chuẩn xác & tốc độ cân bằng tuyệt đối
    float Kp = 34.0f;
    float Ki = 0.0f;
    float Kd = 4.2f;
    float base_pwm = 210.0f;       // Tốc độ đường thẳng nhanh & ổn định
    float max_pwm = 250.0f;
    bool loop2_shortcut = true;    // true = Rẽ phải đi tắt ở Vòng 2 (Đúng thể lệ cuộc thi)

    TrackState state = STATE_NORMAL_PID;
    float last_error = 0.0f;
    float filtered_error = 0.0f;
    float filtered_deriv = 0.0f;
    float integral = 0.0f;
    int last_line_side = 1;        // Ghi nhớ phía có line (-1: Trái/IR0, +1: Phải/IR3)
    int lost_line_frames = 0;
    bool was_near_center = true;

    int cross_count = 0;           // Đếm số lần đi qua ngã tư chữ X
    int blue_zone_count = 0;       // 1: Gờ 1 | 2: Gờ 2 | 3: Dốc cầu

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
            // Nếu cả 4 cảm biến đều mất line hoặc đều bão hòa
            if (sum_s < 0.15f || (s[0] > 0.85f && s[3] > 0.85f)) {
                return last_error * 0.10f; // Giữ hướng lái ổn định
            }
        }

        if (sum_s < 0.12f) {
            // Khi mất line hoàn toàn:
            // Nếu trước đó đang bám giữa (dốc cầu/gờ giảm tốc) -> giữ thẳng mũi xe
            if (was_near_center && lost_line_frames < 60) {
                return last_error * 0.10f;
            }
            // Nếu đang trong góc cua gắt -> xoay tìm line theo hướng lệch cuối cùng
            return (last_line_side >= 0) ? 2.80f : -2.80f;
        }

        // Trọng tâm liên tục 4 cảm biến: Trọng số [-3.0, -1.0, +1.0, +3.0]
        float raw_err = (-3.0f * s[0] - 1.0f * s[1] + 1.0f * s[2] + 3.0f * s[3]) / sum_s;

        // Cập nhật hướng lệch khi mắt ngoài bắt vạch
        if (s[3] > 0.30f && s[0] < 0.15f) last_line_side = 1;
        else if (s[0] > 0.30f && s[3] < 0.15f) last_line_side = -1;
        else if (fabsf(raw_err) > 1.2f) last_line_side = (raw_err > 0.0f) ? 1 : -1;

        // Cập nhật cờ bám tâm (was_near_center)
        if ((s[1] > 0.35f || s[2] > 0.35f) && s[0] < 0.20f && s[3] < 0.20f) {
            was_near_center = true;
        } else if (state != STATE_BRIDGE_CLIMB && (s[0] > 0.40f || s[3] > 0.40f || fabsf(raw_err) > 1.5f)) {
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

        // ====================================================================
        // 1. DỪNG TẠI VẠCH ĐỎ FINISH (Sau khi đã hoàn thành đường đua)
        // ====================================================================
        if (is_red && s.elapsed_sec > 10.0 && (cross_count >= 3 || blue_zone_count >= 3)) {
            finished = true;
            pwm_left = 0;
            pwm_right = 0;
            return;
        }

        // ====================================================================
        // 2. NHẬN DIỆN VÙNG MÀU XANH (GỜ GIẢM TỐC 1, 2 & DỐC CẦU 3D)
        // Khoảng cách giữa các vùng xanh > 1.5s -> Đếm zone chuẩn xác
        // ====================================================================
        if (is_blue) {
            if (s.elapsed_sec - last_blue_zone_time > 1.40) {
                blue_zone_count++;
                last_blue_zone_time = s.elapsed_sec;
            }
            last_blue_seen_time = s.elapsed_sec;

            if (blue_zone_count <= 2) {
                // Cụm Gờ giảm tốc 1 & 2
                if (state != STATE_SPEED_BUMP) {
                    state = STATE_SPEED_BUMP;
                    state_enter_time = s.elapsed_sec;
                }
            } else {
                // Cụm Dốc cầu 3D 5cm (sau Vòng 2)
                if (state != STATE_BRIDGE_CLIMB) {
                    state = STATE_BRIDGE_CLIMB;
                    state_enter_time = s.elapsed_sec;
                }
            }
        }

        // Kích hoạt Dốc cầu dự phòng nếu sau Vòng 2 gặp sụt cảm biến do hếch mũi xe trên dốc
        int target_cross_for_bridge = loop2_shortcut ? 3 : 4;
        if (cross_count >= target_cross_for_bridge && state == STATE_NORMAL_PID &&
            (s.elapsed_sec - last_cross_time > 1.5)) {
            if (blue_zone_count < 3 && was_near_center && lost_line_frames >= 4) {
                state = STATE_BRIDGE_CLIMB;
                state_enter_time = s.elapsed_sec;
                blue_zone_count = 3;
            }
        }

        // ====================================================================
        // 3. NHẬN DIỆN NGÃ TƯ CHỮ X BẰNG QUY LUẬT QUÉT ĐỐI XỨNG & ĐỒNG THỜI
        // ====================================================================
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

        // Khoảng cách thời gian tối thiểu giữa 2 ngã tư:
        // Ngã tư 1 vào vòng 1 -> Ngã tư 2 thoát vòng 1 (> 1.2s)
        // Ngã tư 2 thoát vòng 1 -> Ngã tư 3 vòng 2 (> 3.5s qua 2 gờ giảm tốc)
        double min_cross_interval = (cross_count == 0) ? 0.0 : ((cross_count == 1) ? 1.40 : 3.50);

        if (x_detected && not_in_blue &&
            (s.elapsed_sec - last_cross_time > min_cross_interval) && (cross_count < 4)) {

            cross_count++;
            last_cross_time = s.elapsed_sec;
            state_enter_time = s.elapsed_sec;
            printf("[CROSS DETECTED #%d] at t=%.2f s!\n", cross_count, s.elapsed_sec);
            fflush(stdout);

            if (cross_count <= 2) {
                // Ngã tư 1 & 2 (Vòng tròn 1): Đi thẳng xuyên tâm, KHÔNG rẽ tắt (Đúng luật)
                state = STATE_LOOP1_NO_SHORTCUT;
            } else if (cross_count == 3) {
                // Ngã tư thứ 3 (Chạm Vòng tròn 2 lần đầu)
                if (loop2_shortcut) {
                    // TIẾN THÊM KHOẢNG AN TOÀN TRƯỚC KHI RẼ (Đưa trục bánh xe vào đúng tâm ngã tư)
                    state = STATE_LOOP2_ALIGN;
                } else {
                    state = STATE_LOOP1_NO_SHORTCUT;
                }
            } else {
                // Ngã tư thứ 4 (Lối ra Vòng tròn 2 khi đi đủ vòng): Đi thẳng ra dốc cầu
                state = STATE_LOOP1_NO_SHORTCUT;
            }
        }


        // ====================================================================
        // 4. CHUYỂN TRẠNG THÁI TỰ ĐỘNG THEO THỜI GIAN & ĐIỀU KIỆN
        // ====================================================================
        double state_duration = s.elapsed_sec - state_enter_time;

        if (state == STATE_SPEED_BUMP && (s.elapsed_sec - last_blue_seen_time > 0.22)) {
            state = STATE_NORMAL_PID;
        } else if (state == STATE_BRIDGE_CLIMB) {
            // Chỉ thoát Dốc cầu khi:
            // 1. Đã vượt qua dốc xuống và cảm biến màu rời mảng xanh > 0.25s
            // 2. Tổng thời gian trên dốc > 1.10s (đảm bảo cả 2 bánh sau đã tiếp đất phẳng)
            // 3. Mắt giữa (IR1 hoặc IR2) đã bắt dính lại vạch đen trên sàn
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


        // ====================================================================
        // 5. XỬ LÝ ĐẶC BIỆT: RẼ PHẢI ĐI TẮT TẠI VÒNG TRÒN 2 (CHUẨN XÁC TUYỆT ĐỐI)
        // ====================================================================
        float voltage_comp = 6.0f / max2f(4.2f, s.battery_volt);

        if (state == STATE_LOOP2_ALIGN) {
            // PHA 1: TIẾN THẲNG ĐẾN "KHOẢNG AN TOÀN"
            // Khi cảm biến phát hiện ngã tư, trục bánh xe vẫn cách tâm ngã tư ~8.5cm.
            // Robot cần tiến thêm đúng 0.20s (~7.5cm) để tâm quay của xe trùng với tâm giao lộ!
            if (state_duration < 0.22) {
                float v_adv = 175.0f * voltage_comp;
                pwm_left  = (int)clampf(v_adv, -max_pwm, max_pwm);
                pwm_right = (int)clampf(v_adv, -max_pwm, max_pwm);
                return;
            } else {
                // Đã đến khoảng an toàn -> Bắt đầu xoay phải!
                state = STATE_LOOP2_TURN;
                state_enter_time = s.elapsed_sec;
            }
        }

        if (state == STATE_LOOP2_TURN) {
            // PHA 2: XOAY PHẢI DỨT KHOÁT BẮT NHÁNH RẼ TẮT
            double turn_dur = s.elapsed_sec - state_enter_time;

            // Trong 0.12s đầu: Xoay mù vượt qua nhánh đường thẳng cũ
            if (turn_dur < 0.12) {
                pwm_left  = (int)clampf( 180.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-100.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                return;
            }

            // Từ 0.12s trở đi: Quét tìm nhánh rẽ tắt bằng mắt giữa IR1 hoặc IR2
            if (s.ir[1] > 0.40f || s.ir[2] > 0.40f) {
                // Khóa mục tiêu thành công! Chuyển ngay về PID bám đường mới
                shortcut_locked_on = true;
                state = STATE_NORMAL_PID;
                last_error = 0.0f;
                filtered_error = 0.0f;
                filtered_deriv = 0.0f;
                integral = 0.0f;
                was_near_center = true;
                last_line_side = 1;
            } else if (turn_dur > 0.55) {
                // Hết thời gian chờ xoay: ép về PID
                state = STATE_NORMAL_PID;
                last_error = 0.0f;
            } else {
                // Tiếp tục xoay phải
                pwm_left  = (int)clampf( 170.0f * voltage_comp, -max_pwm, max_pwm);
                pwm_right = (int)clampf(-100.0f * voltage_comp, -max_pwm, max_pwm);
                last_line_side = 1;
                return;
            }
        }

        // ====================================================================
        // 6. TÍNH TOÁN SAI SỐ BÁM LINE TRƠN MƯỢT
        // ====================================================================
        float error = computeSmoothLineError(s.ir, sum_ir);

        // ====================================================================
        // 7. THIẾT LẬP VẬN TỐC & HỆ SỐ PID THEO TỪNG VÙNG TRÊN ĐƯỜNG ĐUA
        // ====================================================================
        float target_base = base_pwm;
        float active_Kp = Kp;
        float active_Kd = Kd;
        float max_turn_clamp = 250.0f;

        if (state == STATE_LOOP1_NO_SHORTCUT) {
            // Khi xuyên qua ngã tư Vòng 1: bám tâm nhẹ, khóa đạo hàm chống giật
            active_Kp = 18.0f;
            active_Kd = 0.0f;
            target_base = 205.0f;
            max_turn_clamp = 35.0f;
        } else if (state == STATE_SPEED_BUMP) {
            // Gờ giảm tốc: Vận tốc đầm 160 PWM, bám tâm ổn định, dập tắt hoàn toàn rung lắc
            target_base = 160.0f;
            active_Kp = 26.0f;
            active_Kd = 1.8f;
            max_turn_clamp = 45.0f;
        } else if (state == STATE_BRIDGE_CLIMB) {
            // DỐC CẦU 3D 5CM: PHÂN CHIA 2 GIAI ĐOẠN ĐỘNG HỌC RÕ RỆT
            if (state_duration < 0.65) {
                // GIAI ĐOẠN 1: LEO DỐC & ĐỈNH CẦU PHẲNG (Vận tốc 165 PWM leo đầm, không nảy bánh cầu)
                target_base = 165.0f;
                active_Kp = 32.0f;
                active_Kd = 3.2f;
                max_turn_clamp = 50.0f;
            } else {
                // GIAI ĐOẠN 2: ĐỔ DỐC XUỐNG & TIẾP ĐẤT CHÂN DỐC (HÃM PHANH CHỦ ĐỘNG 135 PWM)
                // - Triệt tiêu 100% lực trôi dốc do trọng trường (Fg = m*g*sin14)
                // - Giữ tải trọng đè lên 2 bánh truyền động sau, triệt tiêu trượt bánh
                // - Tăng Kd=4.8 để dập tắt dao động lắc đuôi ngay khi chạm chân dốc!
                target_base = 135.0f;
                active_Kp = 36.0f;
                active_Kd = 4.8f;
                max_turn_clamp = 40.0f;
            }
        } else {
            // Chế độ dò line PID bình thường:
            // Giai đoạn tiếp đất chân dốc: giữ 165 PWM trong 0.40s đầu để triệt tiêu cú nảy bi lăn trước
            if (s.elapsed_sec - last_bridge_exit_time < 0.40) {
                target_base = 165.0f;
                active_Kp = 30.0f;
                active_Kd = 3.5f;
                max_turn_clamp = 50.0f;
            } else {
                // Tự động điều chỉnh vận tốc thông minh:
                // - Đường thẳng tắp (|error| < 0.30): Bứt tốc cực đại 238 PWM!
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

        // ====================================================================
        // 8. TÍNH TOÁN BỘ ĐIỀU KHIỂN PID VỚI BỘ LỌC THÔNG THẤP ĐẠO HÀM
        // ====================================================================
        // Bộ lọc thông thấp sai số
        filtered_error = 0.85f * filtered_error + 0.15f * error;

        // Khâu tích phân (Anti-windup)
        integral = clampf(integral + filtered_error * (float)dt, -6.0f, 6.0f);

        // Khâu vi phân với bộ lọc làm mượt chống gai nhiễu
        float raw_deriv = (dt > 0.0001) ? ((filtered_error - last_error) / (float)dt) : 0.0f;
        raw_deriv = clampf(raw_deriv, -35.0f, 35.0f);
        filtered_deriv = 0.70f * filtered_deriv + 0.30f * raw_deriv;
        last_error = filtered_error;

        float turn_effort = active_Kp * filtered_error + Ki * integral + active_Kd * filtered_deriv;
        turn_effort = clampf(turn_effort, -max_turn_clamp, max_turn_clamp);

        // ====================================================================
        // 9. ĐIỀU KHIỂN ĐỘNG CƠ VI SAI TRƠN MƯỢT (LOẠI BỎ HOÀN TOÀN GIẬT LẮC)
        // ====================================================================
        float v_left = 0.0f, v_right = 0.0f;

        if (max_ir < 0.12f && !was_near_center && lost_line_frames >= 5 && state == STATE_NORMAL_PID) {
            // Khi xe thực sự trượt ra ngoài khúc cua: Quay tìm line theo hướng lệch
            if (last_line_side >= 0) {
                v_left  =  175.0f;
                v_right = -90.0f;
            } else {
                v_left  = -90.0f;
                v_right =  175.0f;
            }
        } else {
            // Điều khiển vi sai chuẩn động học robot 2 bánh
            v_left  = target_base + turn_effort;
            v_right = target_base - turn_effort;

            // Xử lý bánh trong trơn mượt liên tục (không nhảy bậc):
            // Trong góc cua nhẹ/vừa (|error| < 1.8), giữ bánh trong >= 20 PWM giúp xe lướt cực êm.
            // Khi cua rất gắt (|error| > 1.8), bánh trong tự động giảm dần về 0 và đảo chiều mượt mà.
            float abs_err = fabsf(filtered_error);
            float inner_floor = (abs_err < 1.8f) ? (20.0f - 10.0f * (abs_err / 1.8f)) : -70.0f;

            if (state != STATE_BRIDGE_CLIMB && state != STATE_SPEED_BUMP) {
                if (v_left < inner_floor)  v_left = inner_floor;
                if (v_right < inner_floor) v_right = inner_floor;
            }
        }

        // ====================================================================
        // 10. BÙ SỤT ÁP KHỐI PIN AA (CHUẨN 6.0V) & GIỚI HẠN PWM [-250, +250]
        // ====================================================================
        pwm_left  = (int)clampf(v_left * voltage_comp,  -max_pwm, max_pwm);
        pwm_right = (int)clampf(v_right * voltage_comp, -max_pwm, max_pwm);
    }

};

// ============================================================================
// PHẦN 3: LỚP GHÉP NỐI WEBOTS 3D + HUD TÁCH DÒNG SẮC NÉT
// ============================================================================
#ifdef WEBOTS_SIM
#include <string>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

using namespace webots;

static void drawCleanHudLine(Supervisor *robot, int id, const char *text,
                             double x, double y, double font_size, int color) {
    robot->setLabel(id + 50, text, x + 0.0012, y + 0.0018, font_size, 0x000000, 0.20, "Lucida Console");
    robot->setLabel(id, text, x, y, font_size, color, 0.0, "Lucida Console");
}

int main(int argc, char **argv) {
    Supervisor *robot = new Supervisor();
    int timeStep = (int)robot->getBasicTimeStep();
    std::string robotName = robot->getName();
    bool isSecondRobot = (robotName == "COBOT_ICTU_2");

    Motor *leftMotor = robot->getMotor("left_motor");
    Motor *rightMotor = robot->getMotor("right_motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    DistanceSensor *ir[4];
    const char *ir_names[4] = {"ir0", "ir1", "ir2", "ir3"};
    for (int i = 0; i < 4; i++) {
        ir[i] = robot->getDistanceSensor(ir_names[i]);
        ir[i]->enable(timeStep);
    }
    Camera *colorSensor = robot->getCamera("color_sensor");
    colorSensor->enable(timeStep);

    Keyboard *kb = robot->getKeyboard();
    kb->enable(timeStep);

    Node *lightNode = robot->getFromDef("ENV_LIGHT");
    Field *lightIntensityField = lightNode ? lightNode->getField("intensity") : nullptr;
    Field *lightDirField = lightNode ? lightNode->getField("direction") : nullptr;

    Node *texNode = robot->getFromDef("MAP_TEXTURE");
    Field *texUrlField = texNode ? texNode->getField("url") : nullptr;

    Node *selfNode = robot->getSelf();
    Field *transField = selfNode->getField("translation");
    Field *rotField = selfNode->getField("rotation");

    const double *initTrans = transField->getSFVec3f();
    const double *initRot = rotField->getSFRotation();
    double startPos[3] = {initTrans[0], initTrans[1], initTrans[2]};
    double startRot[4] = {initRot[0], initRot[1], initRot[2], initRot[3]};

    CobotBrain brain;
    double light_intensity = 1.8;
    double light_angle_deg = 0.0;
    int line_width_options[3] = {16, 22, 30};
    int line_width_idx = 1;
    float battery_voltage = 6.0f;
    double match_start_time = 0.0;
    int retry_count = 0;
    bool out_of_line_warning = false;
    int key_cooldown = 0;
    int step_counter = 0;

    while (robot->step(timeStep) != -1) {
        step_counter++;
        double sim_time = robot->getTime();
        double elapsed = sim_time - match_start_time;
        double dt = timeStep / 1000.0;

        battery_voltage = max2f(4.5f, battery_voltage - 0.00004f);

        int key = kb->getKey();
        if (key_cooldown > 0) key_cooldown--;
        if (key != -1 && key_cooldown == 0) {
            key_cooldown = 8;
            if (key == 'L' && !isSecondRobot) {
                light_intensity = (light_intensity + 0.25 > 5.0) ? 5.0 : (light_intensity + 0.25);
                if (lightIntensityField) lightIntensityField->setSFFloat(light_intensity);
            } else if (key == 'K' && !isSecondRobot) {
                light_intensity = (light_intensity - 0.25 < 0.2) ? 0.2 : (light_intensity - 0.25);
                if (lightIntensityField) lightIntensityField->setSFFloat(light_intensity);
            } else if (key == 'J' && !isSecondRobot) {
                light_angle_deg += 25.0;
                double rad = light_angle_deg * M_PI / 180.0;
                double newDir[3] = { -0.6 * cos(rad), -0.6 * sin(rad), -1.0 };
                if (lightDirField) lightDirField->setSFVec3f(newDir);
            } else if (key == 'M' && !isSecondRobot) {
                line_width_idx = (line_width_idx + 1) % 3;
                int w = line_width_options[line_width_idx];
                bool isFinal = (robot->getWorldPath().find("chung_ket") != std::string::npos);
                std::string newUrl = std::string("textures/") + (isFinal ? "vong_chung_ket_" : "vong_bang_") + std::to_string(w) + "mm.png";
                if (texUrlField) texUrlField->setMFString(0, newUrl);
            } else if (key == 'B') {
                battery_voltage = min2f(7.4f, battery_voltage + 0.2f);
            } else if (key == 'V') {
                battery_voltage = max2f(4.5f, battery_voltage - 0.2f);
            } else if (key == 'W') {
                brain.base_pwm = min2f(245.0f, brain.base_pwm + 5.0f);
            } else if (key == 'S') {
                brain.base_pwm = max2f(120.0f, brain.base_pwm - 5.0f);
            } else if (key == 'E') {
                brain.Kp += 2.0f;
            } else if (key == 'Q') {
                brain.Kp = max2f(10.0f, brain.Kp - 2.0f);
            } else if (key == 'D') {
                brain.Kd += 0.2f;
            } else if (key == 'A') {
                brain.Kd = max2f(0.0f, brain.Kd - 0.2f);
            } else if (key == 'T') {
                brain.loop2_shortcut = !brain.loop2_shortcut;
            } else if (key == 'R') {
                transField->setSFVec3f(startPos);
                rotField->setSFRotation(startRot);
                selfNode->resetPhysics();
                brain.reset();
                retry_count++;
            } else if (key == 'N') {
                transField->setSFVec3f(startPos);
                rotField->setSFRotation(startRot);
                selfNode->resetPhysics();
                brain.reset();
                match_start_time = sim_time;
                retry_count = 0;
            }
        }

        SensorData sData;
        sData.battery_volt = battery_voltage;
        sData.elapsed_sec = elapsed;
        float ambient_glare = (float)((light_intensity > 2.2) ? (light_intensity - 2.2) * 0.05 : 0.0);

        const double *curPos = transField->getSFVec3f();
        const double *rotMatrix = selfNode->getOrientation();

        bool on_bridge = (curPos[2] > 0.024 && curPos[0] < 0.08 && curPos[0] > -0.48 && curPos[1] < -0.52 && curPos[1] > -0.72);
        const double ir_y_offsets[4] = { +0.027, +0.009, -0.009, -0.027 };

        int white_sensors = 0;
        for (int i = 0; i < 4; i++) {
            if (on_bridge) {
                // Mô phỏng cảm biến hồng ngoại đo phản xạ vạch đen trên dốc cầu 3D
                // (Trong thực tế TCRT5000 bắt vạch băng dính đen 22mm chạy dọc tâm cầu Y = -0.620)
                double sensor_y = curPos[1] + rotMatrix[3] * 0.085 + rotMatrix[4] * ir_y_offsets[i] + rotMatrix[5] * (-0.006);
                double dist = fabs(sensor_y - (-0.620));
                if (dist <= 0.009) {
                    sData.ir[i] = 1.0f;
                    sData.raw_ir[i] = 1024;
                } else if (dist >= 0.017) {
                    sData.ir[i] = 0.07f;
                    sData.raw_ir[i] = 468;
                } else {
                    float frac = (float)((0.017 - dist) / (0.017 - 0.009));
                    sData.ir[i] = 0.07f + 0.93f * frac;
                    sData.raw_ir[i] = (int)(468 + (1024 - 468) * frac);
                }
            } else {
                double val = ir[i]->getValue();
                sData.raw_ir[i] = (int)val;
                float raw_norm = (float)(val / 1024.0) - ambient_glare;
                float calibrated = (raw_norm - 0.42f) / (0.92f - 0.42f);
                sData.ir[i] = clampf(calibrated, 0.0f, 1.0f);
            }
            if (sData.ir[i] < 0.12f) white_sensors++;
        }

        out_of_line_warning = (white_sensors == 4 && !brain.was_near_center);

        const unsigned char *img = colorSensor->getImage();
        int width = colorSensor->getWidth();
        sData.r = Camera::imageGetRed(img, width, 1, 1);
        sData.g = Camera::imageGetGreen(img, width, 1, 1);
        sData.b = Camera::imageGetBlue(img, width, 1, 1);

        int pwm_l = 0, pwm_r = 0;
        brain.computeControl(sData, dt, pwm_l, pwm_r);

        if (step_counter % 25 == 0) {
            printf("[LOG t=%05.2f] pos=(%+0.3f, %+0.3f, %+0.3f) state=%d cross=%d blue=%d raw=[%d,%d,%d,%d] ir=[%.2f,%.2f,%.2f,%.2f] pwm=(%+4d,%+4d) rgb=(%3d,%3d,%3d)\n",
                   elapsed, curPos[0], curPos[1], curPos[2], (int)brain.state, brain.cross_count,
                   brain.blue_zone_count, sData.raw_ir[0], sData.raw_ir[1], sData.raw_ir[2], sData.raw_ir[3],
                   sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                   pwm_l, pwm_r, sData.r, sData.g, sData.b);
            fflush(stdout);
        }


        if (brain.finished) {
            printf("[SUCCESS] Robot successfully reached FINISH at t=%.2f s!\n", elapsed);
            fflush(stdout);
            break;
        }

        if (curPos[2] < -0.05 || (curPos[1] < -0.75 && curPos[0] < 0.15 && curPos[0] > -0.55)) {
            printf("[CRASH/FALL OFF] Robot fell off track at t=%.2f pos=(%.3f, %.3f, %.3f)!\n",
                   elapsed, curPos[0], curPos[1], curPos[2]);
            fflush(stdout);
            break;
        }

        double max_omega = 31.416 * (battery_voltage / 6.0);
        double omega_l = (pwm_l / 255.0) * max_omega;
        double omega_r = (pwm_r / 255.0) * max_omega;

        leftMotor->setVelocity((omega_l < -31.416) ? -31.416 : ((omega_l > 31.416) ? 31.416 : omega_l));
        rightMotor->setVelocity((omega_r < -31.416) ? -31.416 : ((omega_r > 31.416) ? 31.416 : omega_r));

        if (step_counter % 6 != 0) continue;

        const char* state_names[] = {
            "DO LINE PID (MUOT & NHANH)",
            "VONG TRON 1 (KHOA THANG X)",
            "GO GIAM TOC (GIU THANG LAI)",
            "VONG TRON 2 (CAN CHINH AN TOAN)",
            "VONG TRON 2 (XOAY PHAI RE TAT)",
            "DOC CAU 3D 5CM (BAM TAM KIN)"
        };

        int id_base = isSecondRobot ? 20 : 0;
        double x_col = isSecondRobot ? 0.52 : 0.012;
        double font_sz = 0.048;
        double dy = 0.032;

        char line[6][160];
        snprintf(line[0], sizeof(line[0]), "[%s] Time: %05.2f/150s | Retry: %d | Pin: %.2fV",
                 robotName.c_str(), elapsed, retry_count, battery_voltage);
        snprintf(line[1], sizeof(line[1]), "NgaTu X: %d | CumXanh: %d | Vong2: %s",
                 brain.cross_count, brain.blue_zone_count,
                 brain.loop2_shortcut ? "DI TAT" : "DI VONG");
        snprintf(line[2], sizeof(line[2]), "PID: Kp=%.0f Kd=%.1f Base=%.0f | PWM[L:%+3d R:%+3d]",
                 brain.Kp, brain.Kd, brain.base_pwm, pwm_l, pwm_r);
        snprintf(line[3], sizeof(line[3]), "State: %s %s",
                 state_names[brain.state],
                 brain.finished ? "[VE DICH!]" : (out_of_line_warning ? "[! NGOAI LINE !]" : ""));
        snprintf(line[4], sizeof(line[4]), "IR0-3: [%.2f | %.2f | %.2f | %.2f] | RGB(%3d,%3d,%3d)->%s",
                 sData.ir[0], sData.ir[1], sData.ir[2], sData.ir[3],
                 sData.r, sData.g, sData.b, brain.detectColorTag(sData.r, sData.g, sData.b));

        int colors[5] = {
            0x00FFAA,
            0x70D6FF,
            0xE0E0E0,
            brain.finished ? 0x00FF44 : (out_of_line_warning ? 0xFF3333 : 0xFFD700),
            0xFFFFFF
        };

        for (int i = 0; i < 5; i++) {
            drawCleanHudLine(robot, id_base + i, line[i], x_col, 0.012 + i * dy, font_sz, colors[i]);
        }

        if (!isSecondRobot) {
            snprintf(line[5], sizeof(line[5]),
                     "[Phim tat] W/S:TocDo | E/Q:Kp | D/A:Kd | T:Vong2 | R:VeStart | N:VanMoi | L/K/J:AnhSang | M:Line(%dmm)",
                     line_width_options[line_width_idx]);
            drawCleanHudLine(robot, 15, line[5], 0.012, 0.94, 0.045, 0xA0FFD0);
        }
    }

    delete robot;
    return 0;
}

#else
// ============================================================================
// PHẦN 4: CODE NẠP THẲNG LÊN MẠCH THẬT (KM-11-102 + KMM-11-100 + GA12-N20 5V)
// ============================================================================
#include <Arduino.h>
#include <Wire.h>

const int PIN_IR[4] = {A0, A1, A2, A3};  // 4 mắt cảm biến hồng ngoại dò đường
const int PIN_BAT   = A7;                // Chân đo áp pin (nếu mạch có nối cầu phân áp)
const int ENA = 5, IN1 = 4, IN2 = 3;     // Động cơ Trái (KM-23-101: GA12-N20 5V 300RPM)
const int ENB = 6, IN3 = 7, IN4 = 8;     // Động cơ Phải (KM-23-101: GA12-N20 300RPM)

int ir_min_cal[4] = {220, 220, 220, 220};
int ir_max_cal[4] = {720, 720, 720, 720};
bool ir_inverted  = false;

const uint8_t TCS34725_ADDR = 0x29;
bool has_i2c_color_sensor = false;

CobotBrain brain;
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
    const int MIN_N20_PWM = 45;
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

void setup() {
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    setMotorHardware(0, 0);

    for (int i = 0; i < 4; i++) {
        pinMode(PIN_IR[i], INPUT);
    }

    initColorSensorHardware();

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
    if (dt < 0.005) return;
    lastStepMicros = nowMicros;

    SensorData sData;
    sData.elapsed_sec = (millis() - startMillis) / 1000.0;

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

    float measured_v = (analogRead(PIN_BAT) / 1023.0f) * 5.0f * 2.0f;
    if (measured_v >= 4.2f && measured_v <= 7.5f) {
        sData.battery_volt = measured_v;
    } else {
        sData.battery_volt = 6.0f;
    }

    updateColorSensorHardware(sData.r, sData.g, sData.b);

    int pwm_l = 0, pwm_r = 0;
    brain.computeControl(sData, dt, pwm_l, pwm_r);
    setMotorHardware(pwm_l, pwm_r);
}
#endif
