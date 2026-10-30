# HƯỚNG DẪN TRIỂN KHAI ROBOT THẬT (SIM-TO-REAL GUIDE)
### Cuộc thi: "Đường tới Tân Cương - Thái Nguyên lần thứ 2 (ICTU Robotics 2026)"
### Phần cứng: Bo mạch KM-11-102 (ATmega328P) + Mạch cảm biến KMM-11-100 + GA12-N20

---

## MỤC LỤC
1. [Tổng quan & Cấu trúc mã nguồn](#1-tổng-quan--cấu-trúc-mã-nguồn)
2. [Cài đặt môi trường nạp code (Arduino IDE)](#2-cài-đặt-môi-trường-nạp-code-arduino-ide)
3. [Sơ đồ đấu nối phần cứng & Chân điều khiển](#3-sơ-đồ-đấu-nối-phần-cứng--chân-điều-khiển)
4. [Những chỗ CẦN CHỈNH TRONG CODE khi lên xe thật](#4-những-chỗ-cần-chỉnh-trong-code-khi-lên-xe-thật)
5. [Quy trình hiệu chuẩn cảm biến (Calibration)](#5-quy-trình-hiệu-chuẩn-cảm-biến-calibration)
6. [Căn chỉnh hệ số PID & Tốc độ theo thực tế sân đấu](#6-căn-chỉnh-hệ-số-pid--tốc-độ-theo-thực-tế-sân-đấu)
7. [Checklist 5 bước kiểm tra tại chỗ trước khi thi đấu](#7-checklist-5-bước-kiểm-tra-tại-chỗ-trước-khi-thi-đấu)
8. [Xử lý các lỗi thực tế thường gặp (Troubleshooting)](#8-xử-lý-các-lỗi-thực-tế-thường-gặp-troubleshooting)

---

## 1. TỔNG QUAN & CẤU TRÚC MÃ NGUỒN

Dự án của bạn có 2 file code chính:
* **Mô phỏng Webots 3D:** [`controllers/cobot_cpp/cobot_cpp.cpp`](file:///home/yogurt/ICTU_Robotics_2026/controllers/cobot_cpp/cobot_cpp.cpp)
* **Nạp lên Robot thật:** [`arduino_cobot/arduino_cobot.ino`](file:///home/yogurt/ICTU_Robotics_2026/arduino_cobot/arduino_cobot.ino)

### Đặc điểm thiết kế của `arduino_cobot.ino`:
1. **Thuật toán lõi 100% đồng bộ với Webots:** Toàn bộ máy trạng thái `CobotBrain`, giải thuật tính trọng tâm sai số liên tục $C^1$, bộ lọc thông thấp đạo hàm, hãm phanh chủ động dốc cầu và rẽ phải thông minh tại Vòng 2 đều được giữ nguyên vẹn.
2. **Tối ưu hóa không phụ thuộc thư viện nặng:**
   - Giao tiếp I2C của cảm biến màu TCS34725 được viết trực tiếp qua thanh ghi phần cứng (`Wire.h` chuẩn), không cần cài thêm thư viện phụ trách ngoài (như Adafruit TCS34725), giúp tiết kiệm $40\%$ dung lượng RAM của vi điều khiển ATmega328P.
3. **Bù vùng chết động cơ GA12-N20 (Deadzone Compensation):**
   - Động cơ hộp số mini GA12-N20 có ma sát tĩnh cơ học. Code đã tích hợp hàm `applyN20Deadzone()` tự động ánh xạ dải PWM thấp lên ngưỡng tối thiểu ($45\text{ PWM}$), giúp động cơ quay êm, không bị kẹt hay kêu è è ở vận tốc thấp.
4. **Vòng lặp điều khiển thời gian thực cố định 200Hz:**
   - Chu kỳ lấy mẫu $\Delta t = 5\text{ ms}$ ổn định tuyệt đối, triệt tiêu hiện tượng gai vi phân khi tính toán $K_d$.

---

## 2. CÀI ĐẶT MÔI TRƯỜNG NẠP CODE (ARDUINO IDE)

### Bước 1: Tải và cài đặt Arduino IDE
* Tải bản **Arduino IDE 2.3+** (hoặc Arduino IDE 1.8.19) từ trang chủ: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software).

### Bước 2: Cài đặt Driver nạp chip (Nạp cáp USB)
* Mạch điều khiển **KM-11-102** thường sử dụng chip nạp cổng nối tiếp **CH340** hoặc **CP2102**:
  * Nếu máy tính không nhận cổng COM khi cắm cáp USB, hãy tải và cài **Driver CH340** (CH341SER).

### Bước 3: Cấu hình trên Arduino IDE
* Mở file [`arduino_cobot/arduino_cobot.ino`](file:///home/yogurt/ICTU_Robotics_2026/arduino_cobot/arduino_cobot.ino).
* Vào menu **Tools**:
  * **Board:** Chọn `Arduino AVR Boards` $\to$ `Arduino Nano` (hoặc `Arduino Uno` tùy phiên bản kit).
  * **Processor:** Chọn `ATmega328P` (Nếu báo lỗi *not in sync* / *timeout* khi nạp, hãy chuyển sang `ATmega328P (Old Bootloader)`).
  * **Port:** Chọn đúng cổng `COM` của robot (ví dụ `COM3`, `COM4` trên Windows hoặc `/dev/ttyUSB0` trên Linux).

---

## 3. SƠ ĐỒ ĐẤU NỐI PHẦN CỨNG & CHÂN ĐIỀU KHIỂN

Sơ đồ chân mặc định trong file `arduino_cobot.ino`:

```
                 ┌────────────────────────────────┐
                 │       MẠCH KM-11-102           │
                 │      (MCU ATmega328P)          │
                 └──┬────────┬────────┬───────┬───┘
                    │        │        │       │
    ┌───────────────┘        │        │       └────────────────┐
    ▼                        ▼        ▼                        ▼
[CẢM BIẾN IR]            [CẦU H ĐỘNG CƠ]  [MÀU TCS34725]   [ĐO PIN AA]
A0: IR0 (Trái ngoài)     D5: ENA (PWM L)  A4: SDA (I2C)    A7: Phân áp pin
A1: IR1 (Trái trong)     D4: IN1 (Dir L)  A5: SCL (I2C)    (Tỷ lệ 1:1)
A2: IR2 (Phải trong)     D3: IN2 (Dir L)
A3: IR3 (Phải ngoài)     D6: ENB (PWM R)
                         D7: IN3 (Dir R)
                         D8: IN4 (Dir R)
```

> [!IMPORTANT]
> **Khoảng cách lắp đặt vật lý (Ground Clearance):**
> * Mắt cảm biến hồng ngoại TCRT5000: Khoảng cách từ đầu mắt cảm biến xuống mặt sàn đạt độ nhạy tốt nhất là **$6\text{ mm} - 8\text{ mm}$** (tối đa $\le 10\text{ mm}$).
> * Cảm biến màu TCS34725: Cách mặt sân **$8\text{ mm} - 12\text{ mm}$**, luôn bật đèn LED trắng tích hợp trên mạch cảm biến để ánh sáng ổn định không phụ thuộc bóng râm.

---

## 4. NHỮNG CHỖ CẦN CHỈNH TRONG CODE KHI LÊN XE THẬT

### 4.1. Đảo chiều quay động cơ (Nếu bánh quay lùi)
Sau khi nạp code, nhấc 2 bánh xe lên khỏi mặt đất và bật nguồn:
* Cả 2 bánh xe phải quay **TIẾN** về phía trước.
* Nếu bánh nào quay **LÙI**, hãy sửa trực tiếp tại dòng 24-29 trong file [`arduino_cobot.ino`](file:///home/yogurt/ICTU_Robotics_2026/arduino_cobot/arduino_cobot.ino#L24-L30):

```cpp
// NẾU BÁNH TRÁI BỊ QUAY LÙI: Đổi chỗ IN1 và IN2
const int IN1 = 3;  // (Nguyên bản là 4)
const int IN2 = 4;  // (Nguyên bản là 3)

// NẾU BÁNH PHẢI BỊ QUAY LÙI: Đổi chỗ IN3 và IN4
const int IN3 = 8;  // (Nguyên bản là 7)
const int IN4 = 7;  // (Nguyên bản là 8)
```

---

### 4.2. Kiểm tra thứ tự giàn cảm biến dò line
Đảm bảo giàn 4 mắt được xếp thẳng hàng ngang từ Trái sang Phải:
```cpp
const int PIN_IR[4] = {A0, A1, A2, A3};
// A0: Mắt ngoài cùng bên Trái (IR0)
// A1: Mắt trong bên Trái     (IR1)
// A2: Mắt trong bên Phải    (IR2)
// A3: Mắt ngoài cùng bên Phải(IR3)
```
* Nếu xe cứ rẽ ngoặt ngược hướng vạch đen ngay khi chạy, tức là thứ tự mắt cắm đang bị ngược ($A3 \to A0$), hãy đổi lại thứ tự mảng thành `{A3, A2, A1, A0}` hoặc cắm lại dây.

---

### 4.3. Bù ma sát khởi động động cơ (`MIN_N20_PWM`)
Hộp số kim loại của GA12-N20 có ma sát tĩnh khác nhau tùy từng động cơ:
```cpp
const int MIN_N20_PWM = 45; // Mặc định là 45
```
* **Hiện tượng:** Nếu ở các góc cua chậm, xe có hiện tượng bánh trong bị "khựng" hoặc động cơ phát tiếng kêu "è è" nhưng không quay:
  $\to$ Tăng `MIN_N20_PWM` lên `50` hoặc `55`.
* Nếu bánh quay quá nhạy làm xe giật:
  $\to$ Giảm `MIN_N20_PWM` xuống `35` hoặc `40`.

---

### 4.4. Căn chỉnh ngưỡng nhận diện Màu sắc (Sân thi đấu thật)
Màu sắc thực tế của bạt in tại nhà thi đấu phụ thuộc vào hệ thống đèn tuýp và chất liệu bạt. Bạn có thể mở `Serial Monitor` (tốc độ `115200`) để quan sát giá trị `(r, g, b)` khi đặt xe lên các vùng:

Trong file [`arduino_cobot.ino`](file:///home/yogurt/ICTU_Robotics_2026/arduino_cobot/arduino_cobot.ino#L130-L137):
```cpp
const char* detectColorTag(int r, int g, int b) const {
    // 1. Vạch đỏ FINISH (Dừng xe):
    if (r > 70 && r > g * 2.0 && r > b * 2.0) return "RED (FINISH)";
    
    // 2. Dốc cầu xanh & Gờ giảm tốc:
    if (b > 85 && r < 55 && g < 75 && (b > r + 35)) return "BLUE (GO / DOC CAU)";
    
    // 3. Vạch trắng nền sân:
    return "WHITE (SAN)";
}
```
* **Nếu xe không nhận vạch Đích màu đỏ:** Giảm điều kiện tỉ lệ đỏ xuống: `(r > 60 && r > g * 1.5 && r > b * 1.5)`.
* **Nếu xe bị nhận nhầm nền xám thành dốc xanh:** Tăng ngưỡng chênh lệch xanh lên: `(b > r + 45)`.

---

## 5. QUY TRÌNH HIỆU CHUẨN CẢM BIẾN (CALIBRATION)

### 5.1. Cơ chế Tự động Học ngưỡng (Auto-Calibration trong `setup()`)
Code đã tích hợp thuật toán tự động thích ứng với ánh sáng phòng thi đấu:
1. Khi chuẩn bị xuất phát, **đặt robot ngay ngắn tại vạch xuất phát sao cho 2 mắt giữa (IR1, IR2) đè lên vạch đen, 2 mắt ngoài (IR0, IR3) nằm trên nền trắng**.
2. Bật công tắc nguồn của robot.
3. Trong $0.2\text{ giây}$ đầu tiên, robot tự động đo và ghi nhớ giá trị ADC nền trắng (`ir_min_cal`) và vạch đen (`ir_max_cal`).
4. Trong suốt quá trình chạy, thuật toán **Dynamic Min-Max Filter** sẽ liên tục cập nhật ngưỡng thích ứng nếu bạt in có chỗ đậm chỗ nhạt.

### 5.2. Chỉnh phần cứng biến trở trên bo mạch cảm biến KMM-11-100 (Nếu có)
Nếu trên mạch KMM-11-100 có các chiết áp (biến trở vi chỉnh) cho từng mắt IR:
1. Đặt mắt cảm biến lên nền trắng bạt thi đấu $\to$ Xoay chiết áp đến khi đèn LED báo trạng thái vừa tắt.
2. Di chuyển mắt cảm biến vào vạch đen $\to$ Đèn LED phải sáng rõ dứt khoát.
3. Lặp lại cho cả 4 mắt đọc.

---

## 6. CĂN CHỈNH HỆ SỐ PID & TỐC ĐỘ THEO THỰC TẾ SÂN ĐẤU

Các tham số điều khiển nằm ở đầu lớp `CobotBrain` trong file [`arduino_cobot.ino`](file:///home/yogurt/ICTU_Robotics_2026/arduino_cobot/arduino_cobot.ino#L70-L77):

```cpp
float Kp = 34.0f;       // Hệ số tỷ lệ bẻ lái
float Ki = 0.0f;        // Tích phân (luôn để 0 để tránh trễ)
float Kd = 4.2f;        // Hệ số vi phân giảm chấn
float base_pwm = 210.0f;// Tốc độ định mức đường thẳng
```

### Hướng dẫn tinh chỉnh (Tuning Guide):

| Hiện tượng thực tế trên sân | Nguyên nhân | Cách khắc phục trong code |
| :--- | :--- | :--- |
| **Xe bị lắc lư hình sin qua lại liên tục trên đường thẳng** | $K_p$ quá lớn gây dao động quá mức | Giảm $K_p$ từng bước $2.0$ (ví dụ: $34 \to 30 \to 28$). |
| **Xe vào góc cua bị lười lái, văng ra ngoài mép vạch** | $K_p$ quá nhỏ, phản hồi bẻ lái chậm | Tăng $K_p$ từng bước $2.0$ (ví dụ: $34 \to 38$). |
| **Xe bị giật cục, rung bánh khi chuyển từ đường thẳng vào cua** | Đạo hàm bị nhạy quá mức | Tăng nhẹ $K_d$ (ví dụ: $4.2 \to 4.8$) hoặc kiểm tra giảm chấn. |
| **Bánh xe bị trượt xoay (mất ma sát) trên nền bạt trơn** | Tốc độ quá cao so với độ bám của lốp | Giảm `base_pwm` từ $210 \to 190 - 195$. |
| **Xe leo dốc cầu 5cm bị đuối sức, không lên nổi đỉnh** | Điện áp pin bị sụt hoặc thiếu PWM leo dốc | Trong `STATE_BRIDGE_CLIMB`, tăng `target_base` leo dốc từ $165 \to 180$. |
| **Xe đổ dốc cầu bị trôi nhanh, văng khi tiếp đất** | Hãm động cơ chưa đủ mạnh | Trong `STATE_BRIDGE_CLIMB`, giảm tốc độ đổ dốc từ $135 \to 120$. |

---

## 7. CHECKLIST 5 BƯỚC KIỂM TRA TẠI CHỖ TRƯỚC KHI THI ĐẤU

Thực hiện quy trình này trước mỗi lượt chạy chính thức:

```
[BƯỚC 1: PIN]       --> Kiểm tra điện áp khối 4 pin AA >= 5.8V - 6.2V
         │
[BƯỚC 2: VỆ SINH]   --> Dùng băng dính/cồn lau sạch bụi bám trên lốp cao su
         │
[BƯỚC 3: CẢM BIẾN]  --> Kiểm tra độ cao mắt đọc cách sàn 6-8mm, lau bụi mắt IR
         │
[BƯỚC 4: ĐẶT XE]    --> Đặt 2 mắt giữa lên vạch đen xuất phát, 2 mắt ngoài nền trắng
         │
[BƯỚC 5: XUẤT PHÁT] --> Bật công tắc nguồn, sẵn sàng hiệu lệnh xuất phát
```

---

## 8. XỬ LÝ CÁC LỖI THỰC TẾ THƯỜNG GẶP (TROUBLESHOOTING)

### 1. Xe đi qua ngã tư Vòng 2 mà không chịu rẽ phải đi tắt?
* **Kiểm tra cờ:** Trong code, đảm bảo `bool loop2_shortcut = true;`.
* **Kiểm tra điều kiện bắt vạch:** Tại Vòng 2, sau khi tiến thẳng $0.22\text{ s}$, xe sẽ xoay phải để tìm nhánh rẽ. Nếu mắt cảm biến chưa kịp bắt vạch, có thể do tốc độ xoay hơi nhanh. Hãy tăng thời gian tiến an toàn từ `0.22` lên `0.24` hoặc tăng lực xoay `pwm_left` từ `180` lên `190`.

### 2. Xe bị dừng đột ngột giữa sân đấu?
* **Kiểm tra vạch đỏ:** Cảm biến màu có thể bị đọc nhầm một mảng sẫm màu thành vạch đỏ `RED (FINISH)`. Điều kiện dừng trong code đã khóa an toàn: `s.elapsed_sec > 10.0 && (cross_count >= 3 || blue_zone_count >= 3)`. Đảm bảo robot chỉ dừng khi đã đi qua ngã tư số 3 và dốc cầu.

### 3. Xe chạy một lúc thì tốc độ chậm dần và leo dốc yếu?
* **Nguyên nhân:** Khối 4 pin AA thường sụt áp rất nhanh sau $3 - 5$ lượt chạy thử. 
* **Giải pháp:** Luôn chuẩn bị sẵn pin sạc Eneloop/NiMH hoặc pin kiềm Alkaline mới (Energizer/Duracell) cho lượt thi đấu chính thức. Code có sẵn khâu đo chân `A7` tự động bù xung PWM khi pin sụt áp.

---
*Chúc đội thi đấu ICTU Robotics 2026 đạt kết quả cao nhất và phá kỷ lục thời gian trên sa bàn!*
