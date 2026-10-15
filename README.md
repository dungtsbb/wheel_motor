# wheel_motor — điều khiển cụm động cơ bánh xe có encoder bằng ESP32

Chương trình mẫu ESP-IDF cho ESP32 + driver TB6612FNG: điều khiển chiều và tốc
độ, đọc encoder bằng PCNT, vòng điều khiển vận tốc PI, và công cụ hiệu chuẩn.

Bài hướng dẫn đầy đủ: https://vinexus.dev/blog/dieu-khien-dong-co-banh-xe-esp32/

## Bắt đầu nhanh

```bash
git clone https://github.com/dungtsbb/wheel_motor.git
cd wheel_motor
idf.py set-target esp32
idf.py -p /dev/ttyUSB0 flash monitor
```

Nối dây theo mục 3, sau đó gõ `h` trong `monitor` để xem bảng lệnh.

---

## 1. Thông số cụm động cơ

### 1.1 Thông số cơ khí

| Thông số | Giá trị | Hằng số trong code |
|---|---|---|
| Đường kính bánh | 60 mm (bán kính 30 mm) | `WHEEL_RADIUS_MM` |
| Xung encoder mỗi vòng **trục motor** | 20 | `ENC_PPR_MOTOR` |
| Tỉ số hộp giảm tốc | 4123/88 ≈ 46.85 : 1 | `GEAR_RATIO` |
| Duty tối thiểu để bánh quay | 28 % | `MIN_DUTY` |

Encoder gắn ở **trục motor**, trước hộp giảm tốc. Nó quay nhanh gấp 46.85 lần
bánh, nên dù chỉ 20 xung mỗi vòng vẫn cho độ phân giải 0.2 mm ở bánh.

### 1.2 Giá trị suy ra

| Đại lượng | Công thức | Giá trị |
|---|---|---|
| Xung mỗi vòng **bánh** | `ENC_PPR_MOTOR × GEAR_RATIO` | 937 xung |
| Chu vi bánh | `2πR` | 188.5 mm |
| Quãng đường mỗi xung | `2πR / 937` | **0.2012 mm** |
| Độ phân giải vận tốc (chu kỳ 20 ms) | `0.2012 mm / 20 ms` | 0.01 m/s |
| Tần số xung ở 0.5 m/s | — | 2 490 Hz |

`0.2012 mm/xung` là hằng số dùng nhiều nhất: mọi phép đổi xung ↔ quãng đường ↔
vận tốc đều đi qua nó. Code tính sẵn ở macro `MM_PER_PULSE`.

### 1.3 Giới hạn đặt trong chương trình

| | Giá trị | Hằng số |
|---|---|---|
| Vận tốc dài tối đa | 0.50 m/s | `MAX_SPEED_MPS` |
| Tương đương tốc độ bánh | 159 vòng/phút | — |
| Tương đương tốc độ trục motor | 7 460 vòng/phút | — |

### 1.4 Thông số driver TB6612FNG

| | Giá trị |
|---|---|
| Điện áp động cơ (VM) | 2.5 – 13.5 V |
| Điện áp logic (VCC) | 2.7 – 5.5 V |
| Dòng ra mỗi kênh, liên tục | 1.2 A |
| Dòng ra mỗi kênh, đỉnh | 3.2 A |
| Tần số PWM | tới 100 kHz |

### 1.5 Thông số điện của cụm động cơ

Ba thông số điện sau cần đo trước khi chọn nguồn:

| Thông số | Cách đo |
|---|---|
| Điện áp làm việc | Tăng dần từ 6 V, chọn mức cho tốc độ đủ dùng. Thường 12 V hoặc 14.4 V |
| Dòng không tải | Đồng hồ nối tiếp VM, chạy không tải ở duty 100 % |
| Dòng kẹt (stall) | Giữ chặt bánh, cấp duty 20–30 % trong **dưới 1 giây**, đọc dòng |

Dòng kẹt phải nằm dưới 1.2 A để dùng được TB6612. Nếu vượt, chọn driver dòng cao
hơn (DRV8871, BTS7960) — phần code điều khiển giữ nguyên, chỉ đổi cách nối chân.

---

## 2. Danh sách linh kiện

| Món | Số lượng | Ghi chú |
|---|---|---|
| Cụm động cơ bánh xe | 1 | Gồm motor + hộp số + encoder + bánh |
| ESP32 DevKit | 1 | Loại WROOM |
| Driver TB6612FNG | 1 | Dùng kênh A |
| Tụ hoá 100–470 µF | 1 | Lắp ở chân VM |
| Tụ gốm 0.1 µF | 2 | Một ở VM, một ở hai cực motor |
| Trở 10 kΩ + 20 kΩ | 1 bộ | Chỉ cần nếu encoder dùng nguồn 5 V |
| Nguồn cho động cơ | 1 | Theo điện áp đo được ở mục 1.5 |

---

## 3. Sơ đồ nối dây

### 3.1 ESP32 ↔ TB6612FNG

| TB6612FNG | ESP32 | Chức năng |
|---|---|---|
| **VM** | Nguồn động cơ | Đường dòng lớn, dùng dây to |
| **VCC** | **3.3 V** | Nguồn logic |
| **GND** | GND chung | |
| **STBY** | **GPIO 12** | HIGH = driver hoạt động |
| **PWMA** | **GPIO 25** | Tín hiệu PWM |
| **AIN1** | **GPIO 26** | Chiều quay |
| **AIN2** | **GPIO 27** | Chiều quay |
| **AO1 / AO2** | 2 dây động cơ | Đảo 2 dây = đảo chiều quay |

### 3.2 ESP32 ↔ Encoder

| Dây encoder | ESP32 | Ghi chú |
|---|---|---|
| **OUT** | **GPIO 18** | Pull-up nội bật sẵn trong code |
| **VCC** | 3.3 V | Nếu encoder chịu được 3.3 V |
| **GND** | GND chung | |

**Nếu encoder cần nguồn 5 V**, tín hiệu ra cũng ở mức 5 V và phải qua cầu phân
áp trước khi vào GPIO:

```
OUT encoder ──[ 10kΩ ]──┬── GPIO 18
                        │
                     [ 20kΩ ]
                        │
                       GND
```

5 V × 20/(10+20) = 3.3 V.

### 3.3 Bốn điểm cần làm đúng

**GPIO 12 là strapping pin.** Nối thẳng vào STBY thì hoạt động bình thường, nhưng
không được mắc trở kéo lên 3.3 V ở chân này. Nếu board không boot, chuyển STBY
sang GPIO 13 và sửa `PIN_STBY` trong code.

**Tụ lọc ở VM.** Lắp tụ hoá 100–470 µF song song tụ gốm 0.1 µF, đặt sát chân VM
của TB6612. Động cơ chổi than đảo chiều tạo xung áp đủ mạnh làm ESP32 khởi động lại.

**Tụ gốm ở động cơ.** Hàn tụ 0.1 µF ngay hai cực motor để giảm nhiễu chổi than
lên đường tín hiệu encoder.

**Mass hình sao.** ESP32, TB6612 và nguồn động cơ nối GND về cùng một điểm, không
nối dây chuyền.

---

## 4. Nạp chương trình mẫu

```bash
# 1. Kích hoạt môi trường ESP-IDF
source ~/esp/esp-idf/export.sh

# 2. Vào thư mục chương trình mẫu
cd wheel_motor

# 3. Chọn chip và build
idf.py set-target esp32
idf.py build

# 4. Nạp và mở màn hình serial
idf.py -p /dev/ttyUSB0 flash monitor
```

Thoát `monitor` bằng `Ctrl+]`.

Kẹp chặt cụm động cơ vào bàn hoặc kê sao cho bánh quay tự do trước khi gõ lệnh.

Sau khi nạp xong, màn hình hiện:

```
=== Điều khiển cụm động cơ bánh xe ===
Động cơ: PWM=GPIO25 IN1=GPIO26 IN2=GPIO27 STBY=GPIO12
Encoder: GPIO18
Bánh R=30.0mm, hộp số 46.85:1, 937 xung/vòng bánh, 0.2012 mm/xung
PWM 20000Hz 10-bit, vòng điều khiển 50Hz, Kp=0.8 Ki=2.0
```

---

## 5. Bảng lệnh

Gõ trực tiếp trong `idf.py monitor`, không cần nhấn Enter.

| Phím | Chức năng |
|---|---|
| `w` | Tăng vận tốc đích 0.05 m/s |
| `s` | Giảm vận tốc đích 0.05 m/s (âm = quay ngược) |
| `space` | Dừng (thả trôi) |
| `x` | Phanh gấp |
| `p` | Bật/tắt vòng điều khiển PI |
| `e` | In encoder: xung, mm, m/s, số vòng bánh, duty |
| `z` | Xoá bộ đếm xung |
| `r` | Bật/tắt báo cáo tự động 2 Hz |
| `k` | Đo vùng chết — **bánh sẽ quay** |
| `c` | Hướng dẫn đo xung mỗi vòng và đường kính bánh |
| `g` | Quay 500 mm theo encoder — **bánh sẽ quay** |
| `h` | Hiện bảng lệnh |

Phiên làm việc đầu tiên nên theo thứ tự này:

```
h          # xem bảng lệnh
e          # kiểm tra encoder đọc được (xoay bánh bằng tay, bấm e lại)
k          # đo vùng chết thật của cụm
r          # bật báo cáo tự động
w w w      # tăng dần lên 0.15 m/s, quan sát đích vs đo được vs duty
p          # tắt PI, so sánh với chạy duty tỉ lệ thẳng
space      # dừng
```

---

## 6. Hiệu chuẩn cho cụm của bạn

Làm theo thứ tự. Mỗi bước cho ra một hằng số để sửa vào đầu `main.c`.

### Bước 1 — Đo vùng chết → `MIN_DUTY`

Gõ `k`. Chương trình quét duty từ 5 % lên, mỗi mức 400 ms, và dừng ở mức đầu
tiên có xung:

```
  duty | xung
  -----+-----
   24% |    0
   25% |    0
   26% |   31   << bắt đầu quay

Vùng chết: 26%
-> Đặt MIN_DUTY khoảng 0.29
```

Lấy giá trị cao hơn mức đo được vài phần trăm, vì lực cản tăng khi mang tải.

### Bước 2 — Đo xung mỗi vòng bánh → `GEAR_RATIO`

1. Gõ `z` để xoá bộ đếm.
2. Đánh dấu một điểm trên lốp, quay bánh **bằng tay đúng 10 vòng**.
3. Gõ `e`, lấy trị tuyệt đối số xung chia cho 10.

Quay 10 vòng thay vì 1 để chia nhỏ sai số đọc.

```
GEAR_RATIO = (xung mỗi vòng bánh) / ENC_PPR_MOTOR
```

Ví dụ: đo được 9 370 xung cho 10 vòng → 937 xung/vòng → 937 / 20 = 46.85.

> Nếu không biết `ENC_PPR_MOTOR`, đếm số răng trên đĩa khía gắn ở trục motor.

### Bước 3 — Kiểm lại đường kính bánh → `WHEEL_RADIUS_MM`

1. Gõ `z`.
2. Lăn bánh trên sàn đúng 1 000 mm, đo bằng thước.
3. Gõ `e`, đọc số mm chương trình báo.

```
WHEEL_RADIUS_MM mới = cũ × (1000 / số chương trình báo)
```

Báo thiếu so với 1 000 nghĩa là bánh thật to hơn giá trị đang khai báo.

### Bước 4 — Kiểm tra tổng thể

Gõ `g`. Bánh quay đúng quãng ứng với 500 mm tính theo encoder. Đánh dấu lên lốp
trước khi chạy để đo được bánh lăn thực tế bao nhiêu. Sai số dưới 3 % là đạt.

Chương trình cũng in phần **vượt quá do quán tính** — con số này cho biết cần
phanh sớm bao nhiêu nếu sau này bạn làm điều khiển vị trí.

Sửa xong các hằng số thì build và nạp lại.

---

## 7. Giải thích code

File `main/main.c` chia làm sáu phần, đánh dấu bằng dải chú thích.

### 7.1 Pin map và thông số

Toàn bộ hằng số cần sửa nằm ở đầu file:

```c
#define PIN_PWM     GPIO_NUM_25
#define PIN_IN1     GPIO_NUM_26
#define PIN_IN2     GPIO_NUM_27
#define PIN_STBY    GPIO_NUM_12
#define PIN_ENC     GPIO_NUM_18

#define WHEEL_RADIUS_MM   30.0f
#define ENC_PPR_MOTOR     20.0f
#define GEAR_RATIO        (4123.0f / 88.0f)
#define MIN_DUTY          0.28f
```

Hai macro suy ra tự động, không sửa tay:

```c
#define PULSES_PER_REV  (ENC_PPR_MOTOR * GEAR_RATIO)
#define MM_PER_PULSE    (2.0f * (float)M_PI * WHEEL_RADIUS_MM / PULSES_PER_REV)
```

### 7.2 Điều khiển động cơ

TB6612 nhận 3 tín hiệu. Bảng chân lý:

| IN1 | IN2 | PWM | Kết quả |
|---|---|---|---|
| H | L | PWM | Quay thuận |
| L | H | PWM | Quay ngược |
| H | H | — | Phanh ngắn mạch |
| L | L | — | Thả trôi |

Hàm `drive()` là nơi duy nhất chạm vào động cơ:

```c
static void drive(float duty)   /* duty: -1.0 .. +1.0 */
{
    s_duty = duty;

    /* Chiều đếm encoder lấy theo duty thực đang cấp */
    if (duty != 0.0f) {
        s_pending_dir = (duty > 0.0f) ? 1 : -1;
    }

    if (duty > 0.0f)      { gpio_set_level(PIN_IN1, 1); gpio_set_level(PIN_IN2, 0); }
    else if (duty < 0.0f) { gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 1); }
    else                  { gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0); }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, fabsf(duty) * PWM_MAX);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
}
```

Chiều đếm encoder lấy theo **duty thực đang cấp**, không lấy theo vận tốc đích.
Khi vòng PI kéo ngược để hãm bánh, hai giá trị đó trái dấu nhau.

**PWM 20 kHz, độ phân giải 10 bit.** Dưới khoảng 18 kHz, cuộn dây động cơ rung ở
tần số PWM và phát ra tiếng rít. 1 024 mức duty là đủ mịn cho ứng dụng này.

**Bù vùng chết.** Động cơ không quay dưới `MIN_DUTY`, nên duty khác 0 được ánh xạ
lại vào dải `[MIN_DUTY .. 1.0]`:

```c
static float compensate_deadband(float duty)
{
    if (fabsf(duty) < 0.02f) {
        return 0.0f;      /* 0 phải ra đúng 0 */
    }
    float sign = (duty > 0.0f) ? 1.0f : -1.0f;
    return sign * (MIN_DUTY + fabsf(duty) * (1.0f - MIN_DUTY));
}
```

Câu `if` đầu hàm cần thiết để duty rất nhỏ do nhiễu tính toán không bị khuếch đại
thành `MIN_DUTY`.

### 7.3 Đọc encoder bằng PCNT

ESP32 có bộ đếm xung phần cứng (PCNT), đếm mà không cần CPU can thiệp:

```c
pcnt_unit_config_t ucfg = {
    .low_limit = -32767, .high_limit = 32767,
    .flags.accum_count = true,     /* tự cộng dồn khi tràn 16-bit */
};
pcnt_new_unit(&ucfg, &s_pcnt);

/* Lọc xung ngắn hơn 1µs — chống nhiễu từ chổi than */
pcnt_glitch_filter_config_t filt = { .max_glitch_ns = 1000 };
pcnt_unit_set_glitch_filter(s_pcnt, &filt);

pcnt_chan_config_t ccfg = {
    .edge_gpio_num  = PIN_ENC,
    .level_gpio_num = -1,
};
pcnt_new_channel(s_pcnt, &ccfg, &chan);
pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                             PCNT_CHANNEL_EDGE_ACTION_HOLD);
```

`accum_count = true` cho phép bộ đếm cộng dồn qua các lần tràn 16 bit.

Bộ lọc glitch 1 µs loại xung nhiễu ngắn. Không có nó, số xung có thể tự tăng khi
động cơ chạy dù bánh đứng yên.

### 7.4 Xác định chiều quay

PCNT đếm số xung; chiều quay do phần mềm gán, căn cứ vào duty đang cấp. Chiều mới
được áp dụng **có trễ**, trong `apply_pending_dir()`:

```c
if (s_pending_dir == s_dir) { s_flip_wait = 0; return; }

s_flip_wait++;
if (s_rate <= DIR_FLIP_PULSE_MAX ||          /* bánh đã gần dừng */
    s_flip_wait >= DIR_FLIP_TIMEOUT_SAMPLES) /* hoặc đã chờ đủ lâu */
{
    s_dir = s_pending_dir;
    s_speed_mps = 0.0f;    /* xoá tàn dư tốc độ chiều cũ khỏi bộ lọc */
    s_flip_wait = 0;
}
```

Lý do có trễ: sau khi đổi lệnh, bánh còn quay theo quán tính một lúc, và những
xung trong lúc đó vẫn thuộc chiều cũ.

Hai hằng số điều chỉnh:

| Hằng số | Giá trị | Ý nghĩa |
|---|---|---|
| `DIR_FLIP_PULSE_MAX` | 2 xung/chu kỳ | Ngưỡng coi như bánh đã dừng (≈ 0.02 m/s) |
| `DIR_FLIP_TIMEOUT_SAMPLES` | 8 chu kỳ | Chờ tối đa 160 ms rồi áp dụng chiều mới |

### 7.5 Vòng điều khiển 50 Hz

Chạy trong `control_task()`, chu kỳ 20 ms, làm ba việc.

**Lấy mẫu encoder:**

```c
pcnt_unit_get_count(s_pcnt, &raw);
int32_t delta = raw - s_last_raw;
s_last_raw = raw;
s_rate = abs(delta);

apply_pending_dir();

s_count += delta * s_dir;          /* xung tích luỹ có dấu */
```

**Tính vận tốc, lọc EMA:**

```c
float inst = (delta * s_dir) * MM_PER_PULSE / 1000.0f / dt;
s_speed_mps += SPEED_EMA_ALPHA * (inst - s_speed_mps);
```

Bộ lọc EMA (`alpha = 0.35`) làm mượt vận tốc. Ở tốc độ thấp chỉ có vài xung mỗi
chu kỳ nên giá trị tức thời dao động mạnh.

**Vòng PI:**

```c
float err = s_target_mps - s_speed_mps;
s_integ += err * dt;
if (s_integ >  PID_I_CLAMP) s_integ =  PID_I_CLAMP;
if (s_integ < -PID_I_CLAMP) s_integ = -PID_I_CLAMP;

float u = PID_KP * err + PID_KI * s_integ;
drive(compensate_deadband(u));
```

| Hằng số | Giá trị | Vai trò |
|---|---|---|
| `PID_KP` | 0.8 | Đáp ứng tức thời theo sai số |
| `PID_KI` | 2.0 | Khử sai số thường trực, giữ tốc độ khi tải thay đổi |
| `PID_I_CLAMP` | 0.5 | Chặn tích phân |
| `CTRL_PERIOD_MS` | 20 | Chu kỳ vòng lặp |

Không dùng khâu D. Khâu tích phân giữ cho tốc độ đúng giá trị đặt khi tải thay
đổi — ví dụ bánh ăn vào thảm hoặc lên dốc, duty tự tăng để bù.

Khi vận tốc đích bằng 0, tích phân được xoá để không tích luỹ lúc bánh đứng yên.

**Chỉnh PI:** tăng `PID_KP` cho đáp ứng nhanh hơn; nếu bánh dao động quanh tốc độ
đích thì giảm xuống. Tăng `PID_KI` nếu tốc độ ổn định nhưng không đạt đúng giá trị
đặt. Hạ `PID_I_CLAMP` xuống 0.25 nếu bánh vọt lên sau khi được gỡ khỏi chỗ kẹt.

Bấm `p` để tắt PI và chạy duty tỉ lệ thẳng — cách nhanh nhất để thấy vòng PI đang
làm gì.

### 7.6 Giao diện serial

Vòng lặp trong `app_main()` đọc từng ký tự và cập nhật vận tốc đích. Console được
cấu hình nhận từng ký tự, không cần nhấn Enter:

```c
setvbuf(stdin, NULL, _IONBF, 0);
uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0);
uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                      ESP_LINE_ENDINGS_CR);
```

---

## 8. Tham khảo nhanh

### 8.1 Hằng số cần sửa khi đổi phần cứng

| Hằng số | Mặc định | Sửa khi |
|---|---|---|
| `PIN_PWM` / `PIN_IN1` / `PIN_IN2` | GPIO 25 / 26 / 27 | Nối dây khác |
| `PIN_STBY` | GPIO 12 | Nối dây khác |
| `PIN_ENC` | GPIO 18 | Nối dây khác |
| `WHEEL_RADIUS_MM` | 30.0 | Bánh khác kích thước |
| `ENC_PPR_MOTOR` | 20.0 | Encoder khác số răng |
| `GEAR_RATIO` | 46.85 | Hộp số khác |
| `MIN_DUTY` | 0.28 | Động cơ khác |
| `MAX_SPEED_MPS` | 0.50 | Muốn giới hạn khác |
| `PWM_FREQ_HZ` | 20000 | Hiếm khi cần |

### 8.2 Các hàm chính

| Hàm | Việc |
|---|---|
| `drive(duty)` | Đặt duty thô `-1.0..1.0` |
| `brake()` | Phanh ngắn mạch |
| `compensate_deadband(duty)` | Ánh xạ duty vào dải có lực kéo |
| `set_target(v)` | Đặt vận tốc đích (m/s) |
| `apply_pending_dir()` | Áp dụng chiều đếm mới khi đủ điều kiện |
| `measure_deadband()` | Lệnh `k` |
| `go_distance_mm(mm)` | Lệnh `g` |
| `print_encoder()` | Lệnh `e` |

### 8.3 Biến trạng thái

| Biến | Ý nghĩa |
|---|---|
| `s_count` | Xung tích luỹ có dấu từ lần `z` gần nhất |
| `s_speed_mps` | Vận tốc đã lọc EMA |
| `s_duty` | Duty thực đang cấp |
| `s_dir` | Chiều đang dùng để đếm (+1 / −1) |
| `s_rate` | Số xung tuyệt đối trong chu kỳ vừa rồi |
| `s_target_mps` | Vận tốc đích |

### 8.4 Công thức

```
MM_PER_PULSE   = 2πR / (ENC_PPR_MOTOR × GEAR_RATIO)
quãng đường    = s_count × MM_PER_PULSE
vận tốc (m/s)  = Δxung × MM_PER_PULSE / 1000 / Δt
số vòng bánh   = s_count / (ENC_PPR_MOTOR × GEAR_RATIO)
```

---

## 9. Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| Bánh không quay, không tiếng động | VM đã có điện chưa; STBY đã HIGH chưa; GND đã chung chưa |
| Động cơ kêu nhưng bánh không quay | Duty dưới `MIN_DUTY`; kẹt cơ khí; nguồn không đủ dòng |
| Bánh quay nhưng số xung không tăng | Nguồn encoder; dây tín hiệu; cầu phân áp nếu encoder 5 V |
| Số xung tăng khi bánh đứng yên | Thêm tụ gốm 0.1 µF ở hai cực motor; kiểm tra mass chung |
| Chiều quay ngược với lệnh | Đảo hai dây ra motor, hoặc đổi `PIN_IN1` với `PIN_IN2` |
| Vận tốc đọc được nhảy loạn ở tốc độ thấp | Bình thường — mỗi chu kỳ chỉ có vài xung. Tăng `CTRL_PERIOD_MS` hoặc giảm `SPEED_EMA_ALPHA` |
| Bánh dao động quanh tốc độ đặt | Giảm `PID_KP` |
| Tốc độ ổn định nhưng không đạt giá trị đặt | Tăng `PID_KI` |
| ESP32 tự khởi động lại khi đổi chiều | Thêm tụ lọc ở VM |
| Động cơ rít | `PWM_FREQ_HZ` dưới 18 kHz |
| Board không boot | Gỡ trở kéo ở GPIO 12, hoặc chuyển STBY sang GPIO 13 |
| Quãng đường báo sai | Hiệu chuẩn lại theo mục 6 |

---

## Cấu trúc thư mục

```
wheel_motor/
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── main/
    └── main.c          — 483 dòng
```

## License

MIT
