/*
 * Điều khiển MỘT cụm động cơ bánh xe bằng ESP32 + TB6612FNG,
 * đọc encoder, và chạy vòng điều khiển vận tốc PI.
 *
 * Giao tiếp qua USB serial, không cần WiFi. Build & nạp:
 *   idf.py set-target esp32 && idf.py -p /dev/ttyUSB0 flash monitor
 *
 * Nối dây và giải thích chi tiết: xem README.md.
 *
 * Bố cục file:
 *   1. Pin map và thông số cụm động cơ   (sửa ở đây cho khớp phần cứng)
 *   2. Điều khiển động cơ  — drive(), brake(), compensate_deadband()
 *   3. Đọc encoder         — PCNT, xác định chiều quay
 *   4. Vòng điều khiển 50Hz — lấy mẫu + PI
 *   5. Công cụ hiệu chuẩn  — đo vùng chết, đo xung/vòng, đi một quãng
 *   6. Giao diện serial
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/* ===================== 1. Pin map ===================== */
#define PIN_PWM     GPIO_NUM_25   /* PWMA của TB6612 */
#define PIN_IN1     GPIO_NUM_26   /* AIN1 */
#define PIN_IN2     GPIO_NUM_27   /* AIN2 */
#define PIN_STBY    GPIO_NUM_12   /* STBY — strapping pin, xem README */
#define PIN_ENC     GPIO_NUM_18   /* tín hiệu encoder */

/* ===================== Thông số cụm động cơ =====================
 * Giá trị mặc định ứng với một cụm đã hiệu chuẩn. Với cụm khác, chạy lệnh 'k'
 * và 'c' để đo lại rồi sửa vào đây. */
#define WHEEL_RADIUS_MM   30.0f               /* đường kính bánh 60mm */
#define ENC_PPR_MOTOR     20.0f               /* xung mỗi vòng TRỤC MOTOR */
#define GEAR_RATIO        (4123.0f / 88.0f)   /* ~46.85:1 */

/* Suy ra, không sửa tay */
#define PULSES_PER_REV    (ENC_PPR_MOTOR * GEAR_RATIO)              /* ~937 */
#define MM_PER_PULSE      (2.0f * (float)M_PI * WHEEL_RADIUS_MM / PULSES_PER_REV)

/* Duty tối thiểu để bánh bắt đầu quay (ma sát tĩnh). Đo bằng lệnh 'k'. */
#define MIN_DUTY          0.28f

/* ===================== Hằng số điều khiển ===================== */
#define PWM_FREQ_HZ       20000   /* trên ngưỡng nghe được -> động cơ không rít */
#define PWM_RES           LEDC_TIMER_10_BIT
#define PWM_MAX           1023
#define PWM_CHANNEL       LEDC_CHANNEL_0

#define CTRL_PERIOD_MS    20      /* 50Hz — cả lấy mẫu encoder lẫn PI */
#define SPEED_EMA_ALPHA   0.35f   /* lọc nhiễu lượng tử hoá ở tốc độ thấp */
#define PID_KP            0.8f
#define PID_KI            2.0f
#define PID_I_CLAMP       0.5f

#define PCNT_LIMIT        32767
#define MAX_SPEED_MPS     0.50f

/* Ngưỡng xung/chu kỳ coi như bánh đã dừng, mới cho phép đổi chiều đếm */
#define DIR_FLIP_PULSE_MAX       2
/* Chờ quá số chu kỳ này thì áp dụng chiều mới luôn — xem apply_pending_dir() */
#define DIR_FLIP_TIMEOUT_SAMPLES 8    /* 8 × 20ms = 160ms */

/* ===================== Trạng thái ===================== */
static pcnt_unit_handle_t s_pcnt;
static int      s_last_raw;
static int32_t  s_count;         /* xung tích luỹ CÓ DẤU */
static int      s_dir = 1;       /* chiều đang dùng để đếm: +1 / -1 */
static int      s_pending_dir = 1;
static uint32_t s_rate;          /* |xung| mỗi chu kỳ — đo bánh còn quay không */
static uint8_t  s_flip_wait;
static float    s_speed_mps;
static float    s_duty;          /* duty THỰC đang cấp */
static float    s_target_mps;
static float    s_integ;

static bool s_pi_on = true;
static volatile bool s_report = false;

/* ===================== 2. Điều khiển động cơ ===================== */

/* Đặt duty thô [-1..1]. Đây là nơi DUY NHẤT chạm vào động cơ. */
static void drive(float duty)
{
    if (duty > 1.0f)  duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    s_duty = duty;

    /* Encoder cho biết bánh quay bao nhiêu, chiều quay do phần mềm xác định, và
     * căn cứ đúng là DUTY THỰC đang cấp — không phải vận tốc đích. Khi PI kéo
     * ngược để hãm bánh, hai giá trị đó trái dấu nhau. */
    if (duty != 0.0f) {
        s_pending_dir = (duty > 0.0f) ? 1 : -1;
    }

    if (duty > 0.0f) {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
    } else if (duty < 0.0f) {
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
    } else {
        gpio_set_level(PIN_IN1, 0);   /* IN1=IN2=LOW -> thả trôi */
        gpio_set_level(PIN_IN2, 0);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL,
                  (uint32_t)(fabsf(duty) * PWM_MAX));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
}

/* Phanh ngắn mạch: IN1=IN2=HIGH. Dừng nhanh hơn thả trôi. */
static void brake(void)
{
    gpio_set_level(PIN_IN1, 1);
    gpio_set_level(PIN_IN2, 1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
    s_duty = 0.0f;
}

/* Ánh xạ duty khác 0 vào dải [MIN_DUTY..1.0] để lệnh nhỏ vẫn đủ lực kéo. */
static float compensate_deadband(float duty)
{
    if (fabsf(duty) < 0.02f) {
        return 0.0f;      /* 0 phải ra đúng 0 */
    }
    float sign = (duty > 0.0f) ? 1.0f : -1.0f;
    return sign * (MIN_DUTY + fabsf(duty) * (1.0f - MIN_DUTY));
}

/* ===================== 3. Đọc encoder ===================== */

static esp_err_t encoder_init(void)
{
    pcnt_unit_config_t ucfg = {
        .low_limit = -PCNT_LIMIT,
        .high_limit = PCNT_LIMIT,
        .flags.accum_count = true,     /* tự cộng dồn khi tràn 16-bit */
    };
    esp_err_t e = pcnt_new_unit(&ucfg, &s_pcnt);
    if (e != ESP_OK) {
        return e;
    }

    /* Lọc xung nhiễu ngắn hơn 1µs. Động cơ chổi than phát nhiễu điện khá mạnh;
     * thiếu bộ lọc này thì số xung có thể tự tăng khi bánh đứng yên. */
    pcnt_glitch_filter_config_t filt = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(s_pcnt, &filt);

    pcnt_chan_config_t ccfg = {
        .edge_gpio_num = PIN_ENC,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t chan;
    e = pcnt_new_channel(s_pcnt, &ccfg, &chan);
    if (e != ESP_OK) {
        return e;
    }
    pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_unit_add_watch_point(s_pcnt, PCNT_LIMIT);
    pcnt_unit_enable(s_pcnt);
    pcnt_unit_clear_count(s_pcnt);
    pcnt_unit_start(s_pcnt);

    gpio_set_pull_mode(PIN_ENC, GPIO_PULLUP_ONLY);
    return ESP_OK;
}

/*
 * Áp dụng chiều đếm mới, có trễ.
 *
 * Sau khi đổi lệnh, bánh còn quay theo quán tính thêm một lúc, và những xung
 * trong lúc đó vẫn thuộc chiều cũ. Vì vậy chiều mới chỉ được áp dụng khi bánh
 * đã gần dừng (rate <= DIR_FLIP_PULSE_MAX).
 *
 * Kèm timeout: dưới lực kéo ngược của PI, bánh có thể đi qua vùng gần dừng
 * nhanh hơn một chu kỳ 20ms. Chờ quá DIR_FLIP_TIMEOUT_SAMPLES chu kỳ thì áp dụng
 * chiều mới luôn, để chiều đếm không bị kẹt ở giá trị cũ.
 */
static void apply_pending_dir(void)
{
    if (s_pending_dir == s_dir) {
        s_flip_wait = 0;
        return;
    }
    s_flip_wait++;
    if (s_rate <= DIR_FLIP_PULSE_MAX ||
        s_flip_wait >= DIR_FLIP_TIMEOUT_SAMPLES) {
        s_dir = s_pending_dir;
        s_speed_mps = 0.0f;   /* xoá tàn dư tốc độ chiều cũ khỏi bộ lọc */
        s_flip_wait = 0;
    }
}

/* ===================== 4. Vòng điều khiển 50Hz ===================== */

static void control_task(void *arg)
{
    const float dt = CTRL_PERIOD_MS / 1000.0f;
    const float mpp = MM_PER_PULSE / 1000.0f;   /* mét mỗi xung */
    int64_t last_report = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));

        /* --- Lấy mẫu encoder --- */
        int raw = 0;
        pcnt_unit_get_count(s_pcnt, &raw);
        int32_t delta = (int32_t)(raw - s_last_raw);
        s_last_raw = raw;
        s_rate = (uint32_t)(delta >= 0 ? delta : -delta);

        apply_pending_dir();

        int32_t sdelta = delta * s_dir;
        s_count += sdelta;

        /* --- Vận tốc, lọc EMA --- */
        float inst = sdelta * mpp / dt;
        s_speed_mps += SPEED_EMA_ALPHA * (inst - s_speed_mps);

        /* --- Vòng PI --- */
        if (s_pi_on) {
            if (s_target_mps == 0.0f) {
                s_integ = 0.0f;       /* không tích luỹ khi đang đứng yên */
                drive(0.0f);
            } else {
                float err = s_target_mps - s_speed_mps;
                s_integ += err * dt;
                if (s_integ >  PID_I_CLAMP) s_integ =  PID_I_CLAMP;
                if (s_integ < -PID_I_CLAMP) s_integ = -PID_I_CLAMP;
                float u = PID_KP * err + PID_KI * s_integ;
                drive(compensate_deadband(u));
            }
        }

        int64_t now = esp_timer_get_time();
        if (s_report && now - last_report > 500000) {
            last_report = now;
            printf("đích %+.3f | đo %+.3f m/s | duty %+.2f | xung %ld | %.1f mm\n",
                   s_target_mps, s_speed_mps, s_duty,
                   (long)s_count, s_count * MM_PER_PULSE);
        }
    }
}

/* ===================== 5. Công cụ hiệu chuẩn ===================== */

static void enc_zero(void)
{
    s_count = 0;
    printf("Đã xoá bộ đếm xung.\n");
}

/* Quét duty để tìm mức nhỏ nhất bánh bắt đầu quay -> giá trị MIN_DUTY */
static void measure_deadband(void)
{
    bool pi_was = s_pi_on;
    s_pi_on = false;
    s_target_mps = 0.0f;

    printf("\nĐo vùng chết — bánh sẽ quay.\n");
    printf("  duty | xung\n");
    printf("  -----+-----\n");

    int found = -1;
    for (int d = 5; d <= 60; d++) {
        int32_t c0 = s_count;
        drive(d / 100.0f);
        vTaskDelay(pdMS_TO_TICKS(400));
        int32_t n = s_count - c0;
        drive(0.0f);

        if (n < 0) {
            n = -n;
        }
        if (n && found < 0) {
            found = d;
        }
        printf("  %3d%% | %4ld%s\n", d, (long)n,
               (found == d) ? "   << bắt đầu quay" : "");
        if (found > 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    if (found > 0) {
        printf("\nVùng chết: %d%%\n", found);
        printf("-> Đặt MIN_DUTY khoảng %.2f (cao hơn mức đo được vài phần trăm\n"
               "   cho chắc, vì lực cản tăng khi mang tải).\n", (found + 3) / 100.0f);
    } else {
        printf("\nKhông thấy xung nào tới duty 60%%. Xem mục Xử lý sự cố.\n");
    }
    s_pi_on = pi_was;
}

static void calib_help(void)
{
    printf("\n=== Đo số xung mỗi vòng bánh ===\n");
    printf("1. Bấm 'z' để xoá bộ đếm.\n");
    printf("2. Đánh dấu một điểm trên lốp, quay bánh BẰNG TAY đúng 10 vòng.\n");
    printf("   (10 vòng chứ không phải 1: sai số đọc chia cho 10.)\n");
    printf("3. Bấm 'e', lấy |xung| chia 10 = xung mỗi vòng bánh.\n");
    printf("4. GEAR_RATIO = xung mỗi vòng bánh / %.0f\n", ENC_PPR_MOTOR);
    printf("Đang khai báo: %.0f xung/vòng bánh, tỉ số %.2f:1, %.4f mm/xung.\n\n",
           PULSES_PER_REV, GEAR_RATIO, MM_PER_PULSE);
    printf("=== Kiểm lại đường kính bánh ===\n");
    printf("1. Bấm 'z', lăn bánh trên sàn đúng 1000mm (đo bằng thước).\n");
    printf("2. Bấm 'e', đọc số mm chương trình báo.\n");
    printf("3. WHEEL_RADIUS_MM mới = cũ x (1000 / số chương trình báo).\n\n");
}

/* Quay bánh đúng một quãng tính theo encoder — phép kiểm tra cuối: nếu các hằng
 * số đúng thì quãng đường đo bằng thước khớp với con số yêu cầu. */
static void go_distance_mm(float mm)
{
    bool pi_was = s_pi_on;
    s_pi_on = false;
    s_target_mps = 0.0f;

    int32_t c0 = s_count;
    int32_t need = (int32_t)(mm / MM_PER_PULSE);
    printf("Quay %.0fmm = %ld xung...\n", mm, (long)need);

    drive(compensate_deadband(0.35f));
    int64_t deadline = esp_timer_get_time() + 15000000;   /* chống treo */
    while (esp_timer_get_time() < deadline) {
        if (s_count - c0 >= need) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    brake();
    vTaskDelay(pdMS_TO_TICKS(300));

    int32_t done = s_count - c0;
    printf("Xong: %ld xung = %.1f mm (vượt quá do quán tính: %.1f mm)\n",
           (long)done, done * MM_PER_PULSE, (done - need) * MM_PER_PULSE);
    printf("Đo bằng thước xem bánh lăn được bao nhiêu — lệch quá 3%% thì\n"
           "WHEEL_RADIUS_MM hoặc GEAR_RATIO chưa đúng.\n");
    s_pi_on = pi_was;
}

/* ===================== 6. Giao diện serial ===================== */

static void set_target(float v)
{
    s_target_mps = v;
    if (!s_pi_on) {
        drive(compensate_deadband(v / MAX_SPEED_MPS));
    }
}

static void print_help(void)
{
    printf("\n--- Lệnh ---\n"
           "  w / s : tăng / giảm vận tốc đích 0.05 m/s\n"
           "  space : dừng (thả trôi)\n"
           "  x     : phanh gấp\n"
           "  p     : bật/tắt vòng PI vận tốc (đang %s)\n"
           "  e     : in encoder: xung, quãng đường, vận tốc\n"
           "  z     : xoá bộ đếm xung\n"
           "  r     : bật/tắt báo cáo tự động 2Hz\n"
           "  k     : ĐO vùng chết (bánh sẽ quay)\n"
           "  c     : hướng dẫn đo xung/vòng và đường kính bánh\n"
           "  g     : quay 500mm theo encoder (bánh sẽ quay)\n"
           "  h     : trợ giúp\n\n", s_pi_on ? "BẬT" : "TẮT");
}

static void print_encoder(void)
{
    printf("%+8ld xung | %+9.1f mm | %+6.3f m/s | %+6.2f vòng bánh | duty %+.2f\n",
           (long)s_count, s_count * MM_PER_PULSE, s_speed_mps,
           s_count / PULSES_PER_REV, s_duty);
}

static void console_init(void)
{
    /* Mặc định stdin gom theo DÒNG: getchar() chỉ trả về sau khi bấm Enter. */
    setvbuf(stdin, NULL, _IONBF, 0);
    uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0);
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                          ESP_LINE_ENDINGS_CR);
}

void app_main(void)
{
    console_init();

    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2) | (1ULL << PIN_STBY),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&out);
    gpio_set_level(PIN_STBY, 1);      /* LOW = driver ngủ, phải kéo HIGH */

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {
        .gpio_num = PIN_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ledc_channel_config(&ccfg);

    if (encoder_init() != ESP_OK) {
        printf("LỖI: không khởi tạo được PCNT\n");
    }

    printf("\n\n=== Điều khiển cụm động cơ bánh xe ===\n");
    printf("Động cơ: PWM=GPIO%d IN1=GPIO%d IN2=GPIO%d STBY=GPIO%d\n",
           PIN_PWM, PIN_IN1, PIN_IN2, PIN_STBY);
    printf("Encoder: GPIO%d\n", PIN_ENC);
    printf("Bánh R=%.1fmm, hộp số %.2f:1, %.0f xung/vòng bánh, %.4f mm/xung\n",
           WHEEL_RADIUS_MM, GEAR_RATIO, PULSES_PER_REV, MM_PER_PULSE);
    printf("PWM %dHz 10-bit, vòng điều khiển %dHz, Kp=%.1f Ki=%.1f\n",
           PWM_FREQ_HZ, 1000 / CTRL_PERIOD_MS, PID_KP, PID_KI);

    xTaskCreate(control_task, "ctrl", 4096, NULL, 12, NULL);
    print_help();

    float v = 0.0f;
    while (1) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        switch (c) {
        case 'w': v += 0.05f; break;
        case 's': v -= 0.05f; break;
        case ' ': v = 0.0f;   break;
        case 'x':
            v = 0.0f;
            set_target(0.0f);
            brake();
            printf("PHANH\n");
            continue;
        case 'p':
            s_pi_on = !s_pi_on;
            s_integ = 0.0f;
            printf("Vòng PI: %s\n", s_pi_on ? "BẬT" : "TẮT (duty tỉ lệ thẳng)");
            continue;
        case 'e': print_encoder(); continue;
        case 'z': enc_zero(); continue;
        case 'r': s_report = !s_report;
                  printf("Báo cáo tự động: %s\n", s_report ? "BẬT" : "TẮT");
                  continue;
        case 'k': v = 0.0f; set_target(0.0f); measure_deadband(); continue;
        case 'c': calib_help(); continue;
        case 'g': v = 0.0f; set_target(0.0f); go_distance_mm(500.0f); continue;
        case 'h': print_help(); continue;
        default: continue;
        }

        if (v >  MAX_SPEED_MPS) v =  MAX_SPEED_MPS;
        if (v < -MAX_SPEED_MPS) v = -MAX_SPEED_MPS;
        set_target(v);
        printf("vận tốc đích: %+.2f m/s\n", v);
    }
}
