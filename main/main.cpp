/*
 * ESP32 Line Follower Robot (ESP-IDF)
 * روبوت تتبّع خط على ESP32
 *
 * - 5 analog line sensors, PID steering, differential-drive motors.
 * - Auto-detects "reversed zone" (white line on black background).
 *
 * - 5 حساسات تناظرية، توجيه بـ PID، محركان بنظام تفاضلي.
 * - يكتشف تلقائيًا "المنطقة المعكوسة" (خط أبيض على أرضية سوداء).
 */

#include <atomic>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

// ===================== Pins / التوصيلات =====================
constexpr gpio_num_t MOT_L_PWM = GPIO_NUM_18;
constexpr gpio_num_t MOT_L_IN1 = GPIO_NUM_19;
constexpr gpio_num_t MOT_L_IN2 = GPIO_NUM_21;

constexpr gpio_num_t MOT_R_PWM = GPIO_NUM_25;
constexpr gpio_num_t MOT_R_IN1 = GPIO_NUM_22;
constexpr gpio_num_t MOT_R_IN2 = GPIO_NUM_23;

constexpr int NUM_SENSORS = 5;
constexpr adc_channel_t SENSOR_CHANNELS[NUM_SENSORS] = {
    ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_4};

// ===================== Tuning / الإعدادات =====================
// PID
constexpr float Kp = 0.075f;
constexpr float Ki = 0.0001f;
constexpr float Kd = 0.60f;
constexpr float INTEGRAL_DECAY = 0.90f;    // leaky integral / تكامل متسرّب
constexpr float INTEGRAL_LIMIT = 20000.0f; // safety clamp / حد أمان

// Speed / السرعة
constexpr float MAX_PWM = 255.0f;
constexpr float BASE_SPEED_L = 100.0f;
constexpr float BASE_SPEED_R = 100.0f;
constexpr float NORMAL_SPEED_FACTOR = 1.15f;   // black line / خط أسود
constexpr float REVERSED_SPEED_FACTOR = 0.85f; // white line / خط أبيض
constexpr float REVERSED_PID_SCALE = 0.80f;
constexpr float STRONG_TURN_FACTOR = 0.75f;
constexpr float RECOVERY_SPEED = 70.0f;        // spin speed when line lost

// Sensors / الحساسات
constexpr int SENSOR_MAX = 4095;
constexpr int SENSOR_THRESHOLD = 700;     // ignore values below this / تجاهل ما دونها
constexpr int STRONG_TURN_THRESHOLD = 1500;
constexpr float POS_CENTER = 2000.0f;     // (NUM_SENSORS-1)*1000/2
constexpr float POS_LINE_LOST = -1.0f;

// Reversed-zone detection / اكتشاف المنطقة المعكوسة
constexpr int REVERSE_ENTER_BLACK_COUNT = 4; // >= this -> enter reversed
constexpr int REVERSE_EXIT_BLACK_COUNT = 1;  // <= this -> leave reversed
constexpr int REVERSE_DEBOUNCE_CYCLES = 10;  // was 5 / كانت 5

// Timing
constexpr int LOOP_PERIOD_MS = 10;
constexpr int START_DELAY_MS = 3000; // wait before moving / انتظار قبل الحركة

// ===================== Shared state / الحالة المشتركة =====================
// Written by SensorTask, read by ControlTask.
// تُكتب من مهمة الحساسات وتُقرأ من مهمة التحكم.
// std::atomic is the standard-correct way to share data between two cores
// (lock-free for 32-bit types on ESP32, so no performance cost).
// std::atomic هو الأسلوب القياسي الصحيح لمشاركة البيانات بين نواتين
// (بدون أقفال لأنواع 32-بت على ESP32، فلا كلفة على الأداء).
std::atomic<float> current_pos{POS_CENTER};
std::atomic<float> last_pos{POS_CENTER};
std::atomic<bool> strong_turn_detected{false};
std::atomic<bool> reverse_map{false};

// Used only inside ControlTask / تُستخدم داخل مهمة التحكم فقط
static float last_error = 0.0f;
static float integral = 0.0f;

static adc_oneshot_unit_handle_t adc_handle;

// ===================== Motor drive / تحريك المحركات =====================
static void drive(float left, float right)
{
    left = fmaxf(-MAX_PWM, fminf(MAX_PWM, left));
    right = fmaxf(-MAX_PWM, fminf(MAX_PWM, right));

    gpio_set_level(MOT_L_IN1, left < 0);
    gpio_set_level(MOT_L_IN2, left >= 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)fabsf(left));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    gpio_set_level(MOT_R_IN1, right < 0);
    gpio_set_level(MOT_R_IN2, right >= 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)fabsf(right));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// ===================== Sensor task / مهمة الحساسات =====================
static void SensorTask(void *)
{
    int reverse_debounce = 0;

    while (true)
    {
        int raw[NUM_SENSORS];
        int processed[NUM_SENSORS];
        long weighted_sum = 0;
        int total = 0;
        int black_count = 0;
        bool read_ok = true;

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (adc_oneshot_read(adc_handle, SENSOR_CHANNELS[i], &raw[i]) != ESP_OK)
            {
                read_ok = false; // skip this whole cycle / تخطَّ الدورة كلها
                break;
            }
        }
        if (!read_ok)
        {
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
            continue;
        }

        // Normal: black line = low raw value, so invert.
        // Reversed: white line = high raw value, use as is.
        // عادي: الخط الأسود قيمته الخام منخفضة فنعكسها. معكوس: نستخدمها كما هي.
        const bool reversed = reverse_map.load();
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if ((SENSOR_MAX - raw[i]) > SENSOR_THRESHOLD) black_count++;

            processed[i] = reversed ? raw[i] : (SENSOR_MAX - raw[i]);

            int norm = processed[i];
            if (norm < SENSOR_THRESHOLD) norm = 0;

            weighted_sum += (long)norm * (i * 1000);
            total += norm;
        }

        // Reversed-zone detection with debounce.
        // اكتشاف المنطقة المعكوسة مع تأكيد لعدة دورات.
        if (!reversed && black_count >= REVERSE_ENTER_BLACK_COUNT)
        {
            if (++reverse_debounce > REVERSE_DEBOUNCE_CYCLES)
            {
                reverse_map.store(true);
                reverse_debounce = 0;
            }
        }
        else if (reversed && black_count <= REVERSE_EXIT_BLACK_COUNT)
        {
            if (++reverse_debounce > REVERSE_DEBOUNCE_CYCLES)
            {
                reverse_map.store(false);
                reverse_debounce = 0;
            }
        }
        else
        {
            reverse_debounce = 0;
        }

        if (total > 0)
        {
            float pos = (float)weighted_sum / total;
            last_pos.store(pos);
            current_pos.store(pos);
        }
        else
        {
            current_pos.store(POS_LINE_LOST);
        }

        strong_turn_detected.store(
            processed[0] > STRONG_TURN_THRESHOLD || processed[NUM_SENSORS - 1] > STRONG_TURN_THRESHOLD);

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

// ===================== Control task / مهمة التحكم =====================
static void ControlTask(void *)
{
    bool prev_reversed = false;

    while (true)
    {
        // Snapshot shared values ONCE per loop.
        // خذ نسخة من القيم المشتركة مرة واحدة في كل دورة.
        const float pos = current_pos.load();
        const float lastp = last_pos.load();
        const bool reversed = reverse_map.load();
        const bool strong_turn = strong_turn_detected.load();

        // Line lost: spin toward the last known side.
        // فُقد الخط: لُفّ نحو آخر جهة معروفة.
        if (pos < 0.0f)
        {
            integral = 0.0f;
            last_error = 0.0f;
            const float dir = (lastp > POS_CENTER) ? 1.0f : -1.0f;
            drive(RECOVERY_SPEED * dir, -RECOVERY_SPEED * dir);
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
            continue;
        }

        // Zone changed: clear PID memory to avoid a kick.
        // تغيّرت المنطقة: صفّر ذاكرة الـ PID لتجنب الدفعة المفاجئة.
        if (reversed != prev_reversed)
        {
            integral = 0.0f;
            last_error = 0.0f;
            prev_reversed = reversed;
        }

        const float error = pos - POS_CENTER;
        integral = (integral + error) * INTEGRAL_DECAY;
        integral = fmaxf(-INTEGRAL_LIMIT, fminf(INTEGRAL_LIMIT, integral));
        const float derivative = error - last_error;
        last_error = error;

        const float pid = (Kp * error) + (Ki * integral) + (Kd * derivative);

        float eff_L = BASE_SPEED_L;
        float eff_R = BASE_SPEED_R;
        float pid_scale = 1.0f;

        if (!reversed)
        {
            eff_L *= NORMAL_SPEED_FACTOR;
            eff_R *= NORMAL_SPEED_FACTOR;
        }
        else
        {
            eff_L *= REVERSED_SPEED_FACTOR;
            eff_R *= REVERSED_SPEED_FACTOR;
            pid_scale = REVERSED_PID_SCALE;
        }

        if (strong_turn)
        {
            eff_L *= STRONG_TURN_FACTOR;
            eff_R *= STRONG_TURN_FACTOR;
        }

        drive(eff_L + (pid * pid_scale), eff_R - (pid * pid_scale));
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

// ===================== Entry point / نقطة البدء =====================
extern "C" void app_main()
{
    gpio_set_direction(MOT_L_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOT_L_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOT_R_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOT_R_IN2, GPIO_MODE_OUTPUT);

    ledc_timer_config_t timer{};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.timer_num = LEDC_TIMER_0;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    ledc_channel_config_t ch{};
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.timer_sel = LEDC_TIMER_0;
    ch.duty = 0;
    ch.channel = LEDC_CHANNEL_0;
    ch.gpio_num = MOT_L_PWM;
    ledc_channel_config(&ch);

    ch.channel = LEDC_CHANNEL_1;
    ch.gpio_num = MOT_R_PWM;
    ledc_channel_config(&ch);

    adc_oneshot_unit_init_cfg_t adc_cfg{};
    adc_cfg.unit_id = ADC_UNIT_1;
    adc_oneshot_new_unit(&adc_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg{};
    chan_cfg.atten = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    for (auto c : SENSOR_CHANNELS) adc_oneshot_config_channel(adc_handle, c, &chan_cfg);

    // Stay still for a moment so you can place the robot on the track.
    // ابقَ ساكنًا للحظات حتى تضع الروبوت على المضمار.
    drive(0, 0);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, nullptr, 5, nullptr, 1);
}
