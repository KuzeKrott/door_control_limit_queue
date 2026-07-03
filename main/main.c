/**
 * @file door_control.c
 * @brief Управление линейным актуатором FY017-250 через БУКД-5К и ЦАП MCP4725.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/queue.h"

/* ── Пины ─────────────────────────────────────────────────────────── */
#define PIN_SDA             4
#define PIN_SCL             5
#define PIN_START_STOP      6
#define PIN_DIR             7
#define PIN_LIMIT_CLOSE     15  /* коллектор PC817C, pullup            */

/* ── I2C / MCP4725 ────────────────────────────────────────────────── */
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000
#define MCP4725_ADDR        0x60
#define DAC_MAX             4095u

/* ── БУКД-5К ──────────────────────────────────────────────────────── */
#define DIR_CONTACT_CLOSED_FOR_OPEN  1
#define BUKD_PULSE_MS                100u

/* ── Актуатор ─────────────────────────────────────────────────────── */
#define V_MAX_MM_S              10.0f   /* реальная скорость при DAC_MAX, мм/с  */
#define CLOSE_VIRTUAL_STROKE_MM 240.0f  /* виртуальная дистанция закрытия, мм   */

#define D_ACCEL_MM  50.0f
#define D_COAST_MM  200.0f
#define D_DECEL_MM  50.0f

#define UPDATE_MS   50u

#define T_ACCEL_MS  ((uint32_t)(2000.0f * D_ACCEL_MM / V_MAX_MM_S))
#define T_COAST_MS  ((uint32_t)(1000.0f * D_COAST_MM / V_MAX_MM_S))

#define DOOR_MONITOR_ACTIVE_BIT    BIT0

static const char *TAG = "door";
static const char *TAG_OPEN = "door_open";

typedef enum {
    DOOR_STATE_OPENING,
    DOOR_STATE_OPEN,
} door_state_t;

typedef enum {
    DOOR_EVENT_INTERNAL_LIMIT,
    DOOR_EVENT_EXTERNAL_LIMIT,
    DOOR_EVENT_FAULT,
    DOOR_EVENT_TIMEOUT,
} door_event_t;

static QueueHandle_t door_event_queue = NULL;
static EventGroupHandle_t door_monitor_events;

/* ── Тип функции проверки условия досрочной остановки ────────────── */
typedef bool (*stop_check_fn_t)(void);

/* ═══════════════════════════════════════════════════════════════════
   ЦАП MCP4725
   ═══════════════════════════════════════════════════════════════════ */

static uint16_t s_dac_prev = UINT16_MAX;

static esp_err_t dac_set(uint16_t value)
{
    value &= DAC_MAX;

    uint8_t buf[2] = {
        (uint8_t)((value >> 8) & 0x0F),
        (uint8_t)(value & 0xFF)
    };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP4725_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, sizeof(buf), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50)); // показывает ESP_ERR_TIMEOUT при отключенном MCP4725, нужен для проверки наличия устройства
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK && value != s_dac_prev) {
        ESP_LOGI(TAG, "DAC %4"PRIu16" / %4u  (%5.1f%%)",
                 value, DAC_MAX, 100.0f * value / DAC_MAX);
        s_dac_prev = value;
    }

    return ret;  // Возвращает ESP_OK, если команда I2C прошла успешно, иначе код ошибки (например, ESP_ERR_TIMEOUT при отсутствии устройства)
}

/* ═══════════════════════════════════════════════════════════════════
   Концевик закрытия
   ═══════════════════════════════════════════════════════════════════ */

/**
 * NC-датчик + PC817C: металл обнаружен → фототранзистор закрыт → GPIO HIGH.
 * Возвращает true, когда дверь находится в закрытом положении.
 */
static bool limit_close_active(void)
{
    return gpio_get_level(PIN_LIMIT_CLOSE) == 1;
}

/* ═══════════════════════════════════════════════════════════════════
   Сухие контакты БУКД-5К
   ═══════════════════════════════════════════════════════════════════ */

static void contact_close(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD));
}

static void contact_open(gpio_num_t pin)
{
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_FLOATING));
}

static void bukd_pulse_start_stop(void)
{
    ESP_LOGI(TAG, "START/STOP импульс");
    contact_close(PIN_START_STOP);
    vTaskDelay(pdMS_TO_TICKS(BUKD_PULSE_MS));
    contact_open(PIN_START_STOP);
}

static void bukd_set_direction(bool opening)
{
#if DIR_CONTACT_CLOSED_FOR_OPEN
    opening ? contact_close(PIN_DIR) : contact_open(PIN_DIR);
#else
    opening ? contact_open(PIN_DIR) : contact_close(PIN_DIR);
#endif
    ESP_LOGI(TAG, "DIR: %s", opening ? "открытие" : "закрытие");
}

/* ═══════════════════════════════════════════════════════════════════
   Примитивы движения
   ═══════════════════════════════════════════════════════════════════ */

static void dac_ramp_up(uint16_t from, uint16_t to, uint32_t total_ms)
{
    uint32_t steps = total_ms / UPDATE_MS;
    if (steps == 0) { dac_set(to); return; }

    for (uint32_t i = 0; i <= steps; i++) {
        int32_t v = (int32_t)from
                  + ((int32_t)to - (int32_t)from) * (int32_t)i / (int32_t)steps;
        dac_set((uint16_t)v);
        vTaskDelay(pdMS_TO_TICKS(UPDATE_MS));
    }
}

/**
 * Линейно снижает DAC от from до 0 за duration_ms.
 * Проверяет should_stop() перед каждым шагом: если вернула true —
 * немедленно ставит DAC в 0 и возвращает управление.
 */
static esp_err_t dac_ramp_down(uint16_t from, uint32_t duration_ms,
                                stop_check_fn_t should_stop)
{
    if (duration_ms == 0) {
        dac_set(0);
        return false;
    }

    int64_t start_us = esp_timer_get_time();
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms) {
        if (should_stop()) {
            ESP_LOGI(TAG, "Торможение прервано по концевику (пройдено %"PRIu32" мс)",
                     elapsed_ms);
            dac_set(0);
            return true;
        }

        uint32_t remaining_ms = duration_ms - elapsed_ms;
        uint16_t dac = (uint16_t)(
            ((uint32_t)from * remaining_ms + duration_ms / 2) / duration_ms
        );

        esp_err_t err = dac_set(dac);
        if (err != ESP_OK) return err;

        vTaskDelay(pdMS_TO_TICKS(UPDATE_MS));
        elapsed_ms = (uint32_t)((esp_timer_get_time() - start_us) / 1000);
    }

    dac_set(0);
    return false;
}

/**
 * Ожидание duration_ms с проверкой should_stop() каждые UPDATE_MS.
 * Возвращает true, если ожидание прервано концевиком.
 */
static bool wait_with_stop_check(uint32_t duration_ms, stop_check_fn_t should_stop)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms) {
        if (should_stop()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(UPDATE_MS));
        elapsed_ms += UPDATE_MS;
    }

    return false;
}

/* ═══════════════════════════════════════════════════════════════════
   Профили движения
   ═══════════════════════════════════════════════════════════════════ */

static void door_stop_immediately(void)
{
    dac_set(0);
    bukd_pulse_start_stop();
    xEventGroupClearBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT);
}

static void door_open(void)
{
    door_event_t event;
    xQueueReset(door_event_queue);

    xEventGroupSetBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT);  // Включить мониторинг событий двери
    bukd_set_direction(true);
    dac_set(0);
    bukd_pulse_start_stop();
    ESP_LOGI(TAG_OPEN, "Открытие: разгон %"PRIu32" мс", T_ACCEL_MS);
    dac_ramp_up(0, DAC_MAX, T_ACCEL_MS);

    ESP_LOGI(TAG_OPEN, "Максимальная скорость достигнута");
    dac_set(DAC_MAX);

    ESP_LOGI(TAG_OPEN, "Ожидание limit, fault или timeout %"PRIu32" мс", T_COAST_MS);
    
    BaseType_t event_received = xQueueReceive(door_event_queue, &event, pdMS_TO_TICKS(T_COAST_MS)); // Ждать первое событие не дольше T_COAST_MS

    if (event_received == pdFALSE) {                                    // В очереди ничего не появилось за допустимое время
        event = DOOR_EVENT_TIMEOUT;                                     // Это таймаут    
    }

    switch(event) {
        case DOOR_EVENT_TIMEOUT:
            ESP_LOGW(TAG_OPEN, "Остановка двери: Вышло время %"PRIu32" мс", T_COAST_MS);
            door_stop_immediately();
            break;

        case DOOR_EVENT_INTERNAL_LIMIT:
            ESP_LOGW(TAG_OPEN, "Остановка двери: сработал внутренний концевик актуатора");
            door_stop_immediately();
            break;

        case DOOR_EVENT_FAULT:
            ESP_LOGE(TAG_OPEN, "Остановка двери: ошибка: значение тока превышает допустимое");
            door_stop_immediately();
            break;
        default:
            ESP_LOGE(TAG_OPEN, "Остановка: неизвестная ошибка");
            door_stop_immediately();
            break;
    }

    door_stop_immediately();
}

/**

 */
static void door_close(void)
{
    if (limit_close_active()) {
        ESP_LOGW(TAG, "Закрытие отменено: концевик уже активен");
        return;
    }

    if (CLOSE_VIRTUAL_STROKE_MM <= D_DECEL_MM) {
        ESP_LOGE(TAG, "Ошибка профиля: CLOSE_VIRTUAL_STROKE_MM должна быть > D_DECEL_MM");
        return;
    }

    const float coast_mm = CLOSE_VIRTUAL_STROKE_MM - D_DECEL_MM;
    const uint32_t coast_ms = (uint32_t)(1000.0f * coast_mm  / V_MAX_MM_S + 0.5f);
    const uint32_t decel_ms = (uint32_t)(2000.0f * D_DECEL_MM / V_MAX_MM_S + 0.5f);

    ESP_LOGI(TAG,
             "Закрытие: virtual=%.0f мм, ход=%"PRIu32" мс, торможение=%"PRIu32" мс",
             CLOSE_VIRTUAL_STROKE_MM, coast_ms, decel_ms);

    bukd_set_direction(false);
    ESP_ERROR_CHECK(dac_set(DAC_MAX));
    bukd_pulse_start_stop();

    ESP_LOGI(TAG, "Закрытие: максимальная скорость");
    bool stopped_early = wait_with_stop_check(coast_ms, limit_close_active);    // Проверяет, что концевик сработал во время хода 

    if (!stopped_early) {
        ESP_LOGI(TAG, "Закрытие: линейное торможение");
        stopped_early = dac_ramp_down(DAC_MAX, decel_ms, limit_close_active);  // Проверяет, что концевик сработал во время торможения
        vTaskDelay(pdMS_TO_TICKS(UPDATE_MS));    /* дать DAC=0 дойти до MCP4725 */
    }

    ESP_LOGI(TAG, "Закрытие: стоп%s",
             stopped_early ? " (по концевику)" : "(По расчётной позиции)");
    bukd_pulse_start_stop();
}

/* ═══════════════════════════════════════════════════════════════════
   Инициализация
   ═══════════════════════════════════════════════════════════════════ */

static void i2c_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_SDA,
        .scl_io_num       = PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

static void gpio_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_DIR) | (1ULL << PIN_START_STOP),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    contact_open(PIN_START_STOP);
    contact_open(PIN_DIR);

    gpio_config_t lim = {
        .pin_bit_mask = 1ULL << PIN_LIMIT_CLOSE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&lim));
}

static void door_monitor_init(void)
{
    door_event_queue = xQueueCreate(10, sizeof(door_event_t));
    door_monitor_events = xEventGroupCreate();

    xTaskCreate(overcurrent_monitor_task, "overcurrent_monitor", 2048, NULL, 5, NULL);
    xTaskCreate(internal_limit_monitor_task, "internal_limit_monitor", 2048, NULL, 4, NULL);
    xTaskCreate(external_limit_monitor_task, "external_limit_monitor", 2048, NULL, 4, NULL);
}

/* ═══════════════════════════════════════════════════════════════════
   Точка входа
   ═══════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    door_monitor_init();

    vTaskDelay(pdMS_TO_TICKS(5000));
    i2c_init();
    gpio_init();
    dac_set(0);

    ESP_LOGI(TAG, "Концевик закрытия при старте: %s",
             limit_close_active() ? "АКТИВЕН" : "не активен");

    vTaskDelay(pdMS_TO_TICKS(2000));

    door_open();
    vTaskDelay(pdMS_TO_TICKS(3000));
    door_close();
}
