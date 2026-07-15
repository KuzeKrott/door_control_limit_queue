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
#define I2C_FREQ_HZ         100000
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

#define TASK_POLL_MS          20u  
#define TASK_CONFIRM_STEP_MS  10u  
#define TASK_CONFIRM_TOTAL_MS 40u  

static const char *TAG = "door";
static const char *TAG_OPEN = "door_open";
static const char *TAG_CLOSE = "door_close";

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

    uint8_t buf[3] = {
        0x40,
        (uint8_t)(value >> 4),
        (uint8_t)((value & 0x0F) << 4)
    };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);

    i2c_master_write_byte(
        cmd,
        (MCP4725_ADDR << 1) | I2C_MASTER_WRITE,
        true
    );

    i2c_master_write(cmd, buf, sizeof(buf), true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        I2C_PORT,
        cmd,
        pdMS_TO_TICKS(100)
    );

    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK && value != s_dac_prev) {
        ESP_LOGI(TAG, "DAC %4" PRIu16 " / %4u (%5.1f%%)", value, DAC_MAX, 100.0f * value / DAC_MAX);
        s_dac_prev = value;
    }

    return ret;
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
   Линейное торможение ЦАП
   ═══════════════════════════════════════════════════════════════════ */

/**
 * Линейно снижает ЦАП от from до 0 за duration_ms, проверяя очередь
 * событий двери перед каждым шагом торможения.
 * Не выполняет остановку привода — dac_set(0), импульс «сухой контакт»
 * и очистка бита мониторинга остаются обязанностью door_close(),
 * которая делает это один раз после возврата из этой функции.
 * Возвращает true, если торможение завершилось по расчётному времени,
 * false — если было прервано событием.
 */
static bool dac_ramp_down(uint16_t from, uint32_t duration_ms)
{
    if (duration_ms == 0) {
        return true;
    }

    int64_t start_us = esp_timer_get_time();
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms) {
        door_event_t event;
        if (xQueueReceive(door_event_queue, &event, 0) == pdTRUE) {
            switch (event) {
                case DOOR_EVENT_FAULT:
                    ESP_LOGE(TAG_CLOSE, "Торможение: остановка двери — значение тока превышает допустимое");
                    return false;
                case DOOR_EVENT_INTERNAL_LIMIT:
                    ESP_LOGW(TAG_CLOSE, "Торможение: остановка двери — сработал внутренний концевик");
                    return false;
                case DOOR_EVENT_EXTERNAL_LIMIT:
                    ESP_LOGW(TAG_CLOSE, "Торможение: остановка двери — достигнут внешний концевик");
                    return false;
                default:
                    break;  /* событие не требует аварийной остановки — торможение продолжается */
            }
        }

        uint32_t remaining_ms = duration_ms - elapsed_ms;
        uint16_t dac = (uint16_t)(((uint32_t)from * remaining_ms + duration_ms / 2) / duration_ms);
        dac_set(dac);

        vTaskDelay(pdMS_TO_TICKS(UPDATE_MS));
        elapsed_ms = (uint32_t)((esp_timer_get_time() - start_us) / 1000);
    }

    ESP_LOGI(TAG_CLOSE, "Торможение: остановка двери — вышло расчётное время");
    return true;
}

static bool dac_ramp_up(uint16_t from, uint16_t to, uint32_t total_ms)
{
    uint32_t steps = total_ms / UPDATE_MS;
    if (steps == 0) steps = 1;

    for (uint32_t i = 0; i < steps; i++) {
        int32_t v = (int32_t)from + ((int32_t)to - (int32_t)from) * (int32_t)i / (int32_t)steps;

        door_event_t event;
        if (xQueueReceive(door_event_queue, &event, 0) == pdTRUE) {
            switch (event) {
                case DOOR_EVENT_FAULT:
                    ESP_LOGE(TAG_OPEN, "Разгон прерван: во время разгона ток превысил допустимое значение");
                    return false;
                case DOOR_EVENT_INTERNAL_LIMIT:
                    ESP_LOGW(TAG_OPEN, "Разгон прерван: во время разгона обнаружен внутренний концевик");
                    return false;
                default:
                    ESP_LOGE(TAG_OPEN, "Разгон прерван: неизвестное событие");
                    return false;
            }
        }

        dac_set((uint16_t)v);
        vTaskDelay(pdMS_TO_TICKS(UPDATE_MS));
    }

    return true;
}

static void door_stop_immediately(void)
{
    xEventGroupClearBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT);
    dac_set(0);
    bukd_pulse_start_stop();
}

static void door_open(void)
{
    door_event_t event;
    xQueueReset(door_event_queue);

    xEventGroupSetBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT);  // Включить мониторинг событий двери
    ESP_LOGI(TAG_OPEN, "Открытие: разгон %"PRIu32" мс", T_ACCEL_MS);
    bukd_set_direction(true);
    dac_set(0);
    bukd_pulse_start_stop();

    if (!dac_ramp_up(0, DAC_MAX, T_ACCEL_MS)) {
        door_stop_immediately();
        return;
    }

    ESP_LOGI(TAG_OPEN, "Открытие: ход %"PRIu32" мс", T_COAST_MS);
    dac_set(DAC_MAX);

    BaseType_t event_received = xQueueReceive(door_event_queue, &event, pdMS_TO_TICKS(T_COAST_MS)); // Ждать первое событие не дольше T_COAST_MS

    if (event_received == pdFALSE) {                                    // В очереди ничего не появилось за допустимое время
        event = DOOR_EVENT_TIMEOUT;                                     // Это таймаут    
    }

    switch(event) {
        case DOOR_EVENT_TIMEOUT:
            ESP_LOGW(TAG_OPEN, "Остановка двери: вышло время");
            door_stop_immediately();
            break;

        case DOOR_EVENT_FAULT:
            ESP_LOGE(TAG_OPEN, "Остановка двери: значение тока превышает допустимое");
            door_stop_immediately();
            break;

        case DOOR_EVENT_INTERNAL_LIMIT:
            ESP_LOGW(TAG_OPEN, "Остановка двери: внутренний концевик");
            door_stop_immediately();
            break;

        default:
            ESP_LOGE(TAG_OPEN, "Остановка двери: неизвестное событие");
            break;
    }
}


static void door_close(void)
{
    if (CLOSE_VIRTUAL_STROKE_MM <= D_DECEL_MM) {
        ESP_LOGE(TAG_CLOSE, "Ошибка профиля: CLOSE_VIRTUAL_STROKE_MM должна быть > D_DECEL_MM");
        return;
    }

    door_event_t event;
    xQueueReset(door_event_queue);
    xEventGroupSetBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT);  // Включить мониторинг событий двери

    const float coast_distance_mm = CLOSE_VIRTUAL_STROKE_MM - D_DECEL_MM;
    const uint32_t coast_ms = (uint32_t)(1000.0f * coast_distance_mm / V_MAX_MM_S);
    const uint32_t decel_ms = (uint32_t)(2.0f * 1000.0f * D_DECEL_MM / V_MAX_MM_S);

    ESP_LOGI(TAG_CLOSE,
             "Закрытие: virtual=%.0f мм, ход=%"PRIu32" мс, торможение=%"PRIu32" мс",
             CLOSE_VIRTUAL_STROKE_MM, coast_ms, decel_ms);

    bukd_set_direction(false);
    dac_set(DAC_MAX);
    bukd_pulse_start_stop();
    ESP_LOGI(TAG_CLOSE, "Закрытие: максимальная скорость");

    BaseType_t event_received = xQueueReceive(door_event_queue, &event, pdMS_TO_TICKS(coast_ms)); // Ждать первое событие не дольше coast_ms

    if (event_received == pdTRUE) {
        switch (event) {
            case DOOR_EVENT_FAULT:
                ESP_LOGE(TAG_CLOSE, "Остановка двери: значение тока превышает допустимое");
                door_stop_immediately();
                return;

            case DOOR_EVENT_INTERNAL_LIMIT:
                ESP_LOGW(TAG_CLOSE, "Остановка двери: внутренний концевик");
                door_stop_immediately();
                return;

            case DOOR_EVENT_EXTERNAL_LIMIT:
                ESP_LOGW(TAG_CLOSE, "Остановка двери: достигнут внешний концевик");
                door_stop_immediately();
                return;

            default:
                break;  /* прочие события не требуют аварийной остановки — переходим к торможению */
        }
    }

    ESP_LOGI(TAG_CLOSE, "Закрытие: линейное торможение");

    dac_ramp_down(DAC_MAX, decel_ms);  /* причина завершения торможения уже залогирована внутри */

    door_stop_immediately();
}

/* ═══════════════════════════════════════════════════════════════════
   Датчик тока актуатора
   ═══════════════════════════════════════════════════════════════════ */

/* Реализация амперметра находится во внешнем модуле датчика тока. */
static void current_sensor_init(void)
{
}
static float get_current(void)
{
    return 0.5f;
}

/**
 * Отправляет FAULT event, если ток актуатора остаётся не ниже 0.7 А
 * в течение 40 мс подряд. Работает постоянно; измеряет ток только пока
 * установлен DOOR_MONITOR_ACTIVE_BIT.
 */
static void overcurrent_monitor_task(void *arg)
{
    (void)arg;
    const float fault_threshold_a = 0.7f;
    current_sensor_init();

    for (;;) {
        xEventGroupWaitBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT,
                             pdFALSE, pdFALSE, portMAX_DELAY);

        if (!(xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT)) {
            continue;  // бит сняли раньше, чем задача проснулась
        }

        float current_a = get_current();

        while (xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT) {
            if (current_a < fault_threshold_a) {
                vTaskDelay(pdMS_TO_TICKS(TASK_POLL_MS));
                current_a = get_current();
                continue;
            }

            // Ток не ниже порога: подтверждение аварии в течение TASK_CONFIRM_TOTAL_MS
            uint32_t time_max_a = 0;
            bool door_stopped = false;

            while (time_max_a < TASK_CONFIRM_TOTAL_MS) {
                vTaskDelay(pdMS_TO_TICKS(TASK_CONFIRM_STEP_MS));

                if (!(xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT)) {
                    door_stopped = true;  // выйти к блокирующему ожиданию, если дверь остановлена
                    break;
                }

                current_a = get_current();
                if (current_a < fault_threshold_a) {
                    break;  // подтверждение отменено, current_a уже готов для верхнего цикла
                }

                time_max_a += TASK_CONFIRM_STEP_MS;
            }

            if (door_stopped) {
                break;  // вернуться к блокирующему ожиданию бита
            }

            if (time_max_a >= TASK_CONFIRM_TOTAL_MS) {
                door_event_t event = DOOR_EVENT_FAULT;
                xQueueSend(door_event_queue, &event, 0);
                ESP_LOGE(TAG, "Авария по току: ток выше %.1f А в течение %"PRIu32" мс",
                         fault_threshold_a, TASK_CONFIRM_TOTAL_MS);

                // Событие отправлено один раз: дальше только ждём снятия бита
                while (xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT) {
                    vTaskDelay(pdMS_TO_TICKS(TASK_CONFIRM_STEP_MS));
                }
                break;
            }
            // иначе current_a уже ниже порога — обычный контроль продолжается без повторного чтения
        }
    }
}

/**
 * Определяет внутренний концевик по устойчивому исчезновению тока актуатора.
 * Работает постоянно; измеряет ток только пока установлен DOOR_MONITOR_ACTIVE_BIT.
 */
static void internal_limit_monitor_task(void *arg)
{
    (void)arg;
    current_sensor_init();

    for (;;) {
        xEventGroupWaitBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT,
                             pdFALSE, pdFALSE, portMAX_DELAY);

        if (!(xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT)) {
            continue;  // бит сняли раньше, чем задача проснулась 
        }

        float prev_current = get_current();

        while (xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT) {
            vTaskDelay(pdMS_TO_TICKS(TASK_POLL_MS));
            float current = get_current();

            if (!(prev_current != 0.0f && current == 0.0f)) {
                prev_current = current;
                continue;
            }

            // Ток исчез: подтверждение падения тока в течение TASK_CONFIRM_TOTAL_MS 
            uint32_t time0 = 0;
            bool door_stopped = false;

            while (time0 < TASK_CONFIRM_TOTAL_MS) {
                vTaskDelay(pdMS_TO_TICKS(TASK_CONFIRM_STEP_MS));

                if (!(xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT)) {
                    door_stopped = true; //Флаг гарантирует выход из цикла подтверждения, если дверь остановлена
                    break;
                }

                current = get_current();
                if (current != 0.0f) {
                    prev_current = current;
                    break;
                }

                time0 += TASK_CONFIRM_STEP_MS;
            }

            if (door_stopped) {
                break;  // вернуться к блокирующему ожиданию бита 
            }

            if (time0 >= TASK_CONFIRM_TOTAL_MS) {
                door_event_t event = DOOR_EVENT_INTERNAL_LIMIT;
                xQueueSend(door_event_queue, &event, 0);
                ESP_LOGW(TAG, "Внутренний концевик: ток отсутствовал %"PRIu32" мс",
                         TASK_CONFIRM_TOTAL_MS);

                // Событие отправлено один раз: дальше только ждём снятия бита 
                while (xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT) {
                    vTaskDelay(pdMS_TO_TICKS(TASK_CONFIRM_STEP_MS));
                }
                break;
            }
            // иначе ток снова появился раньше срока — обычный контроль продолжается 
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
   Внешний Концевик закрытия
   ═══════════════════════════════════════════════════════════════════ */
/**
 * NC-датчик + PC817C: металл обнаружен → фототранзистор закрыт → GPIO HIGH.
 * Возвращает true, когда дверь находится в закрытом положении.
 */
static int get_limit(void)
{
    return gpio_get_level(PIN_LIMIT_CLOSE);
}

/**
 * Отправляет LIMIT event, если внешний концевик остаётся сработавшим
 * не менее 40 мс подряд. Работает постоянно; проверяет концевик только
 * пока установлен DOOR_MONITOR_ACTIVE_BIT.
 */
static void external_limit_monitor_task(void *arg)
{
    (void)arg;

    for (;;) {
        xEventGroupWaitBits(door_monitor_events, DOOR_MONITOR_ACTIVE_BIT,
                             pdFALSE, pdFALSE, portMAX_DELAY);

        if (!(xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT)) {
            continue;  // бит сняли раньше, чем задача проснулась
        }

        int limit = get_limit();

        while (xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT) {
            if (limit == 0) {
                vTaskDelay(pdMS_TO_TICKS(TASK_POLL_MS));
                limit = get_limit();
                continue;
            }

            // Концевик сработал: подтверждение в течение TASK_CONFIRM_TOTAL_MS
            uint32_t limit_time = 0;
            bool door_stopped = false;

            while (limit_time < TASK_CONFIRM_TOTAL_MS) {
                vTaskDelay(pdMS_TO_TICKS(TASK_CONFIRM_STEP_MS));

                if (!(xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT)) {
                    door_stopped = true;  // выйти к блокирующему ожиданию, если дверь остановлена
                    break;
                }

                limit = get_limit();
                if (limit == 0) {
                    break;  // подтверждение отменено, limit уже готов для верхнего цикла
                }

                limit_time += TASK_CONFIRM_STEP_MS;
            }

            if (door_stopped) {
                break;  // вернуться к блокирующему ожиданию бита
            }

            if (limit_time >= TASK_CONFIRM_TOTAL_MS) {
                door_event_t event = DOOR_EVENT_EXTERNAL_LIMIT;
                xQueueSend(door_event_queue, &event, 0);
                ESP_LOGW(TAG, "Внешний концевик: сработал %"PRIu32" мс",
                         TASK_CONFIRM_TOTAL_MS);

                // Событие отправлено один раз: дальше только ждём снятия бита
                while (xEventGroupGetBits(door_monitor_events) & DOOR_MONITOR_ACTIVE_BIT) {
                    vTaskDelay(pdMS_TO_TICKS(TASK_CONFIRM_STEP_MS));
                }
                break;
            }
            // иначе limit уже снят — обычная проверка продолжается без повторного чтения
        }
    }
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

    gpio_config_t lim = {       //включение подтягивающего резистора для концевика
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

/*static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Сканирование I2C начато");

    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(
            cmd,
            (address << 1) | I2C_MASTER_WRITE,
            true
        );
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(
            I2C_PORT,
            cmd,
            pdMS_TO_TICKS(30)
        );

        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Найдено I2C-устройство: 0x%02X", address);
        }

        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C-устройство 0x%02X не отвечает (таймаут)", address);
        }

        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Ошибка I2C при сканировании адреса 0x%02X: %s",
                     address, esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "Сканирование I2C завершено");
}*/

/* ═══════════════════════════════════════════════════════════════════
   Точка входа
   ═══════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    i2c_init();
    //*i2c_scan();
    gpio_init();

    door_monitor_init();

    for (int i = 0; i < 3; i++){
        ESP_LOGW(TAG, "%d ...", i+1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    door_open();

    for (int i = 0; i < 3; i++){
        ESP_LOGW(TAG, "%d ...", 3-i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    door_close();
}
