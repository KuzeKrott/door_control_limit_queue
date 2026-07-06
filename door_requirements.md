# Door controller requirements

Platform: ESP32-S3, ESP-IDF, C, FreeRTOS.

Existing hardware functions:
- dac_set(uint16_t value)
- bukd_set_direction(bool open)
- bukd_pulse_start_stop(void)

Door event queue already exists:
- QueueHandle_t door_event_open_queue

Events:
- DOOR_EVENT_TIMEOUT
- DOOR_EVENT_FAULT
- DOOR_EVENT_INTERNAL_LIMIT

Monitoring:
- Event Group bit DOOR_MONITOR_ACTIVE_BIT.
- Before any software stop: clear monitoring bit.
- Then dac_set(0).
- Then bukd_pulse_start_stop().

Architecture:
- Monitor tasks only send events.
- Only door_open / door_close stop the motor.
- Queue must be reset before a new movement.
- Monitor tasks must not send repeated events after first trigger.

dac_ramp_up:
- Returns true if acceleration completed.
- Returns false if an event stops movement.
- Must check queue during acceleration.
- Unknown event is treated as stop/error.
- UPDATE_MS = 20.
- If total_ms / UPDATE_MS == 0, steps must be 1.

Code style:
- Keep existing naming style.
- Minimal comments only where non-obvious.
- Do not refactor unrelated code.
- Do not create new files unless required.
- Do not invent hardware APIs.