#pragma once
#include "esp_err.h"

esp_err_t ina226_init(float shunt_ohms, float current_lsb_a);
float     ina226_read_current_a(void);
float     ina226_read_bus_voltage_v(void);