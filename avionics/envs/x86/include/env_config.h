#pragma once
#include "HAL/time.h"
constexpr char BAROMETER_DATA[] = "data/barometer.csv";
constexpr char LOG_FILE_NAME[] = "datalog.csv";

void env_initialize();
void env_callbacks();
#define THIS_IS_NATIVE_CONFIGURATION
