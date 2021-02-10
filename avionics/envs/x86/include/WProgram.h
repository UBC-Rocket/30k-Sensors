// Dummy file for compilation purposes.
#pragma once
#include "HAL/port_impl.h"
#include "HAL/time.h"

typedef Hal::Serial Stream;
static Stream& Serial{Hal::SerialInst::USB};
inline uint32_t millis() { return Hal::now_ms().time_since_epoch().count(); }
