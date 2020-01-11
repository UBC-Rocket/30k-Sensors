#ifndef HAL__GPIO_H_41B1B7824F0141F8A5A03070A0D0A3F3
#define HAL__GPIO_H_41B1B7824F0141F8A5A03070A0D0A3F3

//workaround Arduino's framework that pre-processor defines input & output
#undef INPUT
#undef OUTPUT
#undef LOW
#undef HIGH

#include <cstdint>

namespace Hal {
    enum class PinMode {
        INPUT = 0,
        OUTPUT = 1
    };

    enum class PinDigital {
        LOW = 0,
        HIGH = 1
    };

    void pinMode(uint8_t pin, PinMode mode);

    void digitalWrite(uint8_t pin, PinDigital val);

    int analogRead(uint8_t pin);
}

#endif