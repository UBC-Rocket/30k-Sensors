/*Includes------------------------------------------------------------*/
#include <HAL/gpio.h>
#include <HAL/time.h>

#include "hardware/ignitor.h"

Ignitor::Ignitor(uint8_t ignitePin, uint8_t continuityPin,
                 uint8_t continuityADCPin)
    : ignitePin_(ignitePin), continuityPin_(continuityPin),
      continuityADCPin_(continuityADCPin) {
    /*init ignitor*/
    Hal::pinMode(ignitePin_, Hal::PinMode::OUTPUT);
    Hal::digitalWrite(ignitePin_, Hal::PinDigital::LOW);

    /*continuity check */
    Hal::pinMode(continuityPin_, Hal::PinMode::OUTPUT);

    Hal::digitalWrite(continuityPin_, Hal::PinDigital::HIGH);
    Hal::sleep_us(CONTINUITY_CHECK_DELAY);

    int continuity = Hal::analogRead(continuityADCPin);
    Hal::digitalWrite(continuityPin_, Hal::PinDigital::LOW);

    if (continuity <= DISCONTINUOUS_THRESHOLD) {
        status = HardwareStatus::FAILURE;
    } else {
        status = HardwareStatus::NOMINAL;
    }
}

void Ignitor::fire() {
    Hal::digitalWrite(ignitePin_, Hal::PinDigital::HIGH);
    Hal::sleep_ms(IGNITOR_DELAY);
    Hal::digitalWrite(ignitePin_, Hal::PinDigital::LOW);
}
