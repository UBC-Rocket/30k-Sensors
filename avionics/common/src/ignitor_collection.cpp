#include "ignitor_collection.h"
#include "gpio.h"
#include "log.hpp"

IgnitorCollection::IgnitorCollection()
    : drogue(Pin::DROGUE_IGNITOR, Pin::DROGUE_CONTINUITY_TEST,
             Pin::DROGUE_CONTINUITY_READ),
      main(Pin::MAIN_IGNITOR, Pin::MAIN_CONTINUITY_TEST,
           Pin::MAIN_CONTINUITY_READ) {
    status_ = RocketStatus::NOMINAL;

    if (main.getStatus() == ComponentStatus::FAILURE) {
        LOG_ERROR("Broken ignitor for main parachute");
        status_bitfield_[0] |= 0x80;
        status_ = RocketStatus::CRITICAL_FAILURE;
    }
    if (drogue.getStatus() == ComponentStatus::FAILURE) {
        LOG_ERROR("Broken ignitor for drogue parachute");
        status_bitfield_[0] |= 0x40;
        status_ = RocketStatus::CRITICAL_FAILURE;
    }
}
