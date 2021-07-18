#include "gtest/gtest.h"

#include "HAL/pin_util.h"
#include "log.hpp"
#include "radio.h"

// Mocking StdIoController
void StdIoController::putPacket(unsigned char, unsigned char const *,
                                unsigned short) {}

std::vector<float> StdIoController::requestSensorRead(uint8_t,
                                                      std::size_t num_floats) {
    return std::vector<float>(num_floats, 0);
}

int StdIoController::requestAnalogRead(uint8_t) { return 0; }
int StdIoController::available(uint8_t) { return 0; }
bool StdIoController::get(uint8_t, uint8_t &) { return false; }

// Mocking HAL time support
namespace Hal {
void sleep_us(uint32_t) {}
void sleep_ms(uint32_t) {}

t_point now_ms() { return t_point(); }

uint32_t millis() { return 0; }

uint32_t tpoint_to_uint(t_point timestamp) {
    return static_cast<uint32_t>(timestamp.time_since_epoch().count());
}
} // namespace Hal

TEST(Radio, AllPacketTypes) {
    // There aren't any EXPECT_ / ASSERT_ test macros here; instead, the goal of
    // this test is to exercise all of the methods below, which contain internal
    // assertions.

    std::ofstream output_file;
    rktlog::OstreamOutput output_stream(output_file);

    {
        std::string logfile_name("unit_Radio_debuglog_");
        char datetime_buf[50];
        std::time_t time = std::time(nullptr);
        std::strftime(datetime_buf, 50, "%Y-%m-%dT%H-%M-%SZ",
                      std::localtime(&time));
        logfile_name += datetime_buf;
        output_file.open(logfile_name, std::ios_base::out | std::ios_base::app);
    }

    rktlog::Logger::setOutput(output_stream);

    uint32_t time = 0;
    SensorCollection sensors;
    IgnitorCollection ignitors;

    // Run a few times to ensure it works consistently
    for (int i = 0; i < 4; ++i) {
        Radio::sendStatus(time, RocketStatus::NOMINAL, sensors, ignitors);
        float alt = 100;
        uint16_t state_id = 0;
        Radio::sendBulkSensor(time, alt, sensors.accelerometer,
                              sensors.imuSensor, sensors.gps, state_id);
        Radio::sendMessage(time, "hello");
        Radio::sendGPS(time, sensors.gps);
        Radio::sendSingleSensor(time, 0, 1.23);
        Radio::sendState(time, state_id);
        Radio::sendConfig(time);
        Radio::sendEvent(time, EventId::IGNITOR_FIRED);
    }
}
