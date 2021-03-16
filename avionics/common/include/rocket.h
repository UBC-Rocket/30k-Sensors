/**
 * Overarching object that carries different components.
 */
#pragma once

#include "CSVWriteImpl.h"
#include "CSVwrite.h"
#include "buzzer.h"
#include "calculations.h"
#include "cameras.h"
#include "radio.h"
#include "sensor_collection.h"
#include "state_machine.hpp"
#include "states_config.hpp"

#include "HAL/time.h"

struct Rocket {
  public:
    static constexpr char LOG_FILE_NAME[] = "datalog.csv";

    Rocket()
        : cam(Hal::SerialInst::Camera), datalog(LOG_FILE_NAME),
          init_status(collectStatus(sensors, ignitors)),
          calc(sensors, Hal::now_ms()), config(calc, ignitors, cam),
          state_machine(config.state_map, config.initial_state) {}

    // WARNING - MEMBER ORDER DEPENDENCY
    // https://isocpp.org/wiki/faq/ctors#order-dependency-in-members
    Buzzer buzzer;
    Camera cam;
    SensorCollection sensors;
    IgnitorCollection ignitors;
    CSVWrite<CSVWriteImpl> datalog;
    RocketStatus init_status;
    Calculator calc;
    StateMachineConfig config;
    StateMachine state_machine;
};
class CommandReceiver {
  private:
    Rocket &rocket_;

  public:
    CommandReceiver(Rocket &rocket) : rocket_(rocket) {}

    void run_command(Radio::command_t command) {
        // Sensor-related data should inform based on the last sensor poll time
        uint32_t sensor_poll_time =
            Hal::tpoint_to_uint(rocket_.sensors.last_poll_time());

        auto state = rocket_.state_machine.getState();
        switch (command) {
        case 'P':
            Radio::sendStatus(Hal::millis(), rocket_.init_status,
                              rocket_.sensors, rocket_.ignitors);
            break;
        case 'A':
            rocket_.state_machine.arm();
            rocket_.cam.start_record();
            break;

        case 'D':
            rocket_.state_machine.disarm();
            rocket_.cam.stop_record();
            break;
        case 0x30:
            Radio::sendBulkSensor(
                sensor_poll_time, rocket_.calc.altitude(),
                rocket_.sensors.accelerometer, rocket_.sensors.imuSensor,
                rocket_.sensors.gps, static_cast<uint16_t>(state));
            break;
        case 0x04:
            Radio::sendGPS(sensor_poll_time, rocket_.sensors.gps);
            break;
        case 0x10:
            Radio::sendSingleSensor(sensor_poll_time, 0x10,
                                    rocket_.sensors.accelerometer.getData()[0]);
            break;

        case 0x11:
            Radio::sendSingleSensor(sensor_poll_time, 0x11,
                                    rocket_.sensors.accelerometer.getData()[1]);
            break;
        case 0x12:
            Radio::sendSingleSensor(sensor_poll_time, 0x12,
                                    rocket_.sensors.accelerometer.getData()[2]);
            break;
        case 0x13:
            Radio::sendSingleSensor(sensor_poll_time, 0x13,
                                    rocket_.sensors.barometer.getData()[0]);
            break;
        case 0x14:
            Radio::sendSingleSensor(sensor_poll_time, 0x14,
                                    rocket_.sensors.barometer.getData()[1]);
            break;
        case 0x15:
            Radio::sendSingleSensor(sensor_poll_time, 0x15,
                                    rocket_.sensors.temperature.getData()[0]);
            break;
        case 0x16:
            break;
            // Not implemented - still uncertain about IMU storage
        case 0x17:
            break;
            // Not implemented - still uncertain about IMU storage
        case 0x18:
            break;
            // Not implemented - still uncertain about IMU storage
        case 0x19:
            Radio::sendSingleSensor(sensor_poll_time, 0x19,
                                    rocket_.sensors.gps.getData()[0]);
            break;
        case 0x1A:
            Radio::sendSingleSensor(sensor_poll_time, 0x1A,
                                    rocket_.sensors.gps.getData()[1]);
            break;
        case 0x1B:
            Radio::sendSingleSensor(sensor_poll_time, 0x1B,
                                    rocket_.sensors.gps.getData()[2]);
            break;
        case 0x1C:
            Radio::sendSingleSensor(sensor_poll_time, 0x1C,
                                    rocket_.calc.altitude());
            break;
        case 0x1D:
            Radio::sendState(sensor_poll_time, static_cast<uint16_t>(state));
            break;
        case 0x1E:
            break; // Voltage sensor not implemented
        case 0x1F:
            Radio::sendSingleSensor(sensor_poll_time, 0x1F,
                                    rocket_.calc.altitudeBase());
            break;
        default:
            // TODO: Log invalid command
            break;
        }
    }
};
