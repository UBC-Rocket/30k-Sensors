#pragma once

#undef abs // TODO This macro gets defined somewhere with Teensy and I'm not
           // sure where or whether there's a better way around it

#include <cmath> //std::abs

#include <Eigen/Geometry> //quaternions

#include "CalcHelpers/barometric.h"
#include "CalcHelpers/exponential_moving_avg.h"
#include "calculations_interface.h"
#include "sensor_collection.h"
#include "sensors/IMU.h"
#include "sensors/barometer.h"
#include "state_input_struct.h"

// following recomendation for alpha = 2/(2N + 1)
constexpr float BAROMETER_MOVING_AVERAGE_ALPHA = 2.0f / (2 * 40 + 1);

class Calculator : public ICalculator {
  public:
    Calculator(SensorCollection &sensors)
        : m_baro(sensors.barometer), m_imu(sensors.imuSensor) {
        m_baro.readData();
        m_base_alt.reset(pressureToAltitude(*(m_baro.getData())), 1,
                         BAROMETER_MOVING_AVERAGE_ALPHA);
        m_last_t = Hal::now_ms();
        m_last_alt = m_base_alt.getAverage();
    }

    void calculateValues(StateId const state, StateInput &out_state_input,
                         Hal::t_point t_ms) {
        StateInput &out = out_state_input; // alias

        /* Update altitude average and calculate altitude*/
        const float alt = pressureToAltitude(*(m_baro.getData()));
        const float diff = std::abs(m_base_alt.getAverage() - alt);

        if ((state == StateId::STANDBY || state == StateId::ARMED) &&
            diff < m_base_alt.getStandardDeviation() * 3) {
            m_base_alt.addValue(alt);
        }
        out.altitude = alt - m_base_alt.getAverage();

        /* Do some math to figure out orientation & acceleration in relevant
         * frames*/
        // TODO - check state to only do these calculations when necessary
        // There may be some really clever way to do this with pointers, or at
        // least use comma initialization
        out.accel_rocket(0) =
            m_imu.getData()[0]; //? TODO - make sure these are aligned
        out.accel_rocket(1) = m_imu.getData()[1]; //?
        out.accel_rocket(2) = m_imu.getData()[2]; //?

        Eigen::Quaternionf meas_orient(1, 0, 0, 0);
        // TODO - initialize measured orientation with m_imu's quaternion data
        // instead of dummy data

        // conjugate is equal to the inverse for unit quaternion
        out.orientation = meas_orient * m_gnd_quat.conjugate();
        out.orientation.normalize();

        // convert into quaternion for operation
        const Eigen::Quaternionf xl_rocket_quat(
            0, out.accel_rocket(0), out.accel_rocket(1),
            out.accel_rocket(2)); // There may be some really clever way to do
                                  // this with more pointers

        // conjugate to rotate
        const Eigen::Quaternionf xl_gnd_quat =
            out.orientation.conjugate() * xl_rocket_quat * out.orientation;

        // turn back into vector
        out.accel_ground << xl_gnd_quat.x(), xl_gnd_quat.y(), xl_gnd_quat.z();

        /* Do velocity calculations */
        constexpr int MILLISECONDS_PER_SECOND = 1000;
        out.velocity_vertical =
            (out.altitude - m_last_alt) / (t_ms - m_last_t).count() *
            MILLISECONDS_PER_SECOND; // t_ms and m_last_t are of millisconds

        m_last_alt = out.altitude; // in preparation for next loop
        m_last_t = t_ms;
    }

  private:
    ExponentialMovingAvg<float> m_base_alt;
    Barometer &m_baro;
    IMU &m_imu;

    Eigen::Quaternionf m_gnd_quat;

    float m_last_alt;
    Hal::t_point m_last_t;
};
