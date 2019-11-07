/*Includes------------------------------------------------------------*/
#include "MAX31855k.h"
#include "sensors/thermocouple.h"

/*Variables------------------------------------------------------------*/
#ifdef TESTING
        // Self powered with debug output
        MAX31855k probe(THERMO_SELECT_PIN, NONE, NONE, true);
    #else
        // Self powered with no debug messages
        MAX31855k probe(THERMO_SELECT_PIN);
    #endif  // TESTING

SensorStatus Thermocouple::initSensor() {
        #ifdef TESTING
            SerialUSB.println("Initializing thermocouple");
        #endif

    float thermo_temp = probe.readCJT();
    if (!isnan(thermo_temp)) {
        #ifdef TESTING
            SerialUSB.print("Cold Junction Temperature is [C]: ");
            SerialUSB.println(thermo_temp);
            SerialUSB.println("Thermocouple initialized");
        #endif
    }
    else{
        return SensorStatus::NONCRITICAL_FAILURE;
    }

    return SensorStatus::NOMINAL;
}

SensorStatus Thermocouple::readData(float* data) {
    #ifdef TESTING
            SerialUSB.println("Polling Thermocouple");
    #endif  // TESTING
    data[0] = probe.readTempC();
    #ifdef TESTING
        if (!isnan(*thermocouple_data)) {
            SerialUSB.print("Temp[C]=");
            SerialUSB.println(*thermocouple_data);
        }
        else
            SerialUSB.println("Thermocouple ERROR");
    #endif  // TESTING

    return SensorStatus::NOMINAL;
}

uint8_t Thermocouple::dataLength() {
    return THERMOCOUPLE_DATA_ARRAY_SIZE;
}