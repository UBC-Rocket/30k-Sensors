#ifndef BAROMETER_H
#define BAROMETER_H

/**
  * Barometer Sensor Class
  */

/*Includes------------------------------------------------------------*/
#include "sensors-interface.h"
#include "MS5803_01.h"              // Barometer

/*Constants------------------------------------------------------------*/
#define BAROMETER_STATUS_POSITION 3
#define BAROMETER_DATA_ARRAY_SIZE 2

class Barometer : public ISensor {
public:
SensorStatus initSensor();
SensorStatus readData(float* data);
uint8_t dataLength();

private:
MS_5803* barometer;
};

#endif