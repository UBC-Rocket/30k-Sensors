/* @file    sensors.cpp
 * @author  UBC Rocket Avionics 2018/2019
 * @description   Main sensor initialization and polling file.
 * Implements sensor initialization and error checking as well as
 * sensor polling, sensor logging, and sending data over the radio.
 *
 * @section LICENSE
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Distributed as-is; no warranty is given.
 */

/*Includes------------------------------------------------------------*/
#include "sensors.h"
#include "battery.h"
#include "SparkFun_LIS331.h"        //accelerometer
#include "MS5803_01.h"              //barometer
#include "SparkFunTMP102.h"         //temp sensor
#include "Adafruit_BNO055.h"        //IMU
#include "Venus638FLPx.h"           //GPS
#include "gpio.h"                   //GPIO

#include "commands.h"               //for sendRadioResponse(const char* response);
#include "cameras.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <i2c_t3.h>
#include <SD.h>

/*Variables------------------------------------------------------------*/
File datalog;

Battery powerbattery(BATTERY_SENSOR_PIN);
LIS331 accelerometer;
MS_5803 barometer(1024);
TMP102 temp_sensor(TEMP_SENSOR_ADDRESS);
Adafruit_BNO055 IMU(IMU_ADDRESS);


/*Functions------------------------------------------------------------*/
/**
  * @brief  Initializes all the sensors
  * @param  InitStatus *status - Status variable containing information on status.
  *                         Note: All values in *status will be overwritten; status
  *                         values can be then extracted.
  * @return void
  */

void initSensors(InitStatus *status)
{
    status->overview = NOMINAL;
    int i;
    for(i = 0; i < NUM_SENSORS; i++){       // + 1 for ematch continuity
        status->sensorNominal[i] = true;
    }

    /*init battery*/
    #ifdef TESTING
    SerialUSB.println("Initializing battery");
    #endif

    if(powerbattery.getVoltage() <= LOW_BATTERY_VOLTAGE)
    { //TODO: Uncomment once the battery sensor is implemented
        //status->sensorNominal[BATTERY_STATUS_POSITION] = false;
        // if(powerbattery.getVoltage() <= MINIMUM_BATTERY_VOLTAGE){
        //     status->overview = CRITICAL_FAILURE;
        //     #ifdef TESTING
        //     SerialUSB.println("DANGER: BATTERY AT UNACCEPTABLY LOW VOLTAGE!");
        //     #endif
        // }
        // else{
        //     if(status->overview < NONCRITICAL_FAILURE){
        //         status->overview = NONCRITICAL_FAILURE;
        //     }
        //     #ifdef TESTING
        //     SerialUSB.println("WARNING: Battery at low voltage!");
        //     #endif
        // }

    }

    #ifdef TESTING
    SerialUSB.print("Read voltage (V): ");
    SerialUSB.println(powerbattery.getVoltage());
    #endif

    /* check ematch continuity */
    #ifdef TESTING
    SerialUSB.println("Checking ematch continuity");
    #endif

    // if (!continuityCheck()){
    //     status->overview = CRITICAL_FAILURE;
    //     status->sensorNominal[EMATCH_STATUS_POSITION] = false;

    //     #ifdef TESTING
    //     SerialUSB.println("ERROR: ematch continuity");
    //     #endif
    // }

    /*init SD card*/
    #ifdef TESTING
    SerialUSB.println("Initializing SD card");
    #endif
    if (!SD.begin(BUILTIN_SDCARD)) {
        if(status->overview < NONCRITICAL_FAILURE)
            status->overview = NONCRITICAL_FAILURE;
        status->sensorNominal[FILE_STATUS_POSITION] = false;


        #ifdef TESTING
        SerialUSB.println("ERROR: SD card initialization failed!");
        #endif
    } else {
        datalog = SD.open("datalog.txt", FILE_WRITE);
        if (!datalog) {
            if(status->overview < NONCRITICAL_FAILURE)
                status->overview = NONCRITICAL_FAILURE;
            status->sensorNominal[FILE_STATUS_POSITION] = false;

            #ifdef TESTING
            SerialUSB.println("ERROR: Opening file failed!");
            #endif
        } else {
            datalog.write("SENSOR LOG DATA\n");
            datalog.write("Time (ms), State, Battery Voltage (V),Accelerometer - Acceleration X (g),Accelerometer - Acceleration Y (g),"
            "Accelerometer - Acceleration Z (g),Barometer - Pressure (mbar),Barometer - Temperature (C),"
            "Our - Baseline Pressure (mbar),Our - Altitude (m),Temperature Sensor - Temperature (C),"
            "IMU - Yaw (°),IMU - Roll (°),IMU - Pitch (°),GPS - latitude,GPS - longitude,GPS - altitude,ematch\n");
        }
    }

    /*init accerlerometer*/
    #ifdef TESTING
    SerialUSB.println("Initializing accelerometer");
    #endif
    accelerometer.setI2CAddr(ACCELEROMETER_ADDRESS);
    accelerometer.begin(LIS331::USE_I2C);

    /*init barometer*/
    #ifdef TESTING
    SerialUSB.println("Initializing barometer");
    #endif
    //barometer.reset();
    //barometer.begin();
    #ifdef TESTING
    if (!barometer.initializeMS_5803(true)) { //because one is verbose
        status->overview = CRITICAL_FAILURE;
        status->sensorNominal[BAROMETER_STATUS_POSITION] = false;
    }
    #else
    if (!barometer.initializeMS_5803(false)) {
        status->overview = CRITICAL_FAILURE;
        status->sensorNominal[BAROMETER_STATUS_POSITION] = false;
    }
    #endif

    /*init temp sensor*/
    #ifdef TESTING
    SerialUSB.println("Initializing temperature sensor");
    #endif
    temp_sensor.begin();

    /*init IMU*/
    #ifdef TESTING
    SerialUSB.println("Initializing IMU");
    #endif

    delay(7); //TODO investigate this
    if(!IMU.begin()){
        if(status->overview < NONCRITICAL_FAILURE)
            status->overview = NONCRITICAL_FAILURE;
        status->sensorNominal[IMU_STATUS_POSITION] = false;

        #ifdef TESTING
        SerialUSB.print("ERROR: IMU initialization failed!");
        #endif
    }
    IMU.setExtCrystalUse(true);

    /*init GPS*/
    #ifdef TESTING
    SerialUSB.println("Initializing GPS");
    #endif
    SerialGPS.begin(4800);
    while (!SerialGPS) {}
    SerialGPS.write(GPS_reset_defaults, sizeof(GPS_reset_defaults));
    // SerialGPS.write(GPS_set_baud_rate, sizeof(GPS_set_baud_rate));
    SerialGPS.write(GPS_set_NMEA_message, sizeof(GPS_set_NMEA_message));
    SerialGPS.write(GPS_set_update_rate, sizeof(GPS_set_update_rate));

    /*init radio*/
    #ifdef TESTING
    SerialUSB.println("Initializing radio");
    #endif
    SerialRadio.begin(921600);
    while (!SerialRadio) {}

    #ifdef TESTING
    SerialUSB.println("Initializing cameras");
    #endif
    SerialCamera1.begin(115200);
    SerialCamera2.begin(115200);
    while(!SerialCamera1 || !SerialCamera2){}

    /* log initialization status for each sensor */
    // 'X' for N/A, 'G' for good, 'B' for bad
    datalog.write("X,X,"); //time, state;

    if(status->sensorNominal[BATTERY_STATUS_POSITION])
        datalog.write("G,");
    else
        datalog.write("B,");

    if(status->sensorNominal[ACCELEROMETER_STATUS_POSITION])
        datalog.write("G,G,G,");
    else
        datalog.write("B,B,B,");

    if(status->sensorNominal[BAROMETER_STATUS_POSITION]) //barom pressure & temperature
        datalog.write("G,G,");
    else
        datalog.write("B,B,");

    datalog.write("X,X,"); //Baseline pressure & altitude

    if(status->sensorNominal[TEMPERATURE_STATUS_POSITION])
        datalog.write("G,");
    else
        datalog.write("B,");

    if(status->sensorNominal[IMU_STATUS_POSITION]) //Yaw, roll, pitch
        datalog.write("G,G,G,");
    else
        datalog.write("B,B,B,");

    datalog.write("X, X, X,");   //GPS - as of time of writing, no capability to test success

    if(status->sensorNominal[EMATCH_STATUS_POSITION])   //ematch continuity
        datalog.write("G\n");
    else
        datalog.write("B\n");


    /* transmit sensor report */
        // Key for receiver:
        // S-#-G/B - G/B - G/B
        // S is UID for status, # is digit indicating which part is being sent. G/B is status of a sensor
        // (good/bad), X indicates no sensor for that value yet.
        // Refer to sensors.h definitions to find what order data comes in at.
        // e.g. if all things succeeded except accelerometer, code sends:
        // "S-1-G-B-G", "S-2-G-G-G", "S-3-G-X-X"

    char statusReport1[RADIO_DATA_ARRAY_SIZE];
    char statusReport2[RADIO_DATA_ARRAY_SIZE];
    char statusReport3[RADIO_DATA_ARRAY_SIZE];

    generateStatusReport(status, statusReport1, statusReport2, statusReport3);
    sendRadioResponse(statusReport1);
    sendRadioResponse(statusReport2);
    sendRadioResponse(statusReport3);
    return;
}

/*
 * @brief  Generates status report for initialization. 'G' for good, 'B' for bad, 'X' for Not applicable
 * @param  InitStatus status - status of initialization.
 * @param  char* statusReport1 - char array to hold radio data
 * @param  char* statusReport2 - same as statusReport2 but it's the 2nd half
 * @return void
 */
void generateStatusReport(InitStatus *status, char *statusReport1, char *statusReport2, char *statusReport3)
{
    statusReport1[0] = UID_status;
    statusReport1[1] = '1';

    if(status->sensorNominal[FILE_STATUS_POSITION])
        statusReport1[2] = 'G';
    else
        statusReport1[2] = 'B';

    if(status->sensorNominal[BATTERY_STATUS_POSITION])
        statusReport1[3] = 'G';
    else
        statusReport1[3] = 'B';

    if(status->sensorNominal[ACCELEROMETER_STATUS_POSITION])
        statusReport1[4] = 'G';
    else
        statusReport1[4] = 'B';


    statusReport2[0] = UID_status;
    statusReport2[1] = '2';

    if(status->sensorNominal[BAROMETER_STATUS_POSITION])
        statusReport2[2] = 'G';
    else
        statusReport2[2] = 'B';

    if(status->sensorNominal[TEMPERATURE_STATUS_POSITION])
        statusReport2[3] = 'G';
    else
        statusReport2[3] = 'B';

    if(status->sensorNominal[IMU_STATUS_POSITION])
        statusReport2[4] = 'G';
    else
        statusReport2[4] = 'B';


    statusReport3[0] = UID_status;
    statusReport3[1] = '3';

    if(status->sensorNominal[EMATCH_STATUS_POSITION])
        statusReport3[2] = 'G';
    else
        statusReport3[2] = 'B';

    statusReport3[3] = 'X';
    statusReport3[4] = 'X';
}

/**
  * @brief  Retrieves one pressure value from the barometer.
  * @return float - pressure data point from the barometer.
  */
float barSensorInit(void){
    barometer.readSensor();
    return barometer.pressure();
}

/**
  * @brief  Polls all the sensors
  * @param  unsigned long timestamp - pointer to store the timestamp value
  * @param  float acc_data[] - array to store the accelerometer data
  * @param  float bar_data[] - array to store the barometer data
  * @param  float* temp_sensor_data - pointer to store the temperature sensor data
  * @param  float IMU_data[] - array to store the IMU data
  * @param  char GPS_data[][] - 2D array to store the GPS data
  * @return None
  */
void pollSensors(unsigned long *timestamp, float *battery_voltage, float acc_data[], float bar_data[],
                float *temp_sensor_data, float IMU_data[], float GPS_data[])
{
    int16_t x, y, z;

    *timestamp = millis();

    #ifdef TESTING
    SerialUSB.println("Measuring battery voltage");
    #endif
    *battery_voltage = powerbattery.getVoltage();

    #ifdef TESTING
    SerialUSB.println("Polling accelerometer");
    #endif
    accelerometer.readAxes(x, y, z);
    acc_data[0] = accelerometer.convertToG(ACCELEROMETER_SCALE, x);
    acc_data[1] = accelerometer.convertToG(ACCELEROMETER_SCALE, y);
    acc_data[2] = accelerometer.convertToG(ACCELEROMETER_SCALE, z);

    #ifdef TESTING
    SerialUSB.println("Polling barometer");
    #endif
    bool bar_flag = barometer.readSensor();

    #ifdef TESTING
    if(!bar_flag)
        SerialUSB.println("BAROMETER FAILED READING");
    #endif
    //bar_data[0] = barometer.getPressure(ADC_4096);
    //bar_data[1] = barometer.getTemperature(CELSIUS, ADC_512);
    bar_data[0] = barometer.pressure();
    bar_data[1] = barometer.temperature();


    #ifdef TESTING
    SerialUSB.println("Polling temperature sensor");
    #endif
    *temp_sensor_data = temp_sensor.readTempC();

    #ifdef TESTING
    SerialUSB.println("Polling IMU");
    #endif
    sensors_event_t event; //we don't know what this is but it works so
    IMU.getEvent(&event);
    IMU_data[0] = event.orientation.x;
    IMU_data[1] = event.orientation.y;
    IMU_data[2] = event.orientation.z;

    #ifdef TESTING
    SerialUSB.println("Polling GPS");
    #endif
    if (updateGPS()) {
        getGPS(GPS_data);
    }
}

/**
  * @brief  Logs all the sensor data
  * @param  unsigned long timestamp - pointer to store the timestamp value
  * @param  battery_voltage - pointer to store the voltage of the battery
  * @param  float acc_data[] - array to store the accelerometer data
  * @param  float bar_data[] - array to store the barometer data
  * @param  float* temp_sensor_data - pointer to store the temperature sensor data
  * @param  float IMU_data[] - array to store the IMU data
  * @param  char GPS_data[][] - 2D array to store the GPS data
  * @return None
  */
void logData(unsigned long *timestamp, float *battery_voltage, float acc_data[], float bar_data[],
            float *temp_sensor_data, float IMU_data[], float GPS_data[],
            FlightStates state, float altitude, float baseline_pressure)
{
    /*write data to SD card*/
    #ifdef TESTING
    SerialUSB.println("Writing to SD card");
    #endif
    datalog.print(*timestamp);
    datalog.print(",");
    datalog.print(state);
    datalog.print(",");
    datalog.print(*battery_voltage);
    datalog.print(",");
    for (unsigned int i = 0; i < ACC_DATA_ARRAY_SIZE; i++) {
       datalog.print(acc_data[i]);
       datalog.print(",");
    }
    for (unsigned int i = 0; i < BAR_DATA_ARRAY_SIZE; i++) {
        datalog.print(bar_data[i]);
        datalog.print(",");
    }
    datalog.print(baseline_pressure);
    datalog.print(",");
    datalog.print(altitude);
    datalog.print(",");
    datalog.print(*temp_sensor_data);
    datalog.print(",");
    for (unsigned int i = 0; i < IMU_DATA_ARRAY_SIZE; i++) {
        datalog.print(IMU_data[i]);
        datalog.print(",");
    }
    for (unsigned int i = 0; i < GPS_DATA_ARRAY_SIZE; i++) {
        datalog.print(GPS_data[i]);
        datalog.print(",");
    }
    datalog.print("\n");
    datalog.flush();

    /*output data to serial*/
    #ifdef TESTING
    SerialUSB.print("Time (ms):                          ");
    SerialUSB.println(*timestamp);
    SerialUSB.print("State:                              ");
    SerialUSB.println(state);
    SerialUSB.print("Battery voltage (V):                ");
    SerialUSB.println(*battery_voltage);
    SerialUSB.print("Accelerometer acceleration X (g):   ");
    SerialUSB.println(acc_data[0]);
    SerialUSB.print("Accelerometer acceleration Y (g):   ");
    SerialUSB.println(acc_data[1]);
    SerialUSB.print("Accelerometer acceleration Z (g):   ");
    SerialUSB.println(acc_data[2]);
    SerialUSB.print("Barometer pressure (mbar):          ");
    SerialUSB.println(bar_data[0]);
    SerialUSB.print("Barometer temperature (C):          ");
    SerialUSB.println(bar_data[1]);
    SerialUSB.print("Baseline Pressure (mbar):           ");
    SerialUSB.println(baseline_pressure);
    SerialUSB.print("Altitude (m):                       ");
    SerialUSB.println(altitude);
    SerialUSB.print("Temperature sensor temperature (C): ");
    SerialUSB.println(*temp_sensor_data);
    SerialUSB.print("IMU - Yaw:                          ");
    SerialUSB.println(IMU_data[0]);
    SerialUSB.print("IMU - Roll:                         ");
    SerialUSB.println(IMU_data[1]);
    SerialUSB.print("IMU - Pitch:                        ");
    SerialUSB.println(IMU_data[2]);
    SerialUSB.print("GPS latitude:                       ");
    SerialUSB.println(GPS_data[0]);
    SerialUSB.print("GPS longitude:                      ");
    SerialUSB.println(GPS_data[1]);
    SerialUSB.print("GPS altitude:                       ");
    SerialUSB.println(GPS_data[2]);
    SerialUSB.println("");
    #endif
}



