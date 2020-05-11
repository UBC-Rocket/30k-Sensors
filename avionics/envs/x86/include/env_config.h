#ifndef ENV_CONFIG_H_A5EBD35B10114DF0B6FDCD24E3FE7DA8
#define ENV_CONFIG_H_A5EBD35B10114DF0B6FDCD24E3FE7DA8

// barometer data file
#include <string>
const std::string BAROMETER_DATA{"data/barometer.csv"};

#include "HAL/port_impl.h"
#include "stdio_controller.hpp"
StdIoController std_io_controller;
std::mutex StdIoController::s_cout;

#include "radio.h"
NativeRadioController radio(800, std_io_controller);

auto static SerialUSB = Hal::NativeSerial(1, std_io_controller);
auto static SerialGPS = Hal::NativeSerial(2, std_io_controller);
auto static SerialRadio = Hal::NativeSerial(3, std_io_controller);
auto static IridiumSerial = Hal::NativeSerial(4, std_io_controller);
auto static SerialCamera = Hal::NativeSerial(5, std_io_controller);

#include "CSVwrite.h"
class NativeDumbCSVImpl { // TODO - make this not full of no-ops
  public:
    bool init(char const *filename) { return true; }

    /**
     * @brief writes t to the next csv column in order
     * @param t the data to write
     */
    template <typename T> void print(T t) {}

    /**
     * @brief writes t and ends the line
     * @param t the data to write
     */
    template <typename T> void println(T t) {}

    /**
     * @brief flushes the cached data to the SD card
     */
    void flush() {}
};

constexpr char LOG_FILE_NAME[] = "datalog.csv";
CSVWrite<NativeDumbCSVImpl> datalog;

/* Buzzer */
#include "buzzer.h"
static NativeBuzzer buzzer;

void setup(void);
void loop(void);

#endif
