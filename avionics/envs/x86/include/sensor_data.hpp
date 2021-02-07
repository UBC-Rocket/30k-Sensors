#pragma once

#include <cassert>
#include <fstream>
#include <limits> //for numeric_limits
#include <stdexcept>
#include <string>
#include <vector>

#include "HAL/time.h"
#include "stdio_controller.hpp"
#include "sensors_interface.h"

/**
 * Class for spoofing sensor data from SIM.
 * @tparam data_length the number of floats to request. Should match the sensor specification.
 */

template <std::size_t data_length> class DataSpoof {
  public:
    /**
     * @brief Constructor
     * @param sensor_id the sensor from which to read
     * @param extern_dat_buf external data buffer
     */
    DataSpoof(SensorType sensor_id, float *const extern_data_buff)
        : sensor_id_((uint8_t)sensor_id), dat_read_(extern_data_buff) {}

    /**
     * Request data from SIM. Mutates the dat_read_ array.
     * @return pointer to dat_read_
     */
    float *getData() {
        std::vector<float> sensorData =
            StdIoController::requestSensorRead(sensor_id_, data_length);
        for (std::size_t i = 0; i < sensorData.size(); i++) {
            dat_read_[i] = sensorData[i];
        }
        return dat_read_;
    }

    int getDataLength() { return data_length; }

  private:
    uint8_t sensor_id_;
    float *const dat_read_;
};

// Used for reading data values from a .csv file

// Inputted data to FileDataSpoof is in form of csv with first column
// being time in ms, and the other columns being each entry of data
// NOTE: The time in ms of the data must be aligned with the used time
template <std::size_t data_length> class FileDataSpoof {
  public:
    /**
     * @brief Constructor
     * @param dataFile Path to csv data file.
     * @param extern_dat_buf External data buffer.
     */
    FileDataSpoof(std::string const &dataFile, float *const extern_dat_buf)
        : dat_read(extern_dat_buf) {
        dataStream = std::ifstream(dataFile);

        if (!dataStream) {
            // std::exit(1);
            throw std::runtime_error("Data file could not be opened!");
        }
        // dlength = data_length;
        // data[dlength];
        // std::string dummy;
        // throw away first line of csv (headers)
        dataStream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    // returns next data point in stream
    float *getData() {
        constexpr std::streamsize MAX_BUFF = 255;
        auto t_now = Hal::now_ms().time_since_epoch().count();
        while (t1 < t_now) {
            // std::vector<std::string> dataFields = split_string(dataStr);

            // get line
            char dat_line[MAX_BUFF];
            // std::getline(dataStream, dat_line);
            dataStream.getline(dat_line, MAX_BUFF);
            // neat trick - converting all of the commas to '\0' means we now
            // get a set of c-style strings, all lined up
            char *data_strs[data_length + 1];
            data_strs[0] = dat_line;
            for (int i = 0, j = 0; dat_line[i] != '\0'; i++) {
                if (dat_line[i] == ',') {
                    dat_line[i] = '\0';
                    data_strs[++j] =
                        dat_line + i + 1; // pointer to element after comma
                }
            }

            t0 = t1;
            t1 = std::stol(data_strs[0]);
            assert(t1 - t0 < 200); // Spacing between data points can't be too
                                   // high otherwise sim becomes inaccurate

            for (std::size_t i = 0; i < data_length; i++) {
                dat_read[i] = std::stof(data_strs[i + 1]);
            }
        }

        return dat_read; // Not linearly interpolating b/c that is averaging,
                         // which reduces the noise level
    }

    int getDataLength() { return data_length; }

    ~FileDataSpoof() { dataStream.close(); }

  private:
    std::ifstream dataStream;
    long int t0 = 0; // two variables exist only for the purpose of assert
    long int t1 = 0;

    float *const dat_read;
};
