#pragma once

#include <atomic>
#include <cstdint>
#include <cstring> //for memmove
#include <fstream>
#include <iostream> //for std::cin/cout
#include <mutex>
#include <queue>         // for queue
#include <thread>        // for non-blocking input
#include <type_traits>   // for is_same
#include <unordered_map> // for hash map
#include <vector>

#if defined(WIN32) || defined(_WIN32)
#define OS_IS_WINDOWS
#endif

class StdIoController {
    // Everything is static but it's not a namespace b/c of the private member
    // variables
  private:
    // static std::mutex istream_mutex_; // lock before accessing istreams_
    // static std::unordered_map<Id, std::queue<uint8_t>>
    //     istreams_; // Note - for input only

    // // I got tired of thinking about the best way to pre-compose strings so
    // // instead I'm just going to lock std::cout
    // static std::mutex cout_mutex_;

    // static std::thread input_;          // run infinite inputLoop()
    // static std::atomic_bool run_input_; // controls if input_ is running

    // static std::ofstream out_log_;

    static void outputFiltered(uint8_t const c);
    static void output(uint8_t const c);

    enum class PacketIds : uint8_t {
        analog_read = 0x61,
        sensor_read = 0x73,
        time_update = 0x74,
    };

    static constexpr auto FLOAT_SIZE = sizeof(float);
    static_assert(FLOAT_SIZE == 4,
                  "Code assumes floating point numbers are 4 bytes");

    class BlockingRequest {
        /**
         * @brief Helper class - constructor sends a packet request on
         *stream_id, and blocks until a response is heard. Destructor restarts
         *the input, so as long as your instance of this object is in scope, you
         *have clean, single-threaded access to the incoming data.
         **/
      public:
        BlockingRequest(const uint8_t stream_id, const uint8_t *const packet,
                        const std::size_t packet_len);

        // Can't copy.
        BlockingRequest(const BlockingRequest &) = delete;
        BlockingRequest &operator=(const BlockingRequest &other) = delete;

        ~BlockingRequest();
    };

  public:
    /**
     * @brief sets up SIM
     * Things that are setup: config packet is sent, with 2 way handshake.
     */
    static void initialize();

    constexpr static uint8_t DEV_NULL = 0;

    /**
     * @brief Attempts to extract a single character from the specified buffer
     * Id.
     * @param id ID being used (see documentation on stdio multiplexing spec:
     * http://confluence.ubcrocket.com/display/AV/Specs)
     * @param c If read succeeded, the result is written to c.
     * @return True if read succeeded, false if EOF.
     */
    static bool get(uint8_t id, uint8_t &c);

    static int available(uint8_t id);

    /**
     * @brief Basic level utility to put a packet into std::cout.
     * @param id ID being used
     * @param c Pointer to array of data to be sent
     * @param length Length of the data to be sent
     */
    static void putPacket(uint8_t const id, char const *c,
                          uint16_t const length);

    /**
     * @brief Basic level utility to put a packet into std::cout.
     * @param id ID being used
     * @param c Pointer to array of data to be sent
     * @param length Length of the data to be sent
     */
    static void putPacket(uint8_t const id, uint8_t const *c,
                          uint16_t const length);

    /**
     * @brief Corresponds to Request Analog Read packet in Confluence spec.
     * @param pin_id ID of the pin being read.
     * @return Read value.
     */
    static int requestAnalogRead(uint8_t const pin_id);

    /**
     * @brief Corresponds to Request Sensor Read packet in Confluence spec.
     * @param sensor_id ID of the desired sensor.
     * @param num_floats the number of floats to read.
     * @return vector of sensor measurements.
     */
    static std::vector<float> requestSensorRead(uint8_t sensor_id,
                                                std::size_t num_floats);

    /**
     * @brief Corresponds to Request Time Update package in confluence spec.
     * @param delta_us number of *microseconds* to shift clock foward by.
     * @return the new time in *milliseconds*.
     */
    static uint32_t requestTimeUpdate(uint32_t delta_us = 0);

  private:
    /**
     * @brief reinterprets an array of 4 chars to a floating point
     * number.
     * @param data: an array of 4 chars.
     * @return the floating point representation
     */
    static float charsToFloat(uint8_t data[4]);

    /**
     * @brief Helper function for configuration packet.
     */
    static void putConfigPacket();

    static uint8_t getCinForce();

    static uint8_t getFilteredCin();

    static void extractPacket();

    static void inputLoop();
};
