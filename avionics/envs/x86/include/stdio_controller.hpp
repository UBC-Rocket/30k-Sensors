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
#include <condition_variable>

#if defined(WIN32) || defined(_WIN32)
#define OS_IS_WINDOWS
#endif

class StdIoController {
    // Everything is static but it's not a namespace b/c of the private member
    // variables
  private:
    typedef uint8_t Id;

    // static std::unordered_map<Id, std::mutex> m_mutexes;
    static std::mutex istream_mutex_; // lock before accessing istreams_
    static std::unordered_map<Id, std::queue<uint8_t>>
        istreams_; // Note - for input only

    // I got tired of thinking about the best way to pre-compose strings so
    // instead I'm just going to lock std::cout
    static std::mutex cout_mutex_;

    static std::thread input_;          // run infinite inputLoop()
    static std::atomic_bool run_input_; // controls if input_ is running

    static std::ofstream out_log_;

    static void outputFiltered(uint8_t const c);
    static void output(uint8_t const c);

    static std::mutex blocking_request_mutex_;
    static std::condition_variable blocking_request_cv_;

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
                        const std::size_t packet_len) {
            std::unique_lock<std::mutex> lock(blocking_request_mutex_);
            // input_ will now run at most once more
            StdIoController::run_input_ = false;
            // ensures that at least one more SIM packet exists
            StdIoController::putPacket(stream_id, packet, packet_len);
            blocking_request_cv_.wait(lock);

            // Keep polling for new packet until response is received.
            while (istreams_[stream_id].size() == 0) {
                extractPacket();
            }
        }

        // Can't copy.
        BlockingRequest(const BlockingRequest &) = delete;
        BlockingRequest &operator=(const BlockingRequest &other) = delete;

        ~BlockingRequest() {
            StdIoController::run_input_ = true;
            std::unique_lock<std::mutex> lock(blocking_request_mutex_);
            StdIoController::blocking_request_cv_.notify_one();
        }
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
    static bool get(uint8_t id, uint8_t &c) {
        const std::lock_guard<std::mutex> lock(istream_mutex_);

        if (istreams_[id].size() == 0) {
            return false;
        }
        c = istreams_[id].front();
        istreams_[id].pop();
        return true;
    }

    static int available(uint8_t id) {
        const std::lock_guard<std::mutex> lock(istream_mutex_);
        return istreams_[id].size();
    }

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
    static int requestAnalogRead(uint8_t const pin_id) {
        uint8_t const PACKET_ID = static_cast<uint8_t>(PacketIds::analog_read);
        BlockingRequest restart(PACKET_ID, &pin_id, 1);

        int val = istreams_[PACKET_ID].front();
        istreams_[PACKET_ID].pop();
        val *= 256;
        val += istreams_[PACKET_ID].front();
        istreams_[PACKET_ID].pop();

        return val;
    }

    /**
     * @brief Corresponds to Request Sensor Read packet in Confluence spec.
     * @param sensor_id ID of the desired sensor.
     * @param num_floats the number of floats to read.
     * @return vector of sensor measurements.
     */
    static std::vector<float> requestSensorRead(uint8_t sensor_id,
                                                std::size_t num_floats) {
        uint8_t const PACKET_ID = static_cast<uint8_t>(PacketIds::sensor_read);
        BlockingRequest restart(PACKET_ID, &sensor_id, 1);

        std::vector<float> sensor_data;
        for (std::size_t f = 0; f < num_floats; f++) {
            uint8_t packetData[FLOAT_SIZE];
            for (std::size_t i = 0; i < FLOAT_SIZE; i++) {
                packetData[i] = istreams_[PACKET_ID].front();
                istreams_[PACKET_ID].pop();
            }
            float float_val = charsToFloat(packetData);
            sensor_data.push_back(float_val);
        }

        return sensor_data;
    }

    /**
     * @brief Corresponds to Request Time Update package in confluence spec.
     * @param delta_us number of *microseconds* to shift clock foward by.
     * @return the new time in *milliseconds*.
     */
    static uint32_t requestTimeUpdate(uint32_t delta_us = 0) {
        auto const PACKET_ID = static_cast<uint8_t>(PacketIds::time_update);
        uint8_t chars[sizeof(uint32_t)];
        std::memcpy(chars, &delta_us, sizeof(delta_us));
        BlockingRequest restart(PACKET_ID, chars, sizeof(delta_us));

        uint8_t packetData[sizeof(uint32_t)];
        for (std::size_t i = 0; i < sizeof(uint32_t); i++) {
            packetData[i] = istreams_[PACKET_ID].front();
            istreams_[PACKET_ID].pop();
        }

        uint32_t currentTime;
        std::memcpy(&currentTime, packetData, sizeof(uint32_t));
        return currentTime;
    }

  private:
    /**
     * @brief reinterprets an array of 4 chars to a floating point
     * number.
     * @param data: an array of 4 chars.
     * @return the floating point representation
     */
    static float charsToFloat(uint8_t data[4]) {
        // NOTE Ground station accounts for endianness.
        float f;
        std::memcpy(&f, data, FLOAT_SIZE);
        return f;
    }

    /**
     * @brief Helper function for configuration packet.
     */
    static void putConfigPacket() {
        uint8_t id = 0x01;
        uint32_t int_test = 0x04030201;
        float float_test = -2.0; // 0xC000 0000;

        uint8_t buf[8];
        std::memmove(buf, &int_test, 4);
        std::memmove(buf + 4, &float_test, 4);
        putPacket(id, buf, 8);
    }

    static uint8_t getCinForce() {
        uint8_t c;
        while (true) {
            c = std::cin.get();
            if (std::cin.fail()) {
                std::cin.clear();
                continue;
            }
            return c;
        }
    }

    static uint8_t getFilteredCin() {
        uint8_t msb = getCinForce() - 'A';
        uint8_t lsb = getCinForce() - 'A';

        return (msb << 4) | lsb;
    }

    static void extractPacket() {
        Id id;
        uint16_t length;
        id = getFilteredCin();
        length = getFilteredCin();
        length <<= 8;
        length |= getFilteredCin();

        auto buf = std::vector<uint8_t>();
        buf.reserve(length);
        for (int i = 0; i < length; i++) {
            buf.push_back(getFilteredCin());
        }

        { // scope for lock-guard
            const std::lock_guard<std::mutex> lock(istream_mutex_);
            for (auto j : buf) {
                istreams_[id].push(j);
            } // unlock mutex
        }
    }

    static void inputLoop() {
        while (true) {
            while (run_input_) {
                extractPacket();
            }
            std::unique_lock<std::mutex> lock (blocking_request_mutex_);
            blocking_request_cv_.notify_one();
            blocking_request_cv_.wait(lock);
        }
    }
};
