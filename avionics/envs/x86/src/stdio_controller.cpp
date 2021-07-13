#include <cassert>
#include <fstream>
#include <utility>

#include "stdio_controller.hpp"

std::thread StdIoController::input_{};
std::mutex StdIoController::istream_mutex_{};
std::unordered_map<StdIoController::Id, std::queue<uint8_t>>
    StdIoController::istreams_{};
std::mutex StdIoController::cout_mutex_{};
std::ofstream StdIoController::out_log_{
    "FW_SIM_log", std::ios_base::out | std::ios_base::binary};
std::atomic_bool StdIoController::run_input_{true};
std::mutex StdIoController::blocking_request_mutex_{};
std::condition_variable StdIoController::blocking_request_cv_{};

void StdIoController::initialize() {
    static bool done = false;
    if (done) { // idempotency
        return;
    }
    done = true;

    std::cout << "SYN" << std::flush;
    uint8_t ack[4] = "ACK";
    for (auto i = 0; i < 3; i++) {
        assert(ack[i] == std::cin.get());
    }
    putConfigPacket();
    std::thread input{&StdIoController::inputLoop};
    input_ = std::move(input);
}

void StdIoController::putPacket(uint8_t const id, uint8_t const *c,
                                uint16_t const length) {
    if (id == DEV_NULL) {
        return;
    }
    const std::lock_guard<std::mutex> lock(cout_mutex_);

    outputFiltered(id);                                // ID
    outputFiltered(static_cast<uint8_t>(length >> 8)); // Length, bigendian
    outputFiltered(static_cast<uint8_t>(length & 0xFF));

    // Data
    for (uint8_t const *end = c + length; c != end; c++)
        outputFiltered(*c);

    std::cout << std::flush;
    out_log_.flush();
}
void StdIoController::putPacket(uint8_t const id, char const *c,
                                uint16_t const length) {
    putPacket(id, reinterpret_cast<const uint8_t *>(c), length);
    // Shoudn't need to worry that std::cin et al thinks its pointing to a
    // char - the bits should come out as defined by uint8_t, which would be
    // expected. Char is defined to be a byte, so this is fine as long as a
    // byte is 8 bits (no seriously there are some machines that have bytes
    // with more than 8 bits.)
    // TODO - maybe less sloppily deal with this problem without using
    // reinterpret_cast? Although we're low enough that reinterpret is close
    // to being allowed
}

void StdIoController::outputFiltered(uint8_t const c) {
    // Sends each byte in two bytes to reduce ascii range to [A, A + 16)
    // Effectively avoiding all special characters that may have varying
    // behavior depending on the OS

    uint8_t msb = (c >> 4) + 'A';
    uint8_t lsb = (c & 0x0F) + 'A';

    output(msb);
    output(lsb);
}

void StdIoController::output(uint8_t const c) {
    std::cout.put(c);
    out_log_.put(c);
}
