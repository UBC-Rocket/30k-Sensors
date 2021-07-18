#include <cassert>
#include <fstream>
#include <utility>

#include "stdio_controller.hpp"

// typedef for clarity
typedef uint8_t Id;

// istreams_ for input only
static std::unordered_map<Id, std::queue<uint8_t>> istreams_{};

// lock before accessing istreams_
static std::mutex istream_mutex_{};

// lock when accessing std::cout
static std::mutex cout_mutex_{};
static std::ofstream out_log_{"FW_SIM_log",
                              std::ios_base::out | std::ios_base::binary};

static std::thread input_{};              // run infinite inputLoop()
static std::atomic_bool run_input_{true}; // controls if input_ is running

StdIoController::BlockingRequest::BlockingRequest(
    const uint8_t stream_id, const uint8_t *const packet,
    const std::size_t packet_len) {
    // input_ will now run at most once more
    run_input_ = false;
    // ensures that at least one more SIM packet exists
    StdIoController::putPacket(stream_id, packet, packet_len);
    input_.join(); // so this won't block

    // Keep polling for new packet until response is received.
    while (istreams_[stream_id].size() == 0) {
        extractPacket();
    }
}

StdIoController::BlockingRequest::~BlockingRequest() {
    run_input_ = true;
    std::thread input{&StdIoController::inputLoop};
    input_ = std::move(input);
}
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

bool StdIoController::get(uint8_t id, uint8_t &c) {
    const std::lock_guard<std::mutex> lock(istream_mutex_);

    if (istreams_[id].size() == 0) {
        return false;
    }
    c = istreams_[id].front();
    istreams_[id].pop();
    return true;
}

int StdIoController::available(uint8_t id) {
    const std::lock_guard<std::mutex> lock(istream_mutex_);
    return istreams_[id].size();
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

int StdIoController::requestAnalogRead(uint8_t const pin_id) {
    uint8_t const PACKET_ID = static_cast<uint8_t>(PacketIds::analog_read);
    BlockingRequest restart(PACKET_ID, &pin_id, 1);

    int val = istreams_[PACKET_ID].front();
    istreams_[PACKET_ID].pop();
    val *= 256;
    val += istreams_[PACKET_ID].front();
    istreams_[PACKET_ID].pop();

    return val;
}

std::vector<float> StdIoController::requestSensorRead(uint8_t sensor_id,
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

uint32_t StdIoController::requestTimeUpdate(uint32_t delta_us) {
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

float StdIoController::charsToFloat(uint8_t data[4]) {
    // NOTE Ground station accounts for endianness.
    float f;
    std::memcpy(&f, data, FLOAT_SIZE);
    return f;
}

void StdIoController::putConfigPacket() {
    uint8_t id = 0x01;
    uint32_t int_test = 0x04030201;
    float float_test = -2.0; // 0xC000 0000;

    uint8_t buf[8];
    std::memmove(buf, &int_test, 4);
    std::memmove(buf + 4, &float_test, 4);
    putPacket(id, buf, 8);
}

uint8_t StdIoController::getCinForce() {
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

uint8_t StdIoController::getFilteredCin() {
    uint8_t msb = getCinForce() - 'A';
    uint8_t lsb = getCinForce() - 'A';

    return (msb << 4) | lsb;
}

void StdIoController::extractPacket() {
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

void StdIoController::inputLoop() {
    while (run_input_) {
        extractPacket();
    }
}
