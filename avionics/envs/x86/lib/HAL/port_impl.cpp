#include "HAL/port_impl.h"

namespace Hal {
int Serial::read() {
    uint8_t c;
    if (StdIO::get(IO_ID_, c))
        return c;
    return -1;
}

std::size_t Serial::write(const uint8_t *const inbuf, std::size_t const size) {
    const std::lock_guard<std::mutex> lock(buf_mutex_);
    if (size + buf_used_ > BUFFER_SIZE) {
        send_buffer();
    }
    std::copy(inbuf, inbuf + size, buf_.begin());
    buf_sent_ = true;
    return size;
}
} // namespace Hal