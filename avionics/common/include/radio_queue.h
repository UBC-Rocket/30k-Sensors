#pragma once

#include <deque>  // for std::deque
#include <memory> // for std::unique_ptr
#include <vector> // for std::vector

#include "XBee.h"
#include "subpktptr.h"

/*Constants------------------------------------------------------------*/

/*  Many parts of the XBee library use uint8 for length (e.g. getFrameData).
    Even though the HW likely supports more, this is the theoretical max for
    this XBee library implementation. If we deem this was an implementation
    error, we could PR a fix to the library and then increase this */
constexpr unsigned short RADIO_MAX_SUBPACKET_SIZE = 255 - ZB_TX_API_LENGTH;

/*Classes--------------------------------------------------------------*/

class RadioQueue {
  public:
    RadioQueue(unsigned short const MAX_BYTES) : M_MAX_BYTES(MAX_BYTES) {}

    /**
     * @brief Push a subpacket to the queue. If the pointer is a null pointer or
     * the size is too large, it will silently fail.
     * @param subpacket SubPktPtr to data.
     * */
    void push(SubPktPtr subpacket);

    /**
     * @brief Returns the next SubPktPtr in the queue (i.e. the oldest element),
     * popping that element. If SubPktPtr is empty, behaviour is undefined.
     */
    SubPktPtr popFront();

    /**
     * @brief Test whether container is empty.
     */
    bool empty() const { return m_subpacket_q.empty(); }

    /**
     * @brief Get number of bytes of subpacket data queued up. Note that this is
     * not the same as the number of subpackets that are queued.
     * @return Number of bytes of data.
     * */
    unsigned short getByteCount() const { return m_byte_count; }

    /**
     * @brief Fills payload pointer as full as possible (i.e. at most
     * RADIO_MAX_SUBPACKET_SIZE bytes)
     * @param payload Pointer to payload array.
     * @return Size of payload data filled in.
     */
    uint8_t fillPayload(uint8_t *payload);

  private:
    std::deque<SubPktPtr>
        m_subpacket_q; // subpacket_queue, but the ueue is silent
    unsigned short m_byte_count = 0;
    unsigned short const M_MAX_BYTES;

    /**
     * @brief Dump entire queue into payload. Must check that payload is large
     * enough and queue is small enough that this works (i.e. m_byte_count <
     * payload array size); otherwise this is undefined behaviour
     * @return Size of payload data filled in (= m_byte_count before this
     * starts).
     */
    uint8_t dumpAllIntoPayload(uint8_t *payload);

    /**
     * @brief Dispenses at most RADIO_MAX_SUBPACKET_SIZE (255) bytes worth of
     * subpackets into payload.
     * @return Size of payload data filled in.
     */
    uint8_t dispenseIntoPayload(uint8_t *payload);
};
