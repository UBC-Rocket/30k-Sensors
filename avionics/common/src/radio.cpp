/*
 * Radio Source
 *
 * @file    radio.cpp
 * @author  UBC Rocket Avionics 2018/2019
 * @description  Implements radio send functions and prioritizing
 * sensor data to send over the radio.  This source utilizes the
 * XBee.cpp and .h files also included in this distro.
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
#include <algorithm> //for std::copy
#include <cassert>
#include <cstring>
#include <utility> //for std::move

#include "HAL/port_impl.h"

#include "XBee.h"
#include "ignitor_collection.h"
#include "radio.h"
#include "roar/buffer.hpp"
#include "sensor_collection.h"
#include "state_id_enum.hpp"

// Some defines that are actually done in CMake, but set here to get
// Intellisense to stop yelling.

#ifndef RADIO_CONFIG_PACKET_SIM_ACTIVE
#define RADIO_CONFIG_PACKET_SIM_ACTIVE 2
#endif

#ifndef RADIO_CONFIG_PACKET_ROCKET_ID
#define RADIO_CONFIG_PACKET_ROCKET_ID 255
#endif

#ifndef RADIO_CONFIG_PACKET_VERSION_STR
#define RADIO_CONFIG_PACKET_VERSION_STR                                        \
    "Fake version string; quiet Intellisense "
#endif

namespace {
constexpr uint32_t kDefaultFlaregunAddrMsb = 0x0013A200;
constexpr uint32_t kDefaultFlaregunAddrLsb = 0x41678FC0;
constexpr uint32_t kRadioBaudRate = 921600;
constexpr uint8_t kMaxPacketsPerRxLoop = 8;
constexpr int kMaxQueuedBytes = 800;
// 20 bytes per subpkt on average
constexpr int kMaxQueuedSubpkts = kMaxQueuedBytes / 20;

} // namespace

/*  Many parts of the XBee library use uint8 for length (e.g. getFrameData).
    Even though the HW likely supports more, this is the theoretical max for
    this XBee library implementation.

    TODO: If we deem this was an implementation error, we could PR a fix to the
    library and then increase this */
constexpr unsigned short kPacketPayloadSpace = 255 - ZB_TX_API_LENGTH;

/**
 * The purpose of RadioMembers is to have a set of static member variables for
 * Radio, that are not visible from the interface (.h file), and also
 * maintaining a mechanism similar to constructors, in that there is a defined
 * set of operations that occurs automatically. Specifically, the instance
 * `self` is this set of member variables.
 *
 * The challenge is that some private methods of Radio are used; as such
 * RadioMembers must be a friend of Radio. In the interest of not polluting the
 * global namespace, the type is nested. To further enforce access control, only
 * Radio is permitted to access private members, which are the only useful
 * component.
 *
 * In order to actually make RadioMembers class accessible from the outside,
 * it's listed as a public type under the Radio class definition, but no
 * definition is given for RadioMembers so it's not actually usable (outside of
 * this translation unit). Technically you might be able to provide a little
 * more protection by e.g. marking the type as protected then extracting it by
 * inheriting from Radio, and there might be other tricks you could pull, but I
 * couldn't think of any / was too lazy to try them.
 */
class Radio::RadioMembers {
    friend class Radio;

  public:
    RadioMembers()
        : tx_q_(kPacketPayloadSpace, kMaxQueuedBytes, kMaxQueuedSubpkts),
          tx_packet_(
              XBeeAddress64(kDefaultFlaregunAddrMsb, kDefaultFlaregunAddrLsb),
              payload_, kPacketPayloadSpace) {}

  private:
    roar::Buffer tx_q_; // the [ueue] is silent :)
    uint8_t payload_[kPacketPayloadSpace];

    XBee xbee_;
    ZBTxRequest tx_packet_;
    ZBRxResponse rx;
};
static Radio::RadioMembers self;

constexpr Hal::ms Radio::WATCHDOG_SEND_INTERVAL;
bool Radio::can_send_ = true;

void Radio::initialize() {
    auto &serial = Hal::SerialInst::Radio;
    serial.begin(kRadioBaudRate);
    while (!serial)
        ;
    self.xbee_.setSerial(serial.getSerial());

    Radio::sendMessage(Hal::millis(), "Radio initialized");
    Radio::send();
}

void Radio::addIdTime(Ids id, uint32_t time) {
    self.tx_q_.write(static_cast<uint8_t>(id));
    self.tx_q_.write((&time), sizeof(time));
}

void Radio::send() {
    if (!self.tx_q_.empty()) {
        self.tx_packet_.setPayloadLength(self.tx_q_.fillPayload(self.payload_));
        self.xbee_.send(self.tx_packet_);
    }
}

void Radio::sendStatus(uint32_t time, RocketStatus status,
                       SensorCollection &sensors, IgnitorCollection &ignitors) {
    self.tx_q_.allocSubpkt(10);
    addIdTime(Ids::status_ping, time);

    self.tx_q_.write(static_cast<uint8_t>(status));
    self.tx_q_.write(sensors.getStatusBitfield(), 2);
    self.tx_q_.write(ignitors.getStatusBitfield(), 2);
}

void Radio::sendBulkSensor(uint32_t time, float alt, Accelerometer &xl,
                           IMU &imu, GPS &gps, uint16_t state_id) {
    self.tx_q_.allocSubpkt(43);
    addIdTime(Ids::bulk_sensor, time);

    // Altitude
    self.tx_q_.write(&alt, 4);

    // Accelerometer
    self.tx_q_.write(xl.getData(), 12);

    // IMU // TODO - check that these are the correct 3 floats to send for
    // orientation
    self.tx_q_.write(imu.getData(), 12);

    // GPS
    self.tx_q_.write(gps.getData(), 8);

    // State
    self.tx_q_.write(&state_id, sizeof(uint16_t));
}

void Radio::sendMessage(const uint32_t time, const char *str) {
    auto strlen = std::strlen(str);
    assert(strlen + 5 <= kPacketPayloadSpace);

    self.tx_q_.allocSubpkt(6 + strlen);
    addIdTime(Ids::message, time);

    self.tx_q_.write(strlen);
    self.tx_q_.write(str, strlen);
}

void Radio::sendGPS(const uint32_t time, GPS &gps) {
    self.tx_q_.allocSubpkt(17);

    addIdTime(Ids::gps, time);
    self.tx_q_.write(gps.getData(), 12);
}

void Radio::sendSingleSensor(const uint32_t time, uint8_t id, float data) {
    self.tx_q_.allocSubpkt(9);

    self.tx_q_.write(id);
    self.tx_q_.write(&time, sizeof(time));
    self.tx_q_.write(&data, 4);
}

void Radio::sendState(const uint32_t time, uint16_t state_id) {
    self.tx_q_.allocSubpkt(7);

    addIdTime(Ids::state, time);
    self.tx_q_.write(&state_id, sizeof(state_id));
}

void Radio::sendConfig(const uint32_t time) {
    self.tx_q_.allocSubpkt(47);
    addIdTime(Ids::config, time);

    // Defined in CMakeLists/platformio.ini
    self.tx_q_.write(RADIO_CONFIG_PACKET_SIM_ACTIVE);
    self.tx_q_.write(RADIO_CONFIG_PACKET_ROCKET_ID);

    // -1 because null terminated string
    constexpr size_t len = sizeof(RADIO_CONFIG_PACKET_VERSION_STR) - 1;
    static_assert(len == 40, "RADIO_CONFIG_PACKET_VERSION_STR incorrect size!");

    self.tx_q_.write(RADIO_CONFIG_PACKET_VERSION_STR, len);
}

void Radio::sendEvent(const uint32_t time, const EventId event) {
    self.tx_q_.allocSubpkt(7);

    addIdTime(Ids::event, time);
    self.tx_q_.write(&event, sizeof(uint16_t));
}

int Radio::read_count_ = 0;

Radio::fwd_cmd_t Radio::readPacket(command_t *&command_dat_out,
                                   uint8_t &command_len_out) {
    fwd_cmd_t result = 0;
    command_len_out = 0;

    while (read_count_ < kMaxPacketsPerRxLoop) {
        self.xbee_.readPacket();
        read_count_++;
        if (!(self.xbee_.getResponse().isAvailable() ||
              self.xbee_.getResponse().isError())) {
            break;
        }

        if (self.xbee_.getResponse().isError()) {
            // TODO - figure out whether there's anything
            // we should do about Xbee errors / log error
        } else if (self.xbee_.getResponse().getApiId() ==
                   ZB_TX_STATUS_RESPONSE) {
            result |= CAN_SEND_FLAG;
            // If we get 2 responses in a row, implies previously we sent an
            // extra one, so we shouldn't respond twice again.
        } else if (self.xbee_.getResponse().getApiId() == ZB_RX_RESPONSE) {
            // received command from xbee_
            self.xbee_.getResponse().getZBRxResponse(self.rx);
            command_dat_out = self.rx.getData();
            command_len_out = self.rx.getDataLength();
            return result;
        } else {
            // TODO - log unrecognized API Id
        }
    }
    result |= STOP_PARSE_FLAG;
    return result;
}

void Radio::updateAddress() {
    self.tx_packet_.setAddress64(self.rx.getRemoteAddress64());
}
