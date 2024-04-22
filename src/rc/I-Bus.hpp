/**
 * @file I-Bus.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Парсер протокола I-Bus приёмопередатчиков FlySky
 * @authors https://github.com/bmellink/IBusBM/blob/master/src/IBusBM.h
 */
#pragma once
#include <cstdint>
#include "RC.hpp"

/// @brief I-Bus парсер
class IBus : public RC_parser
{
private:
    static constexpr uint32_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
    static constexpr uint32_t PROTOCOL_MAX_DELAY = 20;

    static constexpr uint8_t PROTOCOL_MAX_LENGTH = 32;
    static constexpr uint8_t PROTOCOL_OVERHEAD = 3; // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes

    static constexpr uint8_t PROTOCOL_COMMAND40 = 0x40; // Command to set servo or motor speed is always 0x40

    static constexpr int16_t MIN_CHANNEL_VALUE = 900;
    static constexpr int16_t MAX_CHANNEL_VALUE = 2'100;

    uint32_t lastReceiveTime = 0;
    uint8_t buffer[PROTOCOL_MAX_LENGTH], bufferLen = 0;

    uint8_t len; // message length
    uint8_t lchksum;
    uint16_t chksum; // checksum calculation
    enum class State
    {
        get_length,
        get_data,
        get_lchksum,
        get_hchksum,
        ignore,
    } state = State::get_length;

public:
    static constexpr uint8_t channelCount = 12;
    bool parseData(uint8_t data, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable) override final;
};