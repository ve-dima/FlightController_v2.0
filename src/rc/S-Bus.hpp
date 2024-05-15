/**
 * @file S-Bus.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Парсер протокола S-Bus
 */
#pragma once
#include <cstdint>
#include "RC.hpp"

/// @brief SBus парсер
class SBus : public RC_parser
{
private:
    static constexpr uint32_t PROTOCOL_TIMEGAP = 5; // Packets are received very ~7ms so use ~half that for the gap
    static constexpr uint32_t PROTOCOL_MAX_DELAY = 20;
    static constexpr uint8_t PROTOCOL_PACKET_LEN = 25;

    static constexpr uint8_t PROTOCOL_DATA_LEN = 23;
    static constexpr uint8_t PROTOCOL_PACKET_HEADER = 0x0F;
    static constexpr uint8_t PROTOCOL_PACKET_FOOTER = 0x00;
    static constexpr uint8_t PROTOCOL_PACKET_CH17_Msk = 0x01;
    static constexpr uint8_t PROTOCOL_PACKET_CH18_Msk = 0x02;
    static constexpr uint8_t PROTOCOL_PACKET_FRAME_LOST_Msk = 0x04;
    static constexpr uint8_t PROTOCOL_PACKET_FAILSAFE_Msk = 0x08;

    static constexpr uint16_t PROTOCOL_MIN_SIGNAL = 172;
    static constexpr uint16_t PROTOCOL_MAX_SIGNAL = 1811;

    uint32_t lastReceiveTime = 0;

    uint8_t dataBuffer[PROTOCOL_DATA_LEN]; // channel's values + info-byte
    uint8_t len;

    enum class State
    {
        get_header,
        get_data,
        get_info,
        get_footer,
        ignore,
    } state = State::get_header;

public:
    static constexpr uint8_t channelCount = 18;
    bool parseData(uint8_t data[], size_t len, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable) override final;
};