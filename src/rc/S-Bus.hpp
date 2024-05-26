#pragma once
#include <cstdint>
#include "RC.hpp"


class SBus : public RC_parser
{
private:
    static constexpr uint32_t PROTOCOL_TIMEGAP = 4; // Packets are received very ~7ms so use ~half that for the gap
    static constexpr uint32_t PROTOCOL_MAX_DELAY = 20;
    static constexpr uint8_t PROTOCOL_PACKET_LEN = 25;

    static constexpr uint8_t PROTOCOL_DATA_LEN = 25;
    static constexpr uint8_t PROTOCOL_PACKET_HEADER = 0x0F;
    static constexpr uint8_t PROTOCOL_PACKET_HEADER_POS = 0;
    static constexpr uint8_t PROTOCOL_PACKET_FOOTER = 0x00;
    static constexpr uint8_t PROTOCOL_PACKET_FOOTER_POS = 24;
    static constexpr uint8_t PROTOCOL_PACKET_CH17_Msk = 0x01;
    static constexpr uint8_t PROTOCOL_PACKET_CH18_Msk = 0x02;
    static constexpr uint8_t PROTOCOL_PACKET_FRAME_LOST_Msk = 0x04;
    static constexpr uint8_t PROTOCOL_PACKET_FAILSAFE_Msk = 0x08;

    uint32_t lastReceiveTime = 0;

    uint8_t dataBuffer[PROTOCOL_DATA_LEN];
    uint8_t len;

public:
    static constexpr uint8_t channelCount = 16;
    bool parseData(uint8_t data[], size_t len, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable) override final;
};