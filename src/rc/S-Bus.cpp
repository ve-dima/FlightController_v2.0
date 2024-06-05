#include "S-Bus.hpp"
#include "Common.hpp"
#include <cstring>
#include <algorithm>

bool SBus::parseData(uint8_t data[], size_t inLen, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable)
{
    if (parityError)
    {
        len = 0;
        return false;
    }

    if (millis() - lastReceiveTime >= PROTOCOL_TIMEGAP)
        len = 0;
    lastReceiveTime = millis();

    const unsigned currentLen = std::min<unsigned>(inLen, PROTOCOL_DATA_LEN - len);
    memcpy(dataBuffer + len, data, currentLen);
    len += currentLen;

    if (len < PROTOCOL_DATA_LEN)
        return false;

    if (dataBuffer[PROTOCOL_PACKET_HEADER_POS] != PROTOCOL_PACKET_HEADER or
        dataBuffer[PROTOCOL_PACKET_FOOTER_POS] != PROTOCOL_PACKET_FOOTER)
        return false;

    channels[0] = static_cast<int16_t>(dataBuffer[1] |
                                       ((dataBuffer[2] << 8) & 0x07FF));

    channels[1] = static_cast<int16_t>((dataBuffer[2] >> 3) |
                                       ((dataBuffer[3] << 5) & 0x07FF));

    channels[2] = static_cast<int16_t>((dataBuffer[3] >> 6) |
                                       (dataBuffer[4] << 2) |
                                       ((dataBuffer[5] << 10) & 0x07FF));

    channels[3] = static_cast<int16_t>((dataBuffer[5] >> 1) |
                                       ((dataBuffer[6] << 7) & 0x07FF));

    channels[4] = static_cast<int16_t>((dataBuffer[6] >> 4) |
                                       ((dataBuffer[7] << 4) & 0x07FF));

    channels[5] = static_cast<int16_t>((dataBuffer[7] >> 7) |
                                       (dataBuffer[8] << 1) |
                                       ((dataBuffer[9] << 9) & 0x07FF));

    channels[6] = static_cast<int16_t>((dataBuffer[9] >> 2) |
                                       ((dataBuffer[10] << 6) & 0x07FF));

    channels[7] = static_cast<int16_t>((dataBuffer[10] >> 5) |
                                       ((dataBuffer[11] << 3) & 0x07FF));

    channels[8] = static_cast<int16_t>(dataBuffer[12] |
                                       ((dataBuffer[13] << 8) & 0x07FF));

    channels[9] = static_cast<int16_t>((dataBuffer[13] >> 3) |
                                       ((dataBuffer[14] << 5) & 0x07FF));

    channels[10] = static_cast<int16_t>((dataBuffer[14] >> 6) |
                                        (dataBuffer[15] << 2) |
                                        ((dataBuffer[16] << 10) & 0x07FF));

    channels[11] = static_cast<int16_t>((dataBuffer[16] >> 1) |
                                        ((dataBuffer[17] << 7) & 0x07FF));

    channels[12] = static_cast<int16_t>((dataBuffer[17] >> 4) |
                                        ((dataBuffer[18] << 4) & 0x07FF));

    channels[13] = static_cast<int16_t>((dataBuffer[18] >> 7) |
                                        (dataBuffer[19] << 1) |
                                        ((dataBuffer[20] << 9) & 0x07FF));

    channels[14] = static_cast<int16_t>((dataBuffer[20] >> 2) |
                                        ((dataBuffer[21] << 6) & 0x07FF));

    channels[15] = static_cast<int16_t>((dataBuffer[21] >> 5) |
                                        ((dataBuffer[22] << 3) & 0x07FF));

    channels[16] = dataBuffer[23] & PROTOCOL_PACKET_CH17_Msk;
    channels[17] = dataBuffer[23] & PROTOCOL_PACKET_CH18_Msk;

    const bool frameLostFlag = dataBuffer[23] & PROTOCOL_PACKET_FRAME_LOST_Msk;
    const bool failSafe = dataBuffer[23] & PROTOCOL_PACKET_FAILSAFE_Msk;

    channels[16] = channels[16] > 0 ? 1 : 0;
    channels[17] = channels[17] > 0 ? 1 : 0;
    channelCount = 18;

    rssi = UINT8_MAX;
    signalAvailable = (failSafe == false) and (frameLostFlag == false);
    return true;
}