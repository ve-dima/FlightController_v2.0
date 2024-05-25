#include "S-Bus.hpp"
#include "Common.hpp"
#include <cstring>
#include <algorithm>

bool SBus::parseData(uint8_t data[], size_t len, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable)
{
    lastReceiveTime = millis();

    if (millis() - lastReceiveTime >= PROTOCOL_TIMEGAP)
        state = State::get_header, len = 0;

    if (parityError)
        state = State::ignore;

    if (len < PROTOCOL_DATA_LEN)
    {
        const unsigned currentLen = std::min<unsigned>(len, PROTOCOL_DATA_LEN - len);
        memcpy(dataBuffer + len, data, currentLen);
        len += currentLen;
        return false;
    }

    bool isOk = false;

    for (unsigned i = 0; i < len; i++)
    {
        uint8_t inByte = dataBuffer[i];
        switch (state)
        {
        case State::get_header:
            if (inByte != PROTOCOL_PACKET_HEADER)
                state = State::get_header;
            else
            {
                len = 0;
                state = State::get_data;
            }
            break;

        case State::get_data:
            dataBuffer[len++] = inByte;
            if (len == PROTOCOL_DATA_LEN)
                state = State::get_footer;
            break;

        case State::get_footer:
        {
            if (inByte != PROTOCOL_PACKET_FOOTER)
            {
                // state = State::ignore;
                state = State::get_header;
                break;
            }

            channels[0] = static_cast<int16_t>(dataBuffer[0] |
                                               ((dataBuffer[1] << 8) & 0x07FF));

            channels[1] = static_cast<int16_t>((dataBuffer[1] >> 3) |
                                               ((dataBuffer[2] << 5) & 0x07FF));

            channels[2] = static_cast<int16_t>((dataBuffer[2] >> 6) |
                                               (dataBuffer[3] << 2) |
                                               ((dataBuffer[4] << 10) & 0x07FF));

            channels[3] = static_cast<int16_t>((dataBuffer[4] >> 1) |
                                               ((dataBuffer[5] << 7) & 0x07FF));

            channels[4] = static_cast<int16_t>((dataBuffer[5] >> 4) |
                                               ((dataBuffer[6] << 4) & 0x07FF));

            channels[5] = static_cast<int16_t>((dataBuffer[6] >> 7) |
                                               (dataBuffer[7] << 1) |
                                               ((dataBuffer[8] << 9) & 0x07FF));

            channels[6] = static_cast<int16_t>((dataBuffer[8] >> 2) |
                                               ((dataBuffer[9] << 6) & 0x07FF));

            channels[7] = static_cast<int16_t>((dataBuffer[9] >> 5) |
                                               ((dataBuffer[10] << 3) & 0x07FF));

            channels[8] = static_cast<int16_t>(dataBuffer[11] |
                                               ((dataBuffer[12] << 8) & 0x07FF));

            channels[9] = static_cast<int16_t>((dataBuffer[12] >> 3) |
                                               ((dataBuffer[13] << 5) & 0x07FF));

            channels[10] = static_cast<int16_t>((dataBuffer[13] >> 6) |
                                                (dataBuffer[14] << 2) |
                                                ((dataBuffer[15] << 10) & 0x07FF));

            channels[11] = static_cast<int16_t>((dataBuffer[15] >> 1) |
                                                ((dataBuffer[16] << 7) & 0x07FF));

            channels[12] = static_cast<int16_t>((dataBuffer[16] >> 4) |
                                                ((dataBuffer[17] << 4) & 0x07FF));

            channels[13] = static_cast<int16_t>((dataBuffer[17] >> 7) |
                                                (dataBuffer[18] << 1) |
                                                ((dataBuffer[19] << 9) & 0x07FF));

            channels[14] = static_cast<int16_t>((dataBuffer[19] >> 2) |
                                                ((dataBuffer[20] << 6) & 0x07FF));

            channels[15] = static_cast<int16_t>((dataBuffer[20] >> 5) |
                                                ((dataBuffer[21] << 3) & 0x07FF));

            channels[16] = dataBuffer[22] & PROTOCOL_PACKET_CH17_Msk;
            channels[17] = dataBuffer[22] & PROTOCOL_PACKET_CH18_Msk;

            // bool frameLostFlag = dataBuffer[22] & PROTOCOL_PACKET_FRAME_LOST_Msk;
            bool failSafe = dataBuffer[22] & PROTOCOL_PACKET_FAILSAFE_Msk;

            state = State::ignore;
            channels[16] = channels[16] > 0 ? 1 : 0;
            channels[17] = channels[17] > 0 ? 1 : 0;
            channelCount = 18;

            rssi = UINT8_MAX;
            signalAvailable = failSafe;
            isOk = true;

            state = State::get_header;

            break;
        }

        default:
            state = State::get_header;
            break;
        }
    }

    return isOk;
}