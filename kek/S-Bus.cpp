#include "S-Bus.hpp"
#include "Common.hpp"

bool SBus::parseData(uint8_t inByte, bool parityError, int16_t ch[], unsigned &chCount, uint8_t &rssi, bool &signalAvailable)
{
    lastReceiveTime = millis();

    if (millis() - lastReceiveTime >= PROTOCOL_TIMEGAP)
        state = State::get_header, len = 0;
    if (parityError)
        state = State::ignore;

    switch (state)
    {
    case State::get_header:
        if (inByte != PROTOCOL_PACKET_HEADER)
            state = State::ignore;
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
            state = State::ignore;
            break;
        }

        ch[0] = static_cast<int16_t>(dataBuffer[0] |
                                     ((dataBuffer[1] << 8) & 0x07FF));

        ch[1] = static_cast<int16_t>((dataBuffer[1] >> 3) |
                                     ((dataBuffer[2] << 5) & 0x07FF));

        ch[2] = static_cast<int16_t>((dataBuffer[2] >> 6) |
                                     (dataBuffer[3] << 2) |
                                     ((dataBuffer[4] << 10) & 0x07FF));

        ch[3] = static_cast<int16_t>((dataBuffer[4] >> 1) |
                                     ((dataBuffer[5] << 7) & 0x07FF));

        ch[4] = static_cast<int16_t>((dataBuffer[5] >> 4) |
                                     ((dataBuffer[6] << 4) & 0x07FF));

        ch[5] = static_cast<int16_t>((dataBuffer[6] >> 7) |
                                     (dataBuffer[7] << 1) |
                                     ((dataBuffer[8] << 9) & 0x07FF));

        ch[6] = static_cast<int16_t>((dataBuffer[8] >> 2) |
                                     ((dataBuffer[9] << 6) & 0x07FF));

        ch[7] = static_cast<int16_t>((dataBuffer[9] >> 5) |
                                     ((dataBuffer[10] << 3) & 0x07FF));

        ch[8] = static_cast<int16_t>(dataBuffer[11] |
                                     ((dataBuffer[12] << 8) & 0x07FF));

        ch[9] = static_cast<int16_t>((dataBuffer[12] >> 3) |
                                     ((dataBuffer[13] << 5) & 0x07FF));

        ch[10] = static_cast<int16_t>((dataBuffer[13] >> 6) |
                                      (dataBuffer[14] << 2) |
                                      ((dataBuffer[15] << 10) & 0x07FF));

        ch[11] = static_cast<int16_t>((dataBuffer[15] >> 1) |
                                      ((dataBuffer[16] << 7) & 0x07FF));

        ch[12] = static_cast<int16_t>((dataBuffer[16] >> 4) |
                                      ((dataBuffer[17] << 4) & 0x07FF));

        ch[13] = static_cast<int16_t>((dataBuffer[17] >> 7) |
                                      (dataBuffer[18] << 1) |
                                      ((dataBuffer[19] << 9) & 0x07FF));

        ch[14] = static_cast<int16_t>((dataBuffer[19] >> 2) |
                                      ((dataBuffer[20] << 6) & 0x07FF));

        ch[15] = static_cast<int16_t>((dataBuffer[20] >> 5) |
                                      ((dataBuffer[21] << 3) & 0x07FF));

        ch[16] = dataBuffer[22] & PROTOCOL_PACKET_CH17_Msk;
        ch[17] = dataBuffer[22] & PROTOCOL_PACKET_CH18_Msk;

        // bool frameLostFlag = dataBuffer[22] & PROTOCOL_PACKET_FRAME_LOST_Msk;
        bool failSafe = dataBuffer[22] & PROTOCOL_PACKET_FAILSAFE_Msk;

        state = State::ignore;
        ch[16] = ch[16] > 0 ? 1 : 0;
        ch[17] = ch[17] > 0 ? 1 : 0;
        chCount = channelCount;
        
        rssi = UINT8_MAX;
        signalAvailable = failSafe;
        return true;

        break;
    }

    default:
        break;
    }

    return false;
}