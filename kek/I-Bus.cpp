#include "I-Bus.hpp"
#include "Common.hpp"
#include <algorithm>

bool IBus::parseData(uint8_t inByte, bool parityError, int16_t ch[], unsigned &chCount, uint8_t &rssi, bool &signalAvailable)
{
    if (millis() - lastReceiveTime >= PROTOCOL_TIMEGAP)
        state = State::get_length, bufferLen = 0;
    lastReceiveTime = millis();

    switch (state)
    {
    case State::get_length:
        if (inByte > PROTOCOL_MAX_LENGTH or inByte < PROTOCOL_OVERHEAD)
            state = State::ignore;
        else
        {
            len = inByte - PROTOCOL_OVERHEAD;
            chksum = 0xFF'FF - inByte;
            bufferLen = 0;
            state = State::get_data;
        }
        break;

    case State::get_data:
        buffer[bufferLen++] = inByte;
        chksum -= inByte;

        if (bufferLen == len)
            state = State::get_lchksum;
        break;

    case State::get_lchksum:
        lchksum = inByte;
        state = State::get_hchksum;
        break;

    case State::get_hchksum:
        if (chksum != ((inByte << 8) | lchksum))
        {
            state = State::ignore;
            break;
        }

        if (buffer[0] != PROTOCOL_COMMAND40)
            break;

        for (int32_t i = std::min<int32_t>(channelCount * 2, len); i > 0; i -= 2)
        {
            int16_t value = buffer[i - 1] | (buffer[i] << 8);
            if (value < MIN_CHANNEL_VALUE or value > MAX_CHANNEL_VALUE)
                return false;
            ch[i / 2 - 1] = value;
        }

        state = State::ignore;

        rssi = UINT8_MAX;
        chCount = channelCount;
        signalAvailable = true;
        return true;
        break;

    default:
        break;
    }
    return false;
}
