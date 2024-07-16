#include <algorithm>
#include <cmath>
#include "Common.hpp"
#include "Board.hpp"
#include "SRT/SRT.hpp"
#include "RC.hpp"
#include "S-Bus.hpp"
#include "CRSF.hpp"
#include "I-Bus.hpp"

namespace RC
{
    float _channels[maxChannelCount];
    int16_t _channelsRaw[maxChannelCount];
    bool _channelInDZ[maxChannelCount];
    uint8_t _channelsCount = 0;

    State _state = State::signal_lose;
    uint8_t _rssi = UINT8_MAX;
    uint32_t _lastValidTimestamp;

    //=================================================

    int32_t _channelsAssign[static_cast<int>(ChannelFunction::__end)] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // CRSF
    //  float _minChannelValue[maxChannelCount] = {191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191},
        //    _maxChannelValue[maxChannelCount] = {1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792},

    // SBUS
    float _minChannelValue[maxChannelCount] = {240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240},
          _maxChannelValue[maxChannelCount] = {1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807, 1807},

          _channelDeadZone[maxChannelCount] = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5},
          _channelIsReverse[maxChannelCount] = {1, 1, 1, 13,13, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    ProtocolDetector _selectedProtocol = ProtocolDetector::SBUS;

    //=================================================

    State state() { return _state; }
    uint8_t rssi() { return _rssi; }

    int16_t rawChannel(unsigned channel)
    {
        if (channel < 1 or
            channel > maxChannelCount or
            channel > _channelsCount)
            return 0;
        else
            return _channelsRaw[channel - 1];
    }

    float channel(unsigned channel)
    {
        if (channel < 1 or
            channel > maxChannelCount or
            channel > _channelsCount)
            return NAN;
        else
            return _channels[channel - 1];
    }

    float channel(ChannelFunction ch)
    {
        if (static_cast<unsigned>(ch) >= static_cast<unsigned>(ChannelFunction::__end) or
            _channelsAssign[static_cast<unsigned>(ch)] == 0)
            return NAN;

        return channel(_channelsAssign[static_cast<unsigned>(ch)]);
    }

    bool inDZ(unsigned channel)
    {
        if (channel == 0 or
            channel > maxChannelCount or
            channel > _channelsCount)
            return false;

        return _channelInDZ[channel - 1];
    }

    bool inDZ(ChannelFunction ch)
    {
        if (static_cast<unsigned>(ch) >= static_cast<unsigned>(ChannelFunction::__end) or
            _channelsAssign[static_cast<unsigned>(ch)] == 0)
            return false;

        return inDZ(_channelsAssign[static_cast<unsigned>(ch)]);
    }

    uint8_t channelCount() { return _channelsCount; }

    void update(int16_t channels[], unsigned channelCount, uint8_t rssi, bool signalAvailable)
    {
        if (not signalAvailable)
            return;

        _state = State::ok;
        _lastValidTimestamp = millis();

        _channelsCount = std::min<unsigned>(maxChannelCount, channelCount);
        _rssi = rssi;

        for (unsigned i = 0; i < channelCount; i++)
        {
            float normalizedValue = channels[i];
            normalizedValue -= _minChannelValue[i];
            normalizedValue /= float(_maxChannelValue[i] - _minChannelValue[i]);
            normalizedValue = std::clamp<float>(normalizedValue, 0, 1);

            if (_channelIsReverse[i] < 0)
                normalizedValue = 1 - normalizedValue;

            normalizedValue = normalizedValue * 2 - 1;
            normalizedValue = std::clamp<float>(normalizedValue, -1, 1);

            _channelInDZ[i] = std::abs(normalizedValue) <= (_channelDeadZone[i] * 1e-2);
            _channels[i] = normalizedValue;
            _channelsRaw[i] = channels[i];
        }
        for (unsigned i = channelCount; i < maxChannelCount; i++)
            _channels[i] = NAN;
    }

    SBus sBusParser;
    CRSF crsfParser;
    static constexpr RC_parser *parsers[] = {
        &crsfParser,
        &sBusParser,
    };

    // uint32_t protocolProbeStart = 0;
    // static constexpr uint32_t protocolProbeTime = 100;
    // void protocolProbeHandler()
    // {
    //     if (_currentProtocol != ProtocolDetector::not_connected)
    //         return;

    //     if (millis() - protocolProbeStart < protocolProbeTime)
    //         return;
    //     protocolProbeStart = millis();

    //     RC_UART.end();
    //     while (RC_UART.available())
    //         RC_UART.read();
    //     RC_UART.clearParityErrorFlag(), RC_UART.clearWriteError();

    //     switch (protocolProbe)
    //     {
    //     case ProtocolDetector::not_connected:
    //         protocolProbe = ProtocolDetector::IBUS;
    //         RC_UART.begin(115'200);
    //         break;
    //     case ProtocolDetector::IBUS:
    //         protocolProbe = ProtocolDetector::SBUS;
    //         RC_UART.begin(100'000,
    //                       UART::WordLen::nine, UART::StopBit::two, UART::ParityControl::even,
    //                       false, false, true);
    //         break;
    //     default:
    //         protocolProbe = ProtocolDetector::not_connected;
    //         break;
    //     }
    // }

    void checkValues()
    {
        for (float &max : _maxChannelValue)
            max = std::clamp<float>(max, 0, 4000);
        for (float &min : _minChannelValue)
            min = std::clamp<float>(min, 0, 4000);
        for (float &dz : _channelDeadZone)
            dz = std::clamp<float>(dz, 0, 100);
        for (float &rev : _channelIsReverse)
            rev = rev > 0 ? 1 : -1;
        for (int32_t &assign : _channelsAssign)
            if (assign < 0 or assign > maxChannelCount)
                assign = 0;
        if (static_cast<int32_t>(_selectedProtocol) >= static_cast<int32_t>(ProtocolDetector::__end))
            _selectedProtocol = ProtocolDetector::not_connected;
    }

    void init()
    {
        _channelsAssign[static_cast<int>(ChannelFunction::ROLL)] = 1;
        _channelsAssign[static_cast<int>(ChannelFunction::PITCH)] = 2;
        _channelsAssign[static_cast<int>(ChannelFunction::THROTTLE)] = 3;
        _channelsAssign[static_cast<int>(ChannelFunction::YAW)] = 4;
        _channelsAssign[static_cast<int>(ChannelFunction::ARMSWITCH)] = 5;
        _channelsAssign[static_cast<int>(ChannelFunction::AUX_1)] = 6;
        _channelsAssign[static_cast<int>(ChannelFunction::AUX_2)] = 7;
        _channelsAssign[static_cast<int>(ChannelFunction::AUX_3)] = 8;
        _channelsAssign[static_cast<int>(ChannelFunction::PARAM_1)] = 9;
        _channelsAssign[static_cast<int>(ChannelFunction::PARAM_2)] = 10;
    }

    void enable()
    {
        checkValues();
        rcUart.end();

        switch (_selectedProtocol)
        {
        case ProtocolDetector::CRSF:
            rcUart.begin(420'000 >> 8);
            break;
        case ProtocolDetector::SBUS:
            rcUart.begin((100'000 >> 8),
                          UART::WordLen::nine, UART::StopBit::two, UART::ParityControl::even,
                          false, true, true);
            break;
        default:
            break;
        }
    }

    void handler() {}

    void ahrsTickHandler()
    {
        if ((millis() - _lastValidTimestamp) > signalLoseTimeout)
            _state = State::signal_lose;
    }

    void callBackHandler()
    {
        while (rcUart.available())
        {
            uint8_t buff[64];
            size_t count = rcUart.readBytes(buff, std::min<size_t>(sizeof(buff), rcUart.available()));

            if (_selectedProtocol == ProtocolDetector::not_connected or
                static_cast<int>(_selectedProtocol) >= static_cast<int>(ProtocolDetector::__end))
                return;

            int16_t ch[maxChannelCount];
            unsigned channelCount;
            uint8_t rssi;
            bool signalAvailable;

            if (parsers[static_cast<int>(_selectedProtocol) - 1]->parseData(buff, count, rcUart.getParityErrorFlag(),
                                                                            ch, channelCount, rssi, signalAvailable) == true)
                update(ch, channelCount, rssi, signalAvailable);

            rcUart.clearParityErrorFlag();
        }
    }

    REGISTER_SRT_MODULE(RC, init, enable, handler);
}
