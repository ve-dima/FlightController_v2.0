#pragma once
#include <cstdint>
#include <cstddef>

namespace RC
{
    static constexpr uint8_t maxChannelCount = 18;

    enum class ProtocolDetector : int32_t
    {
        not_connected = 0,
        CRSF,
        SBUS,
        __end,
    };

    enum class State
    {
        ok,
        signal_lose,
    };

    enum class ChannelFunction
    {
        THROTTLE = 0,
        ROLL = 1,
        PITCH = 2,
        YAW = 3,
        RETURN = 4,
        LOITER = 5,
        OFFBOARD = 6,
        FLAPS = 7,
        AUX_1 = 8,
        AUX_2 = 9,
        AUX_3 = 10,
        AUX_4 = 11,
        AUX_5 = 12,
        AUX_6 = 13,
        PARAM_1 = 14,
        PARAM_2 = 15,
        PARAM_3_5 = 16,
        KILLSWITCH = 17,
        TRANSITION = 18,
        GEAR = 19,
        ARMSWITCH = 20,
        FLTBTN_SLOT_1 = 21,
        FLTBTN_SLOT_2 = 22,
        FLTBTN_SLOT_3 = 23,
        FLTBTN_SLOT_4 = 24,
        FLTBTN_SLOT_5 = 25,
        FLTBTN_SLOT_6 = 26,
        ENGAGE_MAIN_MOTOR = 27,
        __end,
    };

    constexpr uint32_t signalLoseTimeout = 250;

    State state();
    uint8_t rssi();
    uint8_t channelCount();

    int16_t rawChannel(unsigned channel);
    float channel(unsigned channel);
    float channel(ChannelFunction channel);
    bool inDZ(unsigned channel);
    bool inDZ(ChannelFunction channel);

    void callBackHandler();
    void ahrsTickHandler();

    void checkValues();
};

class RC_parser
{
public:
    virtual bool parseData(uint8_t data[], size_t len, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable) = 0;
    virtual void sendTelemetry(uint8_t data[], uint8_t &count) {}
};
