#pragma once
#include <cstdint>

namespace Motor
{
    static constexpr unsigned maxCount = 8;

    static constexpr uint32_t disarmTime = 1'000;
    static constexpr uint32_t armTime = 1'000;

    enum class State
    {
        undefined,
        disarming,
        disarmed,
        arming,
        armed,
    };

    // enum class Protocol
    // {
    //     pwm50,
    //     pwm100,
    //     pwm200,
    //     pwm400,
    //     dshot150,
    //     dshot300,
    // };

    // State getState();
    // void setProtocol(Protocol);
    // Protocol getProtocol();

    bool setPower(unsigned motor, float power);
    float getPower(unsigned motor);
    const float *const getPower();

    void disarm();
    void arm();
    State getState();

    void stateHandler();

    // void setChannel(unsigned motor, int channel);
    // int getChannel(unsigned motor);

    // int getMotor(unsigned channel);

    void updateOutput(unsigned motor);
}