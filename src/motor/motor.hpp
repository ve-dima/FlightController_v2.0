#pragma once
#include <cstdint>

namespace Motor
{
    static constexpr unsigned maxCount = 6;

    static constexpr uint32_t disarmTime = 500;
    static constexpr uint32_t armTime = 500;

    enum class State
    {
        undefined,
        disarming,
        disarm,
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

    void disarm();
    void arm();

    void stateHandler();

    // void setChannel(unsigned motor, int channel);
    // int getChannel(unsigned motor);

    // int getMotor(unsigned channel);

    void updateOutput(unsigned motor);
}