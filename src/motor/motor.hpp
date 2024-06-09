#pragma once
#include <cstdint>

namespace Motor
{
    static constexpr unsigned maxCount = 4;

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

    bool setPower(unsigned motor, float power);
    float getPower(unsigned motor);
    const float *const getPower();

    void disarm();
    void arm();
    State getState();

    void stateHandler();

    void updateOutput(unsigned motor);
}