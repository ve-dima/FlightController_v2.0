#include <cmath>
#include "rc/RC.hpp"
#include "motor/motor.hpp"
#include "SRT/SRT.hpp"

namespace Servo
{
    void init() {}
    void enable() {}

    static constexpr float cameraPostion[] = {
        0,
        0.5,
        1,
    };
    static constexpr float holderPosition[] = {
        0.15,
        0.85,
    };

    void handler()
    {
        const float holderSwitch = RC::channel(RC::ChannelFunction::AUX_3);
        if (holderSwitch > 0)
            Motor::setPower(4, holderPosition[1]);
        else
            Motor::setPower(4, holderPosition[0]);

        const float cameraSwitch = RC::channel(RC::ChannelFunction::AUX_2);
        if (cameraSwitch >= 0.5)
            Motor::setPower(5, cameraPostion[2]);
        else if (cameraSwitch > -0.5 and cameraSwitch < 0.5)
            Motor::setPower(5, cameraPostion[1]);
        else
            Motor::setPower(5, cameraPostion[0]);
    }

    REGISTER_SRT_MODULE(servo, init, enable, handler);
}