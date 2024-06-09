#include "Common.hpp"
#include "rc/RC.hpp"
#include "motor/motor.hpp"
#include <algorithm>

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{
    const float min = std::min<float>(((RC::channel(RC::ChannelFunction::PARAM_1) + 1) / 2),
                                      ((RC::channel(RC::ChannelFunction::PARAM_2) + 1) / 2)),
                max = std::max<float>(((RC::channel(RC::ChannelFunction::PARAM_1) + 1) / 2),
                                      ((RC::channel(RC::ChannelFunction::PARAM_2) + 1) / 2));
    const float out = map(RC::channel(RC::ChannelFunction::AUX_3), -1, 1, min, max);
    Motor::setPower(5, out);
}