#include "Common.hpp"
#include "I2C/I2C.hpp"
#include "SRT/SRT.hpp"
#include "mavlink/common/mavlink.h"

void loop()
{
    for (static uint32_t hearBeatTimer = 0; millis() - hearBeatTimer > 500; hearBeatTimer = millis())
        mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE::MAV_TYPE_IMU, MAV_AUTOPILOT::MAV_AUTOPILOT_INVALID, 0, 0, 0);
}