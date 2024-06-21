#include "mavlink.hpp"
#include "mavlink/common/mavlink.h"
#include "Common.hpp"
#include "SRT/SRT.hpp"
#include "ahrs/ahrs.hpp"
#include "control/Control.hpp"
#include "motor/motor.hpp"
#include "rc/RC.hpp"
#include "ahrs/atmosphere.hpp"
#include "param/param.hpp"
#include "Board.hpp"

// MavLinkReporter::MavLinkReporter(mavlink_channel_t channel, MavLinkProfiles::Profile &profile) : channel(channel), profile(profile) {}

void mavlink_heartbeat_report(mavlink_channel_t ch)
{
    mavlink_msg_heartbeat_send(ch,
                               MAV_TYPE::MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC, 0, 0, 0);
}

void mavlink_quat_report(mavlink_channel_t ch)
{
    const Eigen::Quaternionf attitude = AHRS::getFRD_Attitude();
    // const Eigen::Quaternionf attitude = Control::getTargetAttitude();
    const Eigen::Vector3f rotateRate = AHRS::getFRD_RotateSpeed();

    mavlink_msg_attitude_quaternion_send(ch, millis(),
                                         attitude.w(), attitude.x(), attitude.y(), attitude.z(),
                                         rotateRate.x(), rotateRate.y(), rotateRate.z(), nullptr);
}

void mavlink_euler_report(mavlink_channel_t ch)
{
    const AHRS::Eulerf attitude = AHRS::getFRD_Euler();
    const Eigen::Vector3f rotateRate = AHRS::getFRD_RotateSpeed();

    mavlink_msg_attitude_send(ch, millis(),
                              attitude.roll, attitude.pitch, attitude.yaw,
                              rotateRate.x(), rotateRate.y(), rotateRate.z());
}

void mavlink_attitude_target_report(mavlink_channel_t ch)
{
    Eigen::Quaternionf targetAttitude = Control::getTargetAttitude();
    const float tA[4] = {targetAttitude.w(), targetAttitude.x(), targetAttitude.y(), targetAttitude.z()};

    const Eigen::Vector3f targetRate = Control::getTargetRate();
    const float targetTrust = Control::getTargetThrust();

    mavlink_msg_attitude_target_send(ch, millis(),
                                     0, tA,
                                     targetRate.x(), targetRate.y(), targetRate.z(),
                                     targetTrust);
}

void mavlink_local_position_report(mavlink_channel_t ch)
{
    const Eigen::Vector3f verticalState = AHRS::getZState();
    mavlink_msg_local_position_ned_send(ch, millis(),
                                        NAN, NAN, verticalState(0),
                                        NAN, NAN, verticalState(1));
}

void mavlink_pressure_report(mavlink_channel_t ch)
{
    const float hPa = AHRS::getPressure() * 1e-2;
    int16_t cDeg = AHRS::getTemperature() * 100;
    mavlink_msg_scaled_pressure_send(ch, millis(),
                                     hPa, NAN,
                                     cDeg, 0);
}

void mavlink_state_report(mavlink_channel_t ch)
{
    const Eigen::Vector3f linearAcc = AHRS::getFRD_LinearAcceleration();
    const Eigen::Vector3f verticalState = AHRS::getZState();
    const Eigen::Vector3f verticalVariance = AHRS::getZVaraince();

    const float velVariance[3] = {NAN, NAN, verticalVariance(1)};
    const float posVariance[3] = {NAN, NAN, verticalVariance(0)};

    Eigen::Quaternionf attitude = AHRS::getFRD_Attitude();
    std::swap(attitude.w(), attitude.z()); // eigen store coeffs in x y z w order, need w first
    const Eigen::Vector3f rotateRate = AHRS::getFRD_RotateSpeed();

    mavlink_msg_control_system_state_send(ch, millis() * 1000,
                                          linearAcc.x(), linearAcc.y(), linearAcc.z(),
                                          NAN, NAN, verticalState(1),
                                          NAN, NAN, verticalState(0),
                                          AHRS::getTiltCos(),
                                          velVariance, posVariance,
                                          attitude.coeffs().data(),
                                          rotateRate.x(), rotateRate.y(), rotateRate.z());
}

void mavlink_actuator_report(mavlink_channel_t ch)
{
    float powers[8];
    std::copy(Motor::getPower(), Motor::getPower() + Motor::maxCount, powers);

    mavlink_msg_actuator_control_target_send(ch, millis() * 1000,
                                             0, powers);
}

void mavlink_rc_report(mavlink_channel_t ch)
{
    mavlink_msg_rc_channels_send(ch, millis(), RC::channelCount(),
                                 RC::rawChannel(1),
                                 RC::rawChannel(2),
                                 RC::rawChannel(3),
                                 RC::rawChannel(4),
                                 RC::rawChannel(5),
                                 RC::rawChannel(6),
                                 RC::rawChannel(7),
                                 RC::rawChannel(8),
                                 RC::rawChannel(9),
                                 RC::rawChannel(10),
                                 RC::rawChannel(11),
                                 RC::rawChannel(12),
                                 RC::rawChannel(13),
                                 RC::rawChannel(14),
                                 RC::rawChannel(15),
                                 RC::rawChannel(16),
                                 RC::rawChannel(17),
                                 RC::rawChannel(18),
                                 static_cast<int8_t>(RC::state()));
}

void paramToMavParam(param::paramVarId_t &param, mavlink_param_value_t &mP)
{
    union
    {
        mavlink_param_union_t value;
        mavlink_param_value_t param;
    } mavParam;
    mavParam.param.param_index = param.index;
    mavParam.param.param_count = param::getParamCount();
    strncpy(mavParam.param.param_id, param.ptr->name, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    std::fill(mavParam.value.bytes, &mavParam.value.bytes[4], 0);

    switch (param.ptr->type)
    {
    case param::INT32:
        mavParam.value.param_int32 = *static_cast<const int32_t *>(param.ptr->address);
        mavParam.param.param_type = MAV_PARAM_TYPE_INT32;
        break;
    case param::FLOAT:
        mavParam.value.param_float = *static_cast<const float *>(param.ptr->address);
        mavParam.param.param_type = MAV_PARAM_TYPE_REAL32;
        break;
    default:
        break;
    }

    mP = mavParam.param;
}

void parseMavlinkMessage(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_PROTOCOL_VERSION:
    {
        mavlink_msg_protocol_version_send(MAVLINK_COMM_0,
                                          MAVLINK_VERSION * 100, 100, 200, nullptr, nullptr);
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        for (unsigned i = 0; i < param::getParamCount(); i++)
        {
            param::paramVarId_t p;
            param::getParamByIndex(i, p);

            mavlink_param_value_t mavParam;
            paramToMavParam(p, mavParam);

            mavlink_msg_param_value_send_struct(MAVLINK_COMM_0, &mavParam);
        }
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        mavlink_param_request_read_t readReq;
        mavlink_msg_param_request_read_decode(&msg, &readReq);
        if (readReq.target_component != mavlink_system.compid or
            readReq.target_system != mavlink_system.sysid)
            break;

        if (readReq.param_index == -1)
        {
            param::paramVarId_t p;
            if (not param::getParamByName(readReq.param_id, p))
                break;

            mavlink_param_value_t mavParam;
            paramToMavParam(p, mavParam);

            mavlink_msg_param_value_send_struct(MAVLINK_COMM_0, &mavParam);
        }
        else if (readReq.param_index < int(param::getParamCount()))
        {
            param::paramVarId_t p;
            if (not param::getParamByIndex(readReq.param_index, p))
                break;

            mavlink_param_value_t mavParam;
            paramToMavParam(p, mavParam);

            mavlink_msg_param_value_send_struct(MAVLINK_COMM_0, &mavParam);
        }
        break;
    }
    case MAVLINK_MSG_ID_PARAM_SET:
    {
        mavlink_param_set_t setReq;
        mavlink_msg_param_set_decode(&msg, &setReq);
        if (setReq.target_component != mavlink_system.compid or
            setReq.target_system != mavlink_system.sysid)
            break;

        if (setReq.param_type != MAV_PARAM_TYPE_INT32 and
            setReq.param_type != MAV_PARAM_TYPE_REAL32)
            break;

        param::paramVarId_t p;
        if (not param::getParamByName(setReq.param_id, p))
            break;
        param::updateParamByPtr(&setReq.param_value, p.ptr);

        mavlink_param_value_t mavParam;
        paramToMavParam(p, mavParam);

        mavlink_msg_param_value_send_struct(MAVLINK_COMM_0, &mavParam);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        mavlink_mission_request_list_t req;
        mavlink_msg_mission_request_list_decode(&msg, &req);

        mavlink_msg_mission_count_send(MAVLINK_COMM_0, req.target_system, req.target_component, 0, 0, 0);
        break;
    }
    case MAVLINK_MSG_ID_PING:
    {
        mavlink_ping_t ping;
        mavlink_msg_ping_decode(&msg, &ping);
        if (ping.target_system == 0 && ping.target_component == 0)
            mavlink_msg_ping_send(MAVLINK_COMM_0, ping.seq, msg.sysid, msg.compid, millis());
        break;
    }
    }
}

void init() {}
void enable() {}
void handler()
{
    for (static uint32_t hearBeatTimer = 0; millis() - hearBeatTimer > 1'000; hearBeatTimer = millis())
    {
        mavlink_heartbeat_report(MAVLINK_COMM_0);
        mavlink_pressure_report(MAVLINK_COMM_0);

        mavlink_heartbeat_report(MAVLINK_COMM_1);
        mavlink_pressure_report(MAVLINK_COMM_1);
    }

    for (static uint32_t attitudeTimer = 0; millis() - attitudeTimer > 100; attitudeTimer = millis())
    {
        mavlink_quat_report(MAVLINK_COMM_0);
        mavlink_euler_report(MAVLINK_COMM_0);
        mavlink_state_report(MAVLINK_COMM_0);

        mavlink_quat_report(MAVLINK_COMM_1);
        mavlink_euler_report(MAVLINK_COMM_1);
        mavlink_state_report(MAVLINK_COMM_1);

        // const float pres = AHRS::getPressure();
        // const float heightByPressure = getAltitudeFromPressure(pres, 101'325);
        // const Eigen::Vector3f verticalState = AHRS::getZState();
        // const auto acc = AHRS::getMagneticField();

        // mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, millis(),
        //                                     verticalState(0), verticalState(1), heightByPressure,
        //                                     acc(0), acc(1), acc(2));
        // mavlink_quat_report(MAVLINK_COMM_0);
    }

    for (static uint32_t rcTimer = 0; millis() - rcTimer > 250; rcTimer = millis())
    {
        mavlink_rc_report(MAVLINK_COMM_0), mavlink_rc_report(MAVLINK_COMM_1);
        mavlink_actuator_report(MAVLINK_COMM_0), mavlink_actuator_report(MAVLINK_COMM_1);
        mavlink_attitude_target_report(MAVLINK_COMM_0), mavlink_attitude_target_report(MAVLINK_COMM_1);
    }

    while (mav0Uart.available())
    {
        mavlink_message_t msg;
        mavlink_status_t status = {0};
        if (mavlink_parse_char(MAVLINK_COMM_0, mav0Uart.read(), &msg, &status) != MAVLINK_FRAMING_OK)
            continue;
        parseMavlinkMessage(msg);
    }

    while (mav1Uart.available())
    {
        mavlink_message_t msg;
        mavlink_status_t status = {0};
        if (mavlink_parse_char(MAVLINK_COMM_0, mav1Uart.read(), &msg, &status) != MAVLINK_FRAMING_OK)
            continue;
        parseMavlinkMessage(msg);
    }
}

REGISTER_SRT_MODULE(mavlink_module, init, enable, handler);