#include "mavlink/common/mavlink.h"
#include "Board.hpp"
#include "param/param.hpp"
#include "Common.hpp"
#include "ahrs/ahrs.hpp"

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

void loop()
{
    while (mav0Uart.available())
    {
        mavlink_message_t msg;
        mavlink_status_t status = {0};
        if (mavlink_parse_char(MAVLINK_COMM_0, mav0Uart.read(), &msg, &status) != MAVLINK_FRAMING_OK)
            continue;

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
}