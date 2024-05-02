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
    // case param::UINT8:
    //     mavParam.value.param_uint8 = *static_cast<const uint8_t *>(param.ptr->address);
    //     mavParam.param.param_type = MAV_PARAM_TYPE_UINT8;
    //     break;
    // case param::INT8:
    //     mavParam.value.param_int8 = *static_cast<const int8_t *>(param.ptr->address);
    //     mavParam.param.param_type = MAV_PARAM_TYPE_INT8;
    //     break;
    // case param::UINT32:
    //     mavParam.value.param_uint32 = *static_cast<const uint32_t *>(param.ptr->address);
    //     mavParam.param.param_type = MAV_PARAM_TYPE_UINT32;
    //     break;
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
            // TODO: param set
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

    for (static uint32_t tim = 0; millis() - tim > 50; tim = millis())
    {
        const auto attitude = AHRS::getFRD_Attitude();
        debugUart.print(attitude.w(), 4);
        debugUart.print(',');
        debugUart.print(attitude.x(), 4);
        debugUart.print(',');
        debugUart.print(attitude.y(), 4);
        debugUart.print(',');
        debugUart.print(attitude.z(), 4);
        debugUart.println();
        // debugUart.println(millis());
    }
}