#include "I-Bus.hpp"
#include "Common.hpp"
#include <algorithm>
#include "Board.hpp"
#include "ahrs/ahrs.hpp"
#include "modes/Modes.hpp"
#include "modes/Disarm.hpp"

bool IBus::parseData(uint8_t data[], size_t dataLen, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable)
{
    if (millis() - lastReceiveTime >= PROTOCOL_TIMEGAP)
        state = State::get_length, bufferLen = 0;
    lastReceiveTime = millis();

    for (unsigned i = 0; i < dataLen; i++)
    {
        const uint8_t inByte = data[i];

        switch (state)
        {
        case State::get_length:
            if (inByte > PROTOCOL_MAX_LENGTH or inByte < PROTOCOL_OVERHEAD)
                state = State::ignore;
            else
            {
                len = inByte - PROTOCOL_OVERHEAD;
                chksum = 0xFF'FF - inByte;
                bufferLen = 0;
                state = State::get_data;
            }
            break;

        case State::get_data:
            buffer[bufferLen++] = inByte;
            chksum -= inByte;

            if (bufferLen == len)
                state = State::get_lchksum;
            break;

        case State::get_lchksum:
            lchksum = inByte;
            state = State::get_hchksum;
            break;

        case State::get_hchksum:
        {
            if (chksum != ((inByte << 8) | lchksum))
            {
                state = State::ignore;
                break;
            }

            if (buffer[0] == PROTOCOL_COMMAND40)
            {
                for (int32_t i = std::min<int32_t>(channelCount * 2, len); i > 0; i -= 2)
                {
                    int16_t value = buffer[i - 1] | (buffer[i] << 8);
                    if (value < MIN_CHANNEL_VALUE or value > MAX_CHANNEL_VALUE)
                        return false;
                    channels[i / 2 - 1] = value;
                }

                state = State::ignore;

                rssi = UINT8_MAX;
                channelCount = channelCount;
                signalAvailable = true;
                return true;
                break;
            }

            uint8_t sensorAddress = buffer[0] & 0x0F;
            const SensorInfo_t *s = &sensorList[sensorAddress - 1];
            const int32_t sensorValue = sensorValues[sensorAddress - 1];
            if (sensorAddress <= sensorListSize and
                sensorAddress > 0 and len == 1 and
                needSendTelemetry == false)
            {
                switch (buffer[0] & 0xF0)
                {
                case PROTOCOL_COMMAND_DISCOVER: // 0x80, discover sensor
                    // echo discover command: 0x04, 0x81, 0x7A, 0xFF

                    telemetryLen = 2;
                    telemetryBuffer[0] = 0x04;
                    telemetryBuffer[1] = PROTOCOL_COMMAND_DISCOVER + sensorAddress;
                    break;
                case PROTOCOL_COMMAND_TYPE: // 0x90, send sensor type
                    // echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF

                    telemetryLen = 4;
                    telemetryBuffer[0] = 0x06;
                    telemetryBuffer[1] = PROTOCOL_COMMAND_TYPE + sensorAddress;
                    telemetryBuffer[2] = s->sensorType;
                    telemetryBuffer[3] = s->sensorLength;
                    break;
                case PROTOCOL_COMMAND_VALUE: // 0xA0, send sensor data
                {
                    telemetryBuffer[0] = 0x04 + s->sensorLength;
                    telemetryBuffer[1] = PROTOCOL_COMMAND_VALUE + sensorAddress;
                    telemetryBuffer[2] = sensorValue & 0xFF;
                    telemetryBuffer[3] = (sensorValue >> 8) & 0xFF;
                    if (s->sensorLength == 4)
                    {
                        telemetryBuffer[4] = (sensorValue >> 16) & 0xFF;
                        telemetryBuffer[5] = (sensorValue >> 24) & 0xFF;

                        telemetryLen = 6;
                    }
                    else
                        telemetryLen = 4;
                    break;
                }
                default:
                    sensorAddress = 0; // unknown command, prevent sending chksum
                    break;
                }

                if (sensorAddress > 0)
                {
                    uint16_t crc = 0xFFFF;
                    for (unsigned i = 0; i < telemetryLen; i++)
                        crc -= telemetryBuffer[i];
                    telemetryBuffer[telemetryLen++] = crc & 0xFF;
                    telemetryBuffer[telemetryLen++] = crc >> 8;

                    needSendTelemetry = true;
                }
                else
                {
                    needSendTelemetry = false;
                    telemetryLen = 0;
                }
            }
        }
        default:
            break;
        }
    }

    return false;
}

void IBus::sensorHandler()
{

    sensorValues[external_voltage] = 16.0 * 100;
    sensorValues[pressure] = AHRS::getPressure();

    // const auto att = AHRS::getFRD_Euler();
    // const auto z = AHRS::getZState();
    // const auto acc = AHRS::getRawAcceleration();
    // sensorValues[avg_cell_voltage] = 4.2 * 100;
    // sensorValues[batt_percentage] = 4000;

    // sensorValues[roll] = (int16_t)(att.roll * 100 * (180 / M_PI));
    // sensorValues[pitch] = (int16_t)(att.pitch * 100 * (180 / M_PI));
    // sensorValues[yaw] = (int16_t)(att.yaw * 100 * (180 / M_PI));

    // sensorValues[vertical_speed] = std::abs(z(1) * 100);

    // sensorValues[arm] = FlightModeDispatcher::getCurrentFlightMode() == &disarmMode ? 0 : 1;

    // sensorValues[ax] = acc.x() * 98.06;
    // sensorValues[ay] = acc.y() * 98.06;
    // sensorValues[az] = acc.z() * 98.06;
}

bool IBus::sendTelemetry(uint8_t data[], size_t &count)
{
    if (needSendTelemetry)
    {
        memcpy(data, telemetryBuffer, telemetryLen);
        count = telemetryLen;
        needSendTelemetry = false;
        return true;
    }

    return false;
}
