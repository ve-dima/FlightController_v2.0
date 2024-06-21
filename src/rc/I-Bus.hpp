/**
 * @file I-Bus.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Парсер протокола I-Bus приёмопередатчиков FlySky
 * @authors https://github.com/bmellink/IBusBM/blob/master/src/IBusBM.h
 */
#pragma once
#include <cstdint>
#include "RC.hpp"

/// @brief I-Bus парсер
class IBus : public RC_parser
{
private:
    static constexpr uint32_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
    static constexpr uint32_t PROTOCOL_MAX_DELAY = 20;

    static constexpr uint8_t PROTOCOL_MAX_LENGTH = 32;
    static constexpr uint8_t PROTOCOL_OVERHEAD = 3; // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes

    static constexpr uint8_t PROTOCOL_COMMAND40 = 0x40;        // Command to set servo or motor speed is always 0x40
    static constexpr uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command discover sensor (lowest 4 bits are sensor)
    static constexpr uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command discover sensor (lowest 4 bits are sensor)
    static constexpr uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command send sensor data (lowest 4 bits are sensor)

    static constexpr int16_t MIN_CHANNEL_VALUE = 900;
    static constexpr int16_t MAX_CHANNEL_VALUE = 2'100;

    uint32_t lastReceiveTime = 0;
    uint8_t buffer[PROTOCOL_MAX_LENGTH], bufferLen = 0;

    uint8_t len; // message length
    uint8_t lchksum;
    uint16_t chksum; // checksum calculation
    enum class State
    {
        get_length,
        get_data,
        get_lchksum,
        get_hchksum,
        ignore,
    } state = State::get_length;

    /*
#define IBUS_MEAS_TYPE_EXTV           0x03 // External voltage
#define IBUS_MEAS_TYPE_CELL           0x04 // Avg cell voltage
#define IBUS_MEAS_TYPE_FUEL           0x06 // Remaining battery percentage
#define IBUS_MEAS_TYPE_ROLL           0x0f // Roll
#define IBUS_MEAS_TYPE_PITCH          0x10 // Pitch
#define IBUS_MEAS_TYPE_VERTICAL_SPEED 0x12 // Vertical speed
#define IBUS_MEAS_TYPE_ARMED          0x15 // Armed / unarmed
#define IBUS_MEAS_TYPE_PRES           0x41 // Pressure
*/
    struct SensorInfo_t
    {
        uint8_t sensorType;   // sensor type (0,1,2,3, etc)
        uint8_t sensorLength; // data length for defined sensor (can be 2 or 4)
        // int32_t sensorValue = -1; // sensor data for defined sensors (16 or 32 bits)
    };
    static constexpr SensorInfo_t sensorList[] = {
        {0x03, 2}, // IBUS_MEAS_TYPE_EXTV
        {0x41, 4}, // IBUS_MEAS_TYPE_PRES

        // {0x04, 2}, // IBUS_MEAS_TYPE_CELL
        // {0x06, 2}, // IBUS_MEAS_TYPE_FUEL

        // {0x0f, 2}, // IBUS_MEAS_TYPE_ROLL
        // {0x10, 2}, // IBUS_MEAS_TYPE_PITCH
        // {0x11, 2}, // IBUS_MEAS_TYPE_YAW

        // {0x12, 2}, // IBUS_MEAS_TYPE_VERTICAL_SPEED
        // {0x15, 2}, // IBUS_MEAS_TYPE_ARMED

    };

    static constexpr unsigned sensorListSize = sizeof(sensorList) / sizeof(SensorInfo_t);

    enum SensorList : unsigned
    {
        external_voltage,
        pressure,

        // avg_cell_voltage,
        // batt_percentage,

        // roll,
        // pitch,
        // yaw,

        // vertical_speed,
        // arm,

        // ax,
        // ay,
        // az,
    };
    int32_t sensorValues[sensorListSize];

    uint8_t telemetryBuffer[8];
    uint8_t telemetryLen = 0;
    bool needSendTelemetry = false;

public:
    static constexpr uint8_t channelCount = 12;

    bool parseData(uint8_t data[], size_t len, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable) override final;
    void sensorHandler() override final;
    bool sendTelemetry(uint8_t data[], size_t &count) override final;
};