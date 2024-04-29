#pragma once
#include <cstdint>
#include "RC.hpp"

class CRSF : public RC_parser
{
private:
    static constexpr unsigned CRSF_FRAME_SIZE_MAX = 32; // the actual maximum length is 64, but we're only interested in RC channels and want to minimize buffer size
    static constexpr unsigned CRSF_PAYLOAD_SIZE_MAX = (CRSF_FRAME_SIZE_MAX - 4);
    static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

    struct crsf_frame_header_t
    {
        uint8_t device_address; ///< @see crsf_address_t
        uint8_t length;         ///< length of crsf_frame_t (including CRC) minus sizeof(crsf_frame_header_t)
    };

    struct crsf_frame_t
    {
        crsf_frame_header_t header;
        uint8_t type;                               ///< @see crsf_frame_type_t
        uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; ///< payload data including 1 byte CRC at end
    };

    enum class crsf_parser_state_t : uint8_t
    {
        unsynced = 0,
        synced
    };

    crsf_frame_t crsf_frame;
    unsigned current_frame_position = 0;
    crsf_parser_state_t parser_state = crsf_parser_state_t::unsynced;

    enum class crsf_frame_type_t : uint8_t
    {
        gps = 0x02,
        battery_sensor = 0x08,
        link_statistics = 0x14,
        rc_channels_packed = 0x16,
        attitude = 0x1E,
        flight_mode = 0x21,

        // Extended Header Frames, range: 0x28 to 0x96
        device_ping = 0x28,
        device_info = 0x29,
        parameter_settings_entry = 0x2B,
        parameter_read = 0x2C,
        parameter_write = 0x2D,
        command = 0x32
    };

    enum class crsf_payload_size_t : uint8_t
    {
        gps = 15,
        battery_sensor = 8,
        link_statistics = 10,
        rc_channels = 22, ///< 11 bits per channel * 16 channels = 22 bytes.
        attitude = 6,
    };

    struct crsf_payload_RC_channels_packed_t
    {
        // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes
        unsigned chan0 : 11;
        unsigned chan1 : 11;
        unsigned chan2 : 11;
        unsigned chan3 : 11;
        unsigned chan4 : 11;
        unsigned chan5 : 11;
        unsigned chan6 : 11;
        unsigned chan7 : 11;
        unsigned chan8 : 11;
        unsigned chan9 : 11;
        unsigned chan10 : 11;
        unsigned chan11 : 11;
        unsigned chan12 : 11;
        unsigned chan13 : 11;
        unsigned chan14 : 11;
        unsigned chan15 : 11;
    } __attribute__((packed));
    static_assert(sizeof(crsf_payload_RC_channels_packed_t) == 22, "Wrong struct size");

public:
    static constexpr uint8_t maxChannelCount = 16;
    bool parseData(uint8_t data[], size_t len, bool parityError, int16_t channels[], unsigned &channelCount, uint8_t &rssi, bool &signalAvailable) override final;
    // virtual void sendTelemetry(uint8_t data[], uint8_t &count) override final;

    bool crsf_parse(const uint8_t *frame, unsigned len, int16_t *values,
                    unsigned *num_values, unsigned max_channels);
    bool crsf_parse_buffer(int16_t *values, unsigned *num_values, unsigned max_channels);
    uint8_t crsf_frame_CRC(const crsf_frame_t &frame);

    void write_frame_header(uint8_t *buf, int &offset, crsf_frame_type_t type, uint8_t payload_size);
    void crsf_send_telemetry_battery(uint8_t data[], uint8_t &size,uint16_t voltage, uint16_t current, int fuel, uint8_t remaining);
    void crsf_send_telemetry_gps(uint8_t data[], uint8_t &size,int32_t latitude, int32_t longitude, uint16_t groundspeed,
                                 uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites);
    void crsf_send_telemetry_attitude(uint8_t data[], uint8_t &size,int16_t pitch, int16_t roll, int16_t yaw);
    void crsf_send_telemetry_flight_mode(uint8_t data[], uint8_t &size,const char *flight_mode);
};