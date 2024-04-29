// #pragma once
// #include "mavlink/common/mavlink.h"

// class MavLinkReporter
// {
// private:
//     const mavlink_channel_t channel;
//     const size_t maxBandwidth;

// public:
//     struct Report
//     {
//         uint32_t period;
//         void (*reporter)(mavlink_channel_t);
//         size_t packetSize;
//     };
//     struct ReportProfile
//     {
//         Report *reports;
//         uint8_t reportCount;
//     };
// };