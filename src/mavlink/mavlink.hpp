#pragma once
#include "mavlink_profiles.hpp"
#include "mavlink_bridge_header.hpp"

mavlink_system_t mavlink_system = {.sysid = 81, .compid = 50};

// class MavLinkReporter
// {
// private:
//     const mavlink_channel_t channel;
//     const MavLinkProfiles::Profile &profile;
//     const unsigned profileSize;
//     const uint32_t timers[16];

// public:
//     MavLinkReporter(mavlink_channel_t channel, MavLinkProfiles::Profile &profile);
// };