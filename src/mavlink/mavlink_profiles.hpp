#pragma once
#include "mavlink/common/mavlink.h"

namespace MavLinkProfiles
{
    struct Report
    {
        void (*callback)(mavlink_channel_t channel);
        float rate;
    };

    struct Profile
    {
        const Report *const arr;
        unsigned size;
    };

    extern Profile normal;
    extern Profile lowBandwidth;
    extern Profile highBandwidth;
};