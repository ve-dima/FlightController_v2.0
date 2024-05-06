#include "Common.hpp"
#include "mavlink_bridge_header.hpp"
#include "SRT/SRT.hpp"
#include "modes/Modes.hpp"

mavlink_system_t mavlink_system = {.sysid = 81, .compid = 50};

int main()
{
    setup();
    SRT::init();

    SRT::enable();

    while (1)
    {
        loop();
        SRT::handler();
        FlightModeDispatcher::handler();
    }
}