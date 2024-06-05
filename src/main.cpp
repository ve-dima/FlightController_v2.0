#include "Common.hpp"
#include "SRT/SRT.hpp"
#include "modes/Modes.hpp"

int main()
{
    setup();
    SRT::init();

    SRT::enable();

    while (true)
    {
        loop();
        SRT::handler();
        FlightModeDispatcher::handler();
    }
}