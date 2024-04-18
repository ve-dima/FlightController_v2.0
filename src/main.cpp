#include "Common.hpp"
#include "param/param.hpp"
#include "SRT/SRT.hpp"

int main()
{
    setup();
    SRT::init();

    SRT::enable();

    while (1)
    {
        loop();
        SRT::handler();
    }
}