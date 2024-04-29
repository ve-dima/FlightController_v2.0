#include "LED.hpp"
#include <cstdint>
#include "Common.hpp"

namespace LED
{
    struct Action_tick
    {
        bool state;
        uint32_t delayTime;
    };

    struct Action_desc
    {
        const Action_tick *const actionTicks;
        uint8_t actionCount;
    };

    constexpr Action_tick slow_blink[] = {
        {true, 3000},
        {false, 3000},
    };
    constexpr Action_tick short_blink[] = {
        {true, 100},
        {false, 1000},
    };
    constexpr Action_tick double_short_blink[] = {
        {true, 100},
        {false, 100},
        {true, 100},
        {false, 1000},
    };
    constexpr Action_tick blink[] = {
        {true, 1000},
        {false, 1000},
    };
    constexpr Action_tick fast_blink[] = {
        {true, 100},
        {false, 100},
    };
    constexpr Action_tick off[] = {
        {false, UINT32_MAX},
    };

    constexpr Action_desc actions[] = {
        {.actionTicks = slow_blink, .actionCount = sizeof(slow_blink) / sizeof(Action_tick)},
        {.actionTicks = short_blink, .actionCount = sizeof(short_blink) / sizeof(Action_tick)},
        {.actionTicks = double_short_blink, .actionCount = sizeof(double_short_blink) / sizeof(Action_tick)},
        {.actionTicks = blink, .actionCount = sizeof(blink) / sizeof(Action_tick)},
        {.actionTicks = fast_blink, .actionCount = sizeof(fast_blink) / sizeof(Action_tick)},
        {.actionTicks = off, .actionCount = sizeof(off) / sizeof(Action_tick)},
    };

    uint8_t actionTickCounter = 0;
    uint32_t tickTimer = 0;

    void setLED(Color c, Action a)
    {
        actionTickCounter = 0;
        color = c;
        action = a;
        tickTimer = millis();

        handler();
    }

    void handler()
    {
        const auto &act = actions[static_cast<int>(action)];
        const auto &tick = act.actionTicks[actionTickCounter];

        if (millis() - tickTimer < tick.delayTime)
            return;
        tickTimer = millis();

        setState(tick.state);

        actionTickCounter++;
        if (actionTickCounter >= act.actionCount)
            actionTickCounter = 0;
    }

    Color getColor() { return color; }
    Action getAction() { return action; }
}
