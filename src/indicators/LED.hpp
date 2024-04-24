#pragma once

namespace LED
{
    enum class Color
    {
        off,
        green,
        red,
    };

    enum class Action
    {
        slow_blink,
        short_blink,
        double_short_blink,
        blink,
        fast_blink,
        off
    };

    inline Color color = Color::off;
    inline Action action = Action::off;

    void setLED(Color color, Action action);
    Color getColor();
    Action getAction();

    void handler();
    void setState(bool);
}