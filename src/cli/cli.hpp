#pragma once
#include <cstdint>

namespace cli
{
    struct cli_handler_t
    {
        const char *alias;
        void (*const callback)(uint8_t, char **);
    };

    inline constexpr unsigned maxArgCount = 10;

    bool cliHandler(char string[]);

#define REGISTER_COMMAND(alias, func)                                                                        \
    static const cli_handler_t handler_##alias __attribute__((section(".cli_handler.", #alias), __used__)) = \
        {                                                                                                    \
            .alias = #alias,                                                                                 \
            .callback = func};
}