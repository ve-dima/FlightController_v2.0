#pragma once
#include <cstdint>

namespace SRT
{
    struct Module
    {
        void (*const init)(void);
        void (*const enable)(void);
        void (*const handler)(void);
    };

    void init();
    void enable();
    void handler();
}

#define REGISTER_SRT_MODULE(moduleName, initFunction, enableFunction, handlerFunction) \
    static const SRT::Module module_##moduleName __attribute__((section("modules_list"), __used__)) = {initFunction, enableFunction, handlerFunction}
