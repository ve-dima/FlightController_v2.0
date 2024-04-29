#include "SRT.hpp"
#include "Common.hpp"
#include <algorithm>

extern const SRT::Module __start_modules_list;
extern const SRT::Module __stop_modules_list;

void SRT::init()
{
    for (const Module *ptr = &__start_modules_list; ptr != &__stop_modules_list; ++ptr)
        ptr->init();
}

void SRT::enable()
{
    for (const Module *ptr = &__start_modules_list; ptr != &__stop_modules_list; ++ptr)
        ptr->enable();
}

void SRT::handler()
{
    static uint32_t maxClk = 0;
    uint32_t startTime = tick();
    for (const Module *ptr = &__start_modules_list; ptr != &__stop_modules_list; ++ptr)
        ptr->handler();
    maxClk = std::max(maxClk, tick() - startTime);
}