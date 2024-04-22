#pragma once
#include <cstdint>

namespace param
{
    enum type_t : uint8_t
    {
        INT32,
        FLOAT,
        _end,
    };

    struct param_t
    {
        type_t type : 3;
        uint8_t readOnly : 1;
        uint8_t needSave : 1;

        const char *name;
        void *address;
        void (*callback)(void);
    } __attribute__((aligned(4)));

    struct paramVarId_t
    {
        uint16_t index;
        const param_t *ptr;
    };

    //===========================================

    bool getParamByName(const char name[], paramVarId_t &param);
    bool getParamByIndex(uint16_t index, paramVarId_t &param);

    bool updateParamByPtr(void *value, const param_t *param);
    bool updateParamByIndex(void *value, uint16_t index);
    bool updateParamByName(void *value, const char name[]);

    unsigned getParamCount();
};

#define PARAM_ADD_FULL(TYPE, NAME, IS_READONLY, NEED_SAVE, ADDRESS, CALLBACK)             \
    __attribute__((section(".param." #NAME), used, aligned(1))) static struct param::param_t __params_##NAME = \
        {                                                                                          \
            .type = TYPE,                                                                          \
            .readOnly = IS_READONLY,                                                               \
            .needSave = NEED_SAVE,                                                                 \
            .name = #NAME,                                                                         \
            .address = (void *)(ADDRESS),                                                          \
            .callback = CALLBACK,                                                                  \
    };

#define PARAM_ADD(TYPE, NAME, ADDRESS) \
    PARAM_ADD_FULL(TYPE, NAME, false, true, ADDRESS, nullptr)

#define PARAM_ADD_WITH_CALLBACK(TYPE, NAME, ADDRESS, CALLBACK) \
    PARAM_ADD_FULL(TYPE, NAME, false, true, ADDRESS, CALLBACK)
