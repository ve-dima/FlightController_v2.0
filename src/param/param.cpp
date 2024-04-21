#include "param.hpp"
#include <cstring>

extern const struct param::param_t __param_start;
extern const struct param::param_t __param_stop;

unsigned param::getParamCount() { return (&__param_stop) - (&__param_start); }

bool param::getParamByName(const char name[], param::paramVarId_t &param)
{
    unsigned L = 0;
    unsigned R = getParamCount() - 1;
    while (L <= R)
    {
        unsigned m = (L + R) / 2;
        auto i = strcmp(((&__param_start)[m]).name, name);
        if (i < 0)
            L = m + 1;
        else if (i > 0)
            R = m - 1;
        else
        {
            param.ptr = &((&__param_start)[m]);
            param.index = m;
            return true;
        }
    }

    param.ptr = nullptr;
    param.index = UINT16_MAX;
    return false;
}

bool param::getParamByIndex(const uint16_t index, paramVarId_t &param)
{
    if (index >= getParamCount())
    {
        param.ptr = nullptr;
        param.index = UINT16_MAX;
        return false;
    }

    param.ptr = &((&__param_start)[index]);
    param.index = index;
    return true;
}

bool param::updateParamByPtr(void *value, const param_t *param)
{
    if (param >= (&__param_stop) or param < &__param_start)
        return false;
    if (param->type > _end)
        return false;

    static constexpr unsigned valuesSize[] = {
        1, // UINT8
        1, // INT8
        4, // UINT32
        4, // INT32
        4, // FLOAT
    };
    memcpy(param->address, value, valuesSize[param->type]);
    if (param->callback)
        param->callback();

    return true;
}

bool param::updateParamByIndex(void *value, uint16_t index) { return updateParamByPtr(value, &((&__param_start)[index])); }

bool param::updateParamByName(void *value, const char name[])
{
    paramVarId_t param;
    if (getParamByName(name, param) == false)
        return false;
    return updateParamByPtr(value, param.ptr);
}
