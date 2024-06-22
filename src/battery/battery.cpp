#include "battery.hpp"
#include "param/param.hpp"
#include <utility>
#include <algorithm>
#include <cmath>

namespace Battery
{
    constexpr float voltageFilter = 0.985;
    int32_t cellCount = 4;

    float filteredVoltage = NAN;

    unsigned getCellCount() { return cellCount; }
    float getVoltage() { return filteredVoltage; }
    float getVoltagePerCell() { return filteredVoltage / cellCount; }

    void updateVoltage(float voltage)
    {
        if (not std::isfinite(filteredVoltage))
        {
            filteredVoltage = voltage;
            return;
        }

        filteredVoltage = voltageFilter * filteredVoltage + (1 - voltageFilter) * voltage;
    }

    constexpr std::pair<float, float> percentTable[] = {
        {4.20, 1.0},
        {4.15, .95},
        {4.11, .90},
        {4.08, .85},
        {4.02, .80},
        {3.98, .75},
        {3.95, .70},
        {3.91, .65},
        {3.87, .60},
        {3.85, .55},
        {3.84, .50},
        {3.82, .45},
        {3.80, .40},
        {3.79, .35},
        {3.77, .30},
        {3.75, .25},
        {3.73, .20},
        {3.71, .15},
        {3.69, .10},
        {3.61, .05},
        {3.27, .00},
    };
    constexpr unsigned percentTableSize = sizeof(percentTable) / sizeof(percentTable[0]);
    static_assert([]() constexpr -> bool
                                        {
        for (unsigned i = 1; i < percentTableSize; i++)
            if (percentTable[i - 1].first <= percentTable[i].first or
                percentTable[i - 1].second <= percentTable[i].second)
                return false;
        return true; }() == true,
                  "Wrong table");

    constexpr float maxVoltage = []() constexpr -> float
    {
        float max = -INFINITY;
        for (auto e : percentTable)
            max = std::max(max, e.first);
        return max;
    }();
    constexpr float minVoltage = []() constexpr -> float
    {
        float min = INFINITY;
        for (auto e : percentTable)
            min = std::min(min, e.first);
        return min;
    }();

    float getPercent()
    {
        const float voltage = getVoltagePerCell();

        if (voltage >= maxVoltage)
            return 1;
        else if (voltage <= minVoltage)
            return 0;

        const std::pair<float, float> *upper = percentTable;
        for (auto &e : percentTable)
            if (voltage >= e.first)
            {
                upper = &e - 1;
                break;
            }

        const std::pair<float, float> *lower = (upper == &percentTable[percentTableSize - 1] ? &percentTable[percentTableSize - 1] : upper + 1);
        const float t = 1 - ((upper->first - voltage) / (upper->first - lower->first));

        return std::clamp<float>(lower->second + t * (upper->second - lower->second), 0, 1);
    }

    PARAM_ADD(param::INT32, BAT1_N_CELLS, &cellCount);
}