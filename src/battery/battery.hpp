#pragma once

namespace Battery
{
    unsigned getCellCount();

    float getVoltage();
    float getVoltagePerCell();

    float getPercent();

    void updateByADC(float voltage);
}