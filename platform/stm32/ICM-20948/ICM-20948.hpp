#pragma once

namespace ICM20948
{
    // static constexpr float maxGyroscopeNoise;

    inline bool magIsRead = false;
    void handler();

    bool isOK();
};