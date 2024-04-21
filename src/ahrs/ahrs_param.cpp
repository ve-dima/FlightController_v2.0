#include "ahrs.hpp"
#include "param/param.hpp"

namespace AHRS
{
    float gyroscopeOffset[] = {-0.0070733559764921666, -0.011724833219312131, 0.015173626778647304};
    PARAM_ADD(param::FLOAT, CAL_GYRO_XOFF, &gyroscopeOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_YOFF, &gyroscopeOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_ZOFF, &gyroscopeOffset[2]);

    float accelerometerOffset[] = {0, 0, 0};
    PARAM_ADD(param::FLOAT, CAL_ACC_XOFF, &accelerometerOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_ACC_YOFF, &accelerometerOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_ACC_ZOFF, &accelerometerOffset[2]);

    float accelerationFilterGain;
    PARAM_ADD(param::FLOAT, IMU_ACC_GAIN, &accelerationFilterGain);

    float accelerationRejection;
    PARAM_ADD(param::FLOAT, IMU_ACC_RJT, &accelerationRejection);

    float accelerationRejectionAngle;
    PARAM_ADD(param::FLOAT, IMU_ACC_RJT_ANG, &accelerationRejectionAngle);
}