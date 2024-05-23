#include "ahrs.hpp"
#include "param/param.hpp"

namespace AHRS
{
    float gyroscopeOffset[] = {-0.00852211565, -0.00426105782, -0.0138484379};
    PARAM_ADD(param::FLOAT, CAL_GYRO_XOFF, &gyroscopeOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_YOFF, &gyroscopeOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_ZOFF, &gyroscopeOffset[2]);

    float accelerometerOffset[] = {0, 0, 0.02};
    PARAM_ADD(param::FLOAT, CAL_ACC_XOFF, &accelerometerOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_ACC_YOFF, &accelerometerOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_ACC_ZOFF, &accelerometerOffset[2]);

    float accelerationFilterGain = 0.01;
    PARAM_ADD(param::FLOAT, AHRS__ACC_GAIN, &accelerationFilterGain);

    float accelerationRejection = 0.5;
    PARAM_ADD(param::FLOAT, AHRS_ACC_RJT, &accelerationRejection);

    // float accelerationRejectionAngle;
    // PARAM_ADD(param::FLOAT, IMU_ACC_RJT_ANG, &accelerationRejectionAngle);

    float accelerometerNoise = 0.35;
    PARAM_ADD(param::FLOAT, AHRS_ACC_NOISE, &accelerometerNoise);

    float barometerNoise = 2;
    PARAM_ADD(param::FLOAT, AHRS_BARO_NOISE, &barometerNoise);

    int32_t mulka = 1'000;
    PARAM_ADD(param::INT32, AHRS_EKF_Q, &mulka);
}