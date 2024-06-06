#include "ahrs.hpp"
#include "param/param.hpp"

namespace AHRS
{
    float gyroscopeOffset[] = {0, 0, 0};
    PARAM_ADD(param::FLOAT, CAL_GYRO_XOFF, &gyroscopeOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_YOFF, &gyroscopeOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_GYRO_ZOFF, &gyroscopeOffset[2]);

    float accelerometerOffset[] = {0, 0, 0};
    PARAM_ADD(param::FLOAT, CAL_ACC_XOFF, &accelerometerOffset[0]);
    PARAM_ADD(param::FLOAT, CAL_ACC_YOFF, &accelerometerOffset[1]);
    PARAM_ADD(param::FLOAT, CAL_ACC_ZOFF, &accelerometerOffset[2]);

    float accelerationFilterGain = 0.01;
    PARAM_ADD(param::FLOAT, AHRS_ACC_GAIN, &accelerationFilterGain);

    float accelerationRejection = 0.3;
    PARAM_ADD(param::FLOAT, AHRS_ACC_RJT, &accelerationRejection);

    float magnetometerFilterGain = 0.01;
    PARAM_ADD(param::FLOAT, AHRS_MAG_GAIN, &magnetometerFilterGain);

    float accelerometerNoise = 0.5;
    PARAM_ADD(param::FLOAT, AHRS_ACC_NOISE, &accelerometerNoise);

    float barometerNoise = 1;
    PARAM_ADD(param::FLOAT, AHRS_BARO_NOISE, &barometerNoise);

    float mulka = 1;
    PARAM_ADD(param::FLOAT, AHRS_EKF_Q, &mulka);

    Eigen::Matrix3f Q{
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 1},
    };
    float q1 = 0, q2 = 0, q3 = 1;
    void s()
    {
        Q(0, 0) = q1;
        Q(1, 1) = q2;
        Q(2, 2) = q3;
    }
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, AHRS_POS_Q, &q1, s);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, AHRS_SPD_Q, &q2, s);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, AHRS_ACC_Q, &q3, s);

    float baroSpeedCoef = 0.01;
    float baroPosCoef = 0.01;
    PARAM_ADD(param::FLOAT, AHRS_BARO_SPD, &baroSpeedCoef);
    PARAM_ADD(param::FLOAT, AHRS_BARO_POS, &baroPosCoef);

    float
        magnetometerMaximum[3] = {
            54,
            20,
            47,
    },
        magnetometerMinimum[3] = {
            -30,
            -60,
            -33,
    };
    PARAM_ADD(param::FLOAT, CAL_MAG_XMAX, &magnetometerMaximum[0]);
    PARAM_ADD(param::FLOAT, CAL_MAG_YMAX, &magnetometerMaximum[1]);
    PARAM_ADD(param::FLOAT, CAL_MAG_ZMAX, &magnetometerMaximum[2]);
    PARAM_ADD(param::FLOAT, CAL_MAG_XMIN, &magnetometerMinimum[0]);
    PARAM_ADD(param::FLOAT, CAL_MAG_YMIN, &magnetometerMinimum[1]);
    PARAM_ADD(param::FLOAT, CAL_MAG_ZMIN, &magnetometerMinimum[2]);
}