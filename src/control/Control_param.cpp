#include "Control.hpp"
#include "param/param.hpp"

namespace Control
{
    PIDSettings angleSettings = {
        .axis = {
            .roll = {
                .P = 6.5,
                .I = 0,
                .D = 0,
                .maxICoef = 0,
                .max = 220,
            },
            .pitch = {
                .P = 6.5,
                .I = 0,
                .D = 0,
                .maxICoef = 0,
                .max = 220,
            },
            .yaw = {
                .P = 2.8,
                .I = 0,
                .D = 0,
                .maxICoef = 0,
                .max = 200,
            }}};
    PIDSettings rateSettings = {
        .axis = {
            .roll = {
                .P = 0.15,
                .I = 0.2,
                .D = 0.003,
                .maxICoef = 0.15,
                .max = 1,
            },
            .pitch = {
                .P = 0.15,
                .I = 0.2,
                .D = 0.003,
                .maxICoef = 0.15,
                .max = 1,
            },
            .yaw = {
                .P = 0.2,
                .I = 0.1,
                .D = 0,
                .maxICoef = 0.30,
                .max = 1,
            }}};

    float minimalTrust = 0.1;

    void parameterUpdate()
    {
        angleSettings.axis.roll.P = std::clamp<float>(angleSettings.axis.roll.P, 0, 12);
        angleSettings.axis.pitch.P = std::clamp<float>(angleSettings.axis.pitch.P, 0, 12);
        angleSettings.axis.yaw.P = std::clamp<float>(angleSettings.axis.yaw.P, 0, 5);

        angleSettings.axis.roll.max = std::clamp<float>(angleSettings.axis.roll.max, 0, 1'800);
        angleSettings.axis.pitch.max = std::clamp<float>(angleSettings.axis.pitch.max, 0, 1'800);
        angleSettings.axis.yaw.max = std::clamp<float>(angleSettings.axis.yaw.max, 0, 1'800);

        rateSettings.axis.roll.P = std::clamp<float>(rateSettings.axis.roll.P, 0.01, 0.5);
        rateSettings.axis.roll.I = std::clamp<float>(rateSettings.axis.roll.I, 0, 0.5);
        rateSettings.axis.roll.D = std::clamp<float>(rateSettings.axis.roll.D, 0, 0.01);
        rateSettings.axis.roll.maxICoef = std::clamp<float>(rateSettings.axis.roll.maxICoef, 0, 1);

        rateSettings.axis.pitch.P = std::clamp<float>(rateSettings.axis.pitch.P, 0.01, 0.5);
        rateSettings.axis.pitch.I = std::clamp<float>(rateSettings.axis.pitch.I, 0, 0.5);
        rateSettings.axis.pitch.D = std::clamp<float>(rateSettings.axis.pitch.D, 0, 0.01);
        rateSettings.axis.pitch.maxICoef = std::clamp<float>(rateSettings.axis.pitch.maxICoef, 0, 1);

        rateSettings.axis.yaw.P = std::clamp<float>(rateSettings.axis.yaw.P, 0.01, 0.5);
        rateSettings.axis.yaw.I = std::clamp<float>(rateSettings.axis.yaw.I, 0, 0.5);
        rateSettings.axis.yaw.D = std::clamp<float>(rateSettings.axis.yaw.D, 0, 0.01);
        rateSettings.axis.yaw.maxICoef = std::clamp<float>(rateSettings.axis.yaw.maxICoef, 0, 1);

        minimalTrust = std::clamp<float>(minimalTrust, 0.05, 0.5);
    }

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_ROLL_P, &angleSettings.axis.roll.P, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_PITCH_P, &angleSettings.axis.pitch.P, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_YAW_P, &angleSettings.axis.yaw.P, parameterUpdate);

    //=================================

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_ROLLRATE_MAX, &angleSettings.axis.roll.max, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_PITCHRATE_MAX, &angleSettings.axis.pitch.max, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_YAWRATE_MAX, &angleSettings.axis.yaw.max, parameterUpdate);

    //=================================

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_ROLLRATE_P, &rateSettings.axis.roll.P, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_ROLLRATE_I, &rateSettings.axis.roll.I, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_ROLLRATE_D, &rateSettings.axis.roll.D, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_RR_INT_LIM, &rateSettings.axis.roll.maxICoef, parameterUpdate);

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_PITCHRATE_P, &rateSettings.axis.pitch.P, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_PITCHRATE_I, &rateSettings.axis.pitch.I, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_PITCHRATE_D, &rateSettings.axis.pitch.D, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_PR_INT_LIM, &rateSettings.axis.pitch.maxICoef, parameterUpdate);

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_YAWRATE_P, &rateSettings.axis.yaw.P, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_YAWRATE_I, &rateSettings.axis.yaw.I, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_YAWRATE_D, &rateSettings.axis.yaw.D, parameterUpdate);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MC_YR_INT_LIM, &rateSettings.axis.yaw.maxICoef, parameterUpdate);

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, MPC_THR_MIN, &minimalTrust, parameterUpdate);
}