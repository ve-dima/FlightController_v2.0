/**
 * @file Stabilize.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Режим стабилизации углов по пульту
 */
#pragma once
#include "Modes.hpp"
#include <Eigen/Geometry>

class Stabilize : public FlightMode
{
    bool inGyroscopeCalibration;
    unsigned gyroscopeSamples;
    float manualYawSetPoint;
    float targetAlt = NAN;
    Eigen::Quaternionf homeYaw;
    Eigen::Quaternionf acroSP = Eigen::Quaternionf::Identity();

public:
    virtual const char *const name() { return "stabilize"; }

    bool needEnter(const char *&reason) override final;
    void onEnter() override final;

    void attitudeTickHandler() override final;
    void handler() override final;

private:
    Eigen::Quaternionf getSPFromRC();
    float getThrottleFromRC();

    void levelMode();
    void altMode();
    void acroMode();
};

extern Stabilize stabilizeMode;