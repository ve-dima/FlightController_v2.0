#include "Control.hpp"
#include "Eigen/Core"
#include "ahrs/ahrs.hpp"
#include "motor/motor.hpp"

namespace Control
{
    static constexpr float iReducerMaxRate = 400;

    Modes_t angleModes = {
        .axis = {
            .roll = AngleControlMode::angle,
            .pitch = AngleControlMode::angle,
            .yaw = AngleControlMode::velocity,
        }};

    union
    {
        struct
        {
            PIDf roll;
            PIDf pitch;
            PIDf yaw;
        } axis;
        std::array<PIDf, 3> pids;
    } ratePid = {.axis = {
                     .roll = PIDf(rateSettings.axis.roll),
                     .pitch = PIDf(rateSettings.axis.pitch),
                     .yaw = PIDf(rateSettings.axis.yaw),
                 }},
      anglePid = {.axis = {
                      .roll = PIDf(angleSettings.axis.roll),
                      .pitch = PIDf(angleSettings.axis.pitch),
                      .yaw = PIDf(angleSettings.axis.yaw),
                  }};

    float targetThrust = 0;
    Eigen::Vector3f targetTrustVector = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f targetRate = Eigen::Vector3f(0, 0, 0);
    Eigen::Quaternionf targetAttitude = Eigen::Quaternionf(1, 0, 0, 0);

    Eigen::Vector3f getTargetRate() { return targetRate; }
    Eigen::Quaternionf getTargetAttitude() { return targetAttitude; }
    float getTargetThrust() { return targetThrust; }
    Eigen::Vector3f getTargetThrustVector() { return targetTrustVector; }

    void setTargetRate(Eigen::Vector3f rate) { targetRate = rate; }
    void setTargetAttitude(Eigen::Quaternionf att) { targetAttitude = att; }
    void setTargetThrust(float trt) { targetThrust = trt; }

    void updateMotorPower()
    {
        union MotorPower
        {
            float array[4];
            struct
            {
                float
                    frontRight,
                    backLeft,
                    frontLeft,
                    backRight;
            };
        } outPower = {.array = {0, 0, 0, 0}};

        outPower.frontRight += targetTrustVector.x();
        outPower.frontRight -= targetTrustVector.y();
        outPower.frontRight -= targetTrustVector.z();

        outPower.backLeft -= targetTrustVector.x();
        outPower.backLeft += targetTrustVector.y();
        outPower.backLeft -= targetTrustVector.z();

        outPower.frontLeft += targetTrustVector.x();
        outPower.frontLeft += targetTrustVector.y();
        outPower.frontLeft += targetTrustVector.z();

        outPower.backRight -= targetTrustVector.x();
        outPower.backRight -= targetTrustVector.y();
        outPower.backRight += targetTrustVector.z();

        float minThrust = INFINITY, maxTrust = -INFINITY;
        for (float &p : outPower.array)
            minThrust = std::min(minimalTrust, p),
            maxTrust = std::max(maxTrust, p);

        const float len = maxTrust - minThrust,
                    maxLen = (1 - minimalTrust);
        if (len > maxLen)
        {
            const float multiplier = (1 - minimalTrust) / len;
            for (float &p : outPower.array)
                p = (p - minThrust) * multiplier + minimalTrust;
        }
        else
        {
            const float adder = std::max(minimalTrust,
                                         std::min((1 - len), targetThrust));
            for (float &p : outPower.array)
                p = (p - minThrust) + adder;
        }

        for (int i = 0; i < 4; i++)
            Motor::setPower(i, outPower.array[i]);
    }

    /// Первый каскад управления - PID-контроллер скорости вращения
    void rateHandler()
    {
        Eigen::Vector3f target{0, 0, 0};

        const Eigen::Vector3f rateError = targetRate - AHRS::getRSpeed();
        const Eigen::Vector3f rateAcc = AHRS::getRAcceleration();

        for (int axis = 0; axis < 3; axis++)
        {
            // float iReduceFactor = 1 - (rateError[axis] * rateError[axis]) / iReducerMaxRate;
            // if (iReduceFactor < 0)
            //     iReduceFactor = 0;
            target[axis] = ratePid.pids[axis].calculate(rateError[axis], rateAcc[axis], 0);
        }

        targetTrustVector = target;
    }

    /// Второй каскад управления - P-контроллер наклонов
    void velocityHandler()
    {
        Eigen::Vector3f target = targetRate;

        Eigen::Quaternionf attitude = AHRS::getFRU_Attitude();
        Eigen::Vector3f angleError = (attitude.conjugate() * targetAttitude).vec();
        angleError *= 2;
        if (attitude.w() < 0)
            angleError = -angleError;

        for (int axis = 0; axis < 3; axis++)
            if (angleModes.modes[axis] == AngleControlMode::angle)
                target[axis] = anglePid.pids[axis].calculate(angleError[axis], 0, 0);
        // only P-part, I- & D- disabled

        targetRate = target;
    }
}
