#include "Control.hpp"
#include "ahrs/ahrs.hpp"
#include "math/math.hpp"
#include "motor/motor.hpp"
#include "stm32g4xx.h"

namespace Control
{
    static constexpr float iReducerMaxRate = 400 * (M_PI / 180);

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
        outPower.backLeft -= targetTrustVector.x();
        outPower.frontLeft -= targetTrustVector.x();
        outPower.backRight += targetTrustVector.x();

        outPower.frontRight -= targetTrustVector.y();
        outPower.backLeft += targetTrustVector.y();
        outPower.frontLeft -= targetTrustVector.y();
        outPower.backRight += targetTrustVector.y();

        outPower.frontRight += targetTrustVector.z();
        outPower.backLeft += targetTrustVector.z();
        outPower.frontLeft -= targetTrustVector.z();
        outPower.backRight -= targetTrustVector.z();

        // float minThrust = INFINITY, maxTrust = -INFINITY;
        // for (float &p : outPower.array)
        //     minThrust = std::min(minThrust, p),
        //     maxTrust = std::max(maxTrust, p);

        // const float len = maxTrust - minThrust,
        //             maxLen = (1 - minimalTrust);
        // if (len > maxLen)
        // {
        //     const float multiplier = (1 - minimalTrust) / len;
        //     for (float &p : outPower.array)
        //         p = (p - minThrust) * multiplier + minimalTrust;
        // }
        // else
        // {
        //     const float adder = std::max(minimalTrust,
        //                                  std::min<float>((1 - len), 0));
        //     for (float &p : outPower.array)
        //         p = (p - minThrust) + adder;
        // }

        for (int i = 0; i < 4; i++)
            outPower.array[i] = std::clamp<float>(outPower.array[i] + targetThrust, minimalTrust, 1);

        bool wrongVal = false;
        for (int i = 0; i < 4; i++)
            if (not std::isfinite(outPower.array[i]))
                wrongVal = true;
        if (wrongVal)
        {
            for (int i = 0; i < 4; i++)
                Motor::setPower(i, NAN);
            __BKPT(0);
        }

        for (int i = 0; i < 4; i++)
            Motor::setPower(i, targetThrust);
    }

    /// Первый каскад управления - PID-контроллер скорости вращения
    void rateHandler()
    {
        Eigen::Vector3f target;
        const Eigen::Vector3f rateError = targetRate - AHRS::getFRD_RSpeed();
        const Eigen::Vector3f rateAcc = AHRS::getRAcceleration();
        const float dt = AHRS::lastDT;

        for (int axis = 0; axis < 3; axis++)
        {
            float iReduceFactor = 1 - (rateError[axis] * rateError[axis]) / iReducerMaxRate;
            if (iReduceFactor < 0 or targetThrust < 0.1)
                iReduceFactor = 0;
            target[axis] = ratePid.pids[axis].calculate(rateError[axis], rateAcc[axis], iReduceFactor, dt);
        }

        targetTrustVector = target;
    }

    /// Второй каскад управления - P-контроллер наклонов
    void rotateVelocityHandler()
    {
        // return;
        Eigen::Vector3f target = targetRate;
        Eigen::Quaternionf attitude = AHRS::getFRD_Attitude();
        Eigen::Quaternionf qd = targetAttitude;

        // // calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
        const Eigen::Vector3f e_z = dcm_z(attitude);
        const Eigen::Vector3f e_z_d = dcm_z(qd);
        Eigen::Quaternionf q_tiltError = from2vec(e_z, e_z_d);

        if (std::abs(q_tiltError.x()) > (1.f - 1e-5f) or
            std::abs(q_tiltError.y()) > (1.f - 1e-5f))
        {
            // In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
            // full attitude control anyways generates no yaw input and directly takes the combination of
            // roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
            q_tiltError = qd;
        }
        else
        {
            // transform rotation from current to desired thrust vector into a world frame reduced desired attitude
            q_tiltError *= attitude;
        }

        // mix full and reduced desired attitude
        Eigen::Quaternionf q_mix = q_tiltError.inverse() * qd;
        // q_mix.canonicalize();
        // catch numerical problems with the domain of acosf and asinf
        q_mix.w() = std::clamp(q_mix.w(), -1.f, 1.f);
        q_mix.z() = std::clamp(q_mix.z(), -1.f, 1.f);
        qd = q_tiltError * Eigen::Quaternionf(std::cos(yawWeight * std::acos(q_mix.w())),
                                              0,
                                              0,
                                              std::sin(yawWeight * std::asin(q_mix.z())));

        // quaternion attitude control law, qe is rotation from q to qd

        Eigen::Vector3f angleError = (attitude.conjugate() * qd).vec();
        angleError *= 2.f;
        if (attitude.w() < 0)
            angleError = -angleError;

        for (int axis = 0; axis < 3; axis++)
            target[axis] = anglePid.pids[axis].calculate(angleError[axis], 0, 0);
        // only P-part, I- & D- disabled

        targetRate = target;
    }

    /// Третий каскад управления - PID-контроллер скорости
    void linearVelocityHandler()
    {
        if(trustMode == TrustMode::MANUAL)
            return;
        
    }
}
