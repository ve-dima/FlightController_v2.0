#include <stm32g4xx.h>
#include "ICM-20948/ICM-20948.hpp"
#include "BMP280/BMP280.hpp"
#include "modes/Modes.hpp"
#include "control/Control.hpp"
#include "SRT/SRT.hpp"
#include "rc/RC.hpp"
#include "Common.hpp"
#include <algorithm>

#include "mavlink/common/mavlink.h"
#include "ahrs/ahrs.hpp"
#include "control/Control.hpp"

namespace IMU
{
    void init()
    {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
        __DMB();

        TIM6->PSC = 19;
        TIM6->ARR = 35590; // 224,77 Hz
        TIM6->DIER |= TIM_DIER_UIE;
        TIM6->CR1 |= TIM_CR1_CEN;

        NVIC_DisableIRQ(TIM6_DAC_IRQn);
        NVIC_SetPriority(TIM6_DAC_IRQn, 2);

        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
        GPIOC->MODER &= ~(GPIO_MODER_MODE13);
        GPIOC->MODER |= GPIO_MODER_MODE13_0;
        GPIOC->BSRR = GPIO_BSRR_BS13;
    }

    void enable()
    {
        GPIOC->BSRR = GPIO_BSRR_BS13;
        delay(300);
        GPIOC->BSRR = GPIO_BSRR_BR13;

        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }

    void handler()
    {
    }

    void mavlink_attitude_target_report(mavlink_channel_t ch)
    {
        Eigen::Quaternionf targetAttitude = Control::getTargetAttitude();
        const float tA[4] = {targetAttitude.w(), targetAttitude.x(), targetAttitude.y(), targetAttitude.z()};

        const Eigen::Vector3f targetRate = Control::getTargetRate();
        const float targetTrust = Control::getTargetThrust();

        mavlink_msg_attitude_target_send(ch, millis(),
                                         0, tA,
                                         targetRate.x(), targetRate.y(), targetRate.z(),
                                         targetTrust);
    }

    void mavlink_quat_report(mavlink_channel_t ch)
    {
        const Eigen::Quaternionf attitude = AHRS::getFRD_Attitude();
        // const Eigen::Quaternionf attitude = Control::getTargetAttitude();
        const Eigen::Vector3f rotateRate = AHRS::getFRD_RotateSpeed();

        mavlink_msg_attitude_quaternion_send(ch, millis(),
                                             attitude.w(), attitude.x(), attitude.y(), attitude.z(),
                                             rotateRate.x(), rotateRate.y(), rotateRate.z(), nullptr);
    }

    extern "C" void TIM6_DAC_IRQHandler(void)
    {
        TIM6->SR = ~TIM_SR_UIF;

        ICM20948::handler();
        if (not ICM20948::magIsRead)
            BMP280::handler();
        else
            ICM20948::magIsRead = false;

        RC::ahrsTickHandler();
        FlightModeDispatcher::switchHandler();
        FlightModeDispatcher::attitudeTickHandler();
        Control::positionControlHandler();
        Control::linearVelocityHandler();
        Control::rotateVelocityHandler();
        Control::rateHandler();
        Control::updateMotorPower();

        // for (static uint32_t tim = 0; millis() - tim > 50; tim = millis())
        //     mavlink_quat_report(MAVLINK_COMM_0), mavlink_quat_report(MAVLINK_COMM_1),
        //         mavlink_attitude_target_report(MAVLINK_COMM_0), mavlink_attitude_target_report(MAVLINK_COMM_1);
        // for (static uint32_t tim = 0; millis() - tim > 1'000; tim = millis())
        //     mavlink_msg_heartbeat_send(MAVLINK_COMM_1,
        //                                MAV_TYPE::MAV_TYPE_QUADROTOR,
        //                                MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC, 0, 0, 0);
    }

    REGISTER_SRT_MODULE(imu, init, enable, handler);
};