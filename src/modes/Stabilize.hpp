/**
 * @file Stabilize.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Режим стабилизации углов по пульту
 */
#pragma once
#include "Modes.hpp"
#include "Utility.hpp"

class Stabilize : public FlightMode
{
private:
    uint32_t stickTimer;
    EulerF setAngle;
    bool stickWasOnCenter = false;

public:
    virtual const char *const name() { return "stabilize"; }

    bool needEnter(const char *&reason) override final;
    void onEnter() override final;

    void handler() override final;
    void attitudeTickHandler() override final;

    void onExit() override final;
};

extern Stabilize stabilizeMode;