#pragma once
#include "Modes.hpp"

class Disarm : public FlightMode
{
public:
    virtual const char *const name() override final { return "disarm"; }

    bool needEnter(const char *&reason) override final;
    void onEnter() override final;
    bool canExit(const char *&err) override final;

    void handler() override final;
};

extern Disarm disarmMode;