#include <cstdint>
#include "Modes.hpp"
#include "Common.hpp"
#include "ArduinoAPI/Print.h"
#include "SRT/SRT.hpp"
#include "Board.hpp"

bool FlightMode::needEnter(const char *&reason)
{
    reason = nullptr;
    return false;
}

bool FlightMode::canEnter(const char *&err)
{
    err = nullptr;
    return true;
}

void FlightMode::attitudeTickHandler() {}

void FlightMode::onEnter() {}

void FlightMode::handler() {}

bool FlightMode::canExit(const char *&err)
{
    err = nullptr;
    return true;
}

bool FlightMode::needExit(const char *&reason, FlightMode *&to)
{
    reason = nullptr;
    return false;
}

void FlightMode::onExit() {}

#include "Disarm.hpp"
#include "Stabilize.hpp"
// #include "GyroscopeOffsetCalibrate.hpp"
// #include "Direct.hpp"
// #include "Fault.hpp"

namespace FlightModeDispatcher
{
    static constexpr FlightMode *const modes[] =
        {
            &disarmMode,
            // &faultMode,
            &stabilizeMode,
            // &gyroscopeOffsetCalibrateMode,
            // &directMode,
    };
    static constexpr uint8_t flightModesCount = sizeof(modes) / sizeof(modes[0]);

    FlightMode *currentFlightMode = modes[static_cast<uint32_t>(defaultFlightMode)];

    FlightMode *getCurrentFlightMode() { return currentFlightMode; }

    void forceChangeMode(FlightMode *const newFM)
    {
        currentFlightMode->onExit();
        currentFlightMode = newFM;
        currentFlightMode->onEnter();
    }

    changeResult changeFlightMode(FlightMode *const newFM, const char *error)
    {
        if (currentFlightMode == newFM)
            return changeResult::ok;

        if (not currentFlightMode->canExit(error))
            return changeResult::cant_exit_from_current_mode;

        if (not newFM->canEnter(error))
            return changeResult::cant_enter_in_new_mode;

        forceChangeMode(newFM);

        return changeResult::ok;
    }

    changeResult changeFlightMode(flightModes newFlightMode, const char *error)
    {
        FlightMode *const newFM = modes[static_cast<uint32_t>(newFlightMode)];
        return changeFlightMode(newFM, error);
    }

    void forceChangeMode(flightModes newFlightMode)
    {
        FlightMode *const newFM = modes[static_cast<uint32_t>(newFlightMode)];
        forceChangeMode(newFM);
    }

    void printSwitchInfo(Print &printer, FlightMode *const fm, const char *const enterReason)
    {
        printer.print("FM change: ");
        printer.print(currentFlightMode->name());
        printer.print(" -> ");
        printer.println(fm->name());
        
        if (enterReason)
            printer.print("Reason: "),
                printer.println(enterReason);
    }

    void switchHandler()
    {
        const char *dummy;
        if (currentFlightMode == nullptr or currentFlightMode->canExit(dummy))
            for (auto &i : modes)
            {
                if (i == currentFlightMode)
                    continue;

                const char *enterReason = nullptr;

                if (not i->needEnter(enterReason))
                    continue;

                printSwitchInfo(debugUart, i, enterReason);

                forceChangeMode(i);
                break;
            }

        if (currentFlightMode != nullptr)
        {
            FlightMode *newFM = nullptr;
            const char *exitReason = nullptr, *dummy;

            if (not currentFlightMode->needExit(exitReason, newFM))
                return;

            if (newFM == nullptr or not newFM->canEnter(dummy))
            {
                forceChangeMode(defaultFlightMode);
                return;
            }

            forceChangeMode(newFM);
        }
    }

    void handler()
    {
        if (currentFlightMode != nullptr)
            currentFlightMode->handler();
    }

    void attitudeTickHandler()
    {
        if (currentFlightMode != nullptr)
            currentFlightMode->attitudeTickHandler();
    }

    void init(){}
    void enable(){}
    void h()
    {
        switchHandler();
        handler();
    }
    REGISTER_SRT_MODULE(flightModeDispatcher, init, enable, h);
}
