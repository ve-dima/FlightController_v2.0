#include "atmosphere.hpp"
#include <cmath>

// NOTE: We are currently only modelling the first layer of the US Standard Atmosphere 1976.
// This means that the functions are only valid up to an altitude of 11km.
static constexpr float kAirDensitySeaLevelStandardAtmos = 1.225f; // kg/m^3

// [kg/m^3] air density of standard atmosphere at 11000m above mean sea level (this is the upper limit for the standard
// atmosphere model we are using, see atmosphere lib used)
static constexpr float kAirDensityStandardAtmos11000Amsl = 0.3639;

static constexpr float kAirGasConstant = 287.1f;                     // J/(kg * K)
static constexpr float kAbsoluteNullCelsius = -273.15f;              // °C
static constexpr float kTempRefKelvin = 15.f - kAbsoluteNullCelsius; // temperature at base height in Kelvin
static constexpr float kTempGradient = -6.5f / 1000.f;               // temperature gradient in degrees per meter
static constexpr float kPressRefSeaLevelPa = 101325.f;               // pressure at sea level in Pa

float getAltitudeFromPressure(float pressure_pa, float pressure_sealevel_pa)
{
    // calculate altitude using the hypsometric equation

    const float pressure_ratio = pressure_pa / pressure_sealevel_pa;

    /*
     * Solve:
     *
     *     /        -(aR / g)     \
     *    | (p / p1)          . T1 | - T1
     *     \                      /
     * h = -------------------------------  + h1
     *                   a
     */
    return (((powf(pressure_ratio, (-(kTempGradient * kAirGasConstant) / 9.81))) * kTempRefKelvin) -
            kTempRefKelvin) /
           kTempGradient;
}