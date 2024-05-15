#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>

/**
 * @brief PID-регулятор
 * @tparam type_t Вычисляемый тип
 */
template <typename type_t>
class PID
{
public:
    struct Settings
    {
        type_t P = 0,
               I = 0,
               D = 0,
               maxICoef = 0.2,
               max = 1;
    };

private:
    type_t integral = 0;
    const Settings &settings;
    type_t output = 0;

public:
    explicit PID(const Settings &settings) : settings(settings) {}

    /**
     * @brief Вычисление выхода
     * @param error Ошибка
     * @param change Изменение (для расчёта D-части)
     * @return type_t Выход
     */
    type_t calculate(const type_t error, const type_t change, const float integralReducerFactor = 1, const float dt = 1)
    {
        integral += error * settings.I * integralReducerFactor;
        const type_t pPart = error * settings.P;
        const type_t dPart = change * settings.D;
        integral = std::clamp(integral, -settings.maxICoef, settings.maxICoef);

        output = pPart - integral - dPart;
        output = std::clamp(output, -settings.max, settings.max);

        return output;
    }

    /**
     * @brief Сброс выхода и интегральной части
     */
    void reset() { integral = output = 0; }

    type_t getOutput() { return output; }
    type_t getIntegral() { return integral; }
};

using PIDf = PID<float>;