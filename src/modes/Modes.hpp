#pragma once
#include <array>

/// @brief Режим полёта
class FlightMode
{
public:
    /// @brief Наименование режима
    virtual const char *const name() = 0;

    /// @brief Запрос на вход в режим
    /// @param[out] reason Причина, по которой требуется вход в режим
    /// @return Необходимость перейти в режим
    virtual bool needEnter(const char *&reason);

    /// @brief Разрешение на переход в режим
    /// @param[out] err Причина запрета
    /// @return Разрешение на переход
    virtual bool canEnter(const char *&err);

    /// @brief Обработчик при вхождении в режим
    virtual void onEnter();

    /// @brief "Тяжёлый" обработчик для супер-цикла
    virtual void handler();

    /// @brief "Лёгкий" обработчик, вызываемый каждый тик обновления курсовертикали
    virtual void attitudeTickHandler();

    /// @brief Обработчик покидания режима
    virtual void onExit();

    /// @brief Разрешение на выход из режима
    /// @param[out] err Причина запрета
    /// @return Разрешение на выход
    virtual bool canExit(const char *&err);

    /// @brief Запрос на выход из режима
    /// @param[out] reason Причина выхода
    /// @param[out] to Режим, в которой необходимо перейти
    /// @return Необходимость выйти из режима
    virtual bool needExit(const char *&reason, FlightMode *&to);
};

/// @brief Диспетчер режимов
namespace FlightModeDispatcher
{
    /// @brief Перечень режимов
    enum class flightModes
    {
        disarm,
        fault,
        stabilize,
        gyroscopeOffsetCalibrate,
        direct,
    };
    /// @brief Дефолтный режим
    inline constexpr flightModes defaultFlightMode = flightModes::disarm;

    FlightMode *getCurrentFlightMode();

    /// @brief Обработчик переключения между режимами
    void switchHandler();

    /// @brief Обработчик текущего режима
    void handler();

    /// @brief Обработчик обновления курсовертикали
    void attitudeTickHandler();

    /// @brief Результат смены режима
    enum class changeResult
    {
        ok = 0,
        cant_exit_from_current_mode,
        cant_enter_in_new_mode,
    };

    /// @brief Смена режима
    /// @param newFlightMode Режим, в который необходимо перейти
    /// @param[out] error Описание ошибки
    changeResult changeFlightMode(flightModes newFlightMode, const char *error);

    /// @brief Принудительная смена режима
    /// @param newFlightMode Новый режим
    void forceChangeMode(flightModes newFlightMode);
};