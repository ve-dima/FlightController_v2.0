/**
 * @file RingBuff.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Кольцевой буфер с константным временем работы (в базовых функциях)
 */
#pragma once
#include <cinttypes>
#include <limits>
#include <algorithm>

/**
 * @brief Кольцевой буфер
 *
 * @tparam _size размер буфера. Обязан быть степенью двойки
 * @tparam type_t хранимый тип
 * @tparam index_t тип, используемый для индексов
 */
template <size_t _size = 16, typename type_t = uint8_t, typename index_t = uint16_t>
class RingBuffer
{
private:
    type_t data[_size];
    volatile index_t _readCount;
    volatile index_t _writeCount;
    static constexpr index_t mask = _size - 1;
    static constexpr index_t mask2 = 2 * _size - 1;

    static_assert((_size & (_size - 1)) == 0, "Size must be power of 2");
    static_assert((_size * 2) < std::numeric_limits<index_t>::max(), "index_t not enoughs");

public:
    RingBuffer() : _readCount(0), _writeCount(0) {}

    /// Максимальное количество элементов
    inline index_t maxSize()
    {
        return _size;
    }

    /**
     * @brief Добавить элемент в буфер
     *
     * @param[in] value Элемент
     * @return true Если было доступно место
     * @return false Если места не было
     */
    inline bool push(const type_t &value)
    {
        if (full())
            return false;
        data[_writeCount++ & mask] = value;
        return true;
    }

    /**
     * @brief Добавить элемент в буфер
     * @param[in] value Элементы
     * @param len Количество элементов
     * @return true Если было доступно место
     * @return false Если места не было
     */
    inline bool push(const type_t value[], const index_t len)
    {
        if (available() < len)
            return false;

        while (--len)
            data[_writeCount++ & mask] = value;

        return true;
    }

    /// @brief Вытолкнуть элементы
    /// @param len Количество элементов для выталкивания
    /// @return Количество вытолкнутых элементов
    inline index_t advance(index_t len)
    {
        len = std::min<index_t>(maxSize(), len);
        _readCount += len;
        return len;
    }

    /**
     * @brief Получить и вытолкнуть элемент
     * @param[out] value Элемент
     * @return true Элемент был в наличии
     * @return false Буфер пуст
     */
    inline bool pop(type_t &value)
    {
        if (empty())
            return false;
        value = data[_readCount++ & mask];
        return true;
    }

    /**
     * @brief Получить элемент
     * @return type_t type_t() если пусто
     * @return type_t Элемент
     */
    inline type_t pop()
    {
        if (empty())
            return type_t();

        return data[_readCount++ & mask];
    }

    /**
     * @brief Прочитать, но не выталкивать
     * @return type_t type_t() если пусто
     * @return type_t Элемент
     */
    [[nodiscard]] inline type_t peek() const
    {
        if (empty())
            return type_t();
        return data[_readCount & mask];
    }

    /**
     * @brief Получить НЕ выталкивая
     * @param[out] value Элемент
     * @return true Элемент был в наличии
     * @return false Буфер пуст
     */
    inline bool peek(type_t &peek) const
    {
        if (empty())
            return false;
        peek = data[_readCount & mask];
        return true;
    }

    /**
     * @brief Получить последний доступный элемент
     * @return type_t type_t() если пусто
     * @return type_t Элемент
     */
    [[nodiscard]] inline type_t back() const
    {
        if (empty())
            return type_t();
        return operator[](size() - 1);
    }

    inline type_t &operator[](index_t i)
    {
        return data[(_readCount + i) & mask];
    }

    inline const type_t operator[](index_t i) const
    {
        if (empty())
            return type_t();
        return data[(_readCount + i) & mask];
    }

    // void getLinearBock(type_t *&start, index_t &len)
    // {
    //     len = std::min<index_t>(size(), size());
    //     start = &data[_readCount & mask];
    // }

    /**
     * @return true Буфер пуст
     * @return false Доступны элементы
     */
    [[nodiscard]] inline bool empty() const
    {
        return _writeCount == _readCount;
    }

    /**
     * @return true Буфер полон
     * @return false В буфере есть свободное место
     */
    [[nodiscard]] inline bool full() const
    {
        return ((_writeCount - _readCount) & mask2) == _size;
    }

    /**
     * @return index_t Количество элементов в буфере
     */
    [[nodiscard]] index_t size() const noexcept
    {
        return (_writeCount - _readCount) & mask2;
    }

    /**
     * @return index_t Оставшиеся место для записи
     */
    [[nodiscard]] index_t available() noexcept
    {
        return maxSize() - size();
    }

    /**
     * @brief Полная очистка буфера
     */
    inline void clear()
    {
        _readCount = _writeCount;
    }
};