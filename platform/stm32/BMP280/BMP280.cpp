#include <cstdint>
#include <cmath>
#include "BMP280.hpp"
#include "I2C/I2C.hpp"
#include "Board.hpp"
#include "Common.hpp"
#include "ahrs/ahrs.hpp"

namespace BMP280
{

#define BMP280_ADDR_CAL 0x88  /* address of 12x 2 bytes calibration data */
#define BMP280_ADDR_DATA 0xF7 /* address of 2x 3 bytes p-t data */

#define BMP280_ADDR_CONFIG 0xF5 /* configuration */
#define BMP280_ADDR_CTRL 0xF4   /* controll */
#define BMP280_ADDR_STATUS 0xF3 /* state */
#define BMP280_ADDR_RESET 0xE0  /* reset */
#define BMP280_ADDR_ID 0xD0     /* id */

#define BMP280_VALUE_ID 0x58    /* chip id */
#define BMP280_VALUE_RESET 0xB6 /* reset */

#define BMP280_STATUS_MEASURING (1 << 3) /* if in process of measure */
#define BMP280_STATUS_COPING (1 << 0)    /* if in process of data copy */

#define BMP280_CTRL_P0 (0x0 << 2) /* no p measure */
#define BMP280_CTRL_P1 (0x1 << 2)
#define BMP280_CTRL_P2 (0x2 << 2)
#define BMP280_CTRL_P4 (0x3 << 2)
#define BMP280_CTRL_P8 (0x4 << 2)
#define BMP280_CTRL_P16 (0x5 << 2)

#define BMP280_CTRL_T0 (0x0 << 5) /* no t measure */
#define BMP280_CTRL_T1 (0x1 << 5)
#define BMP280_CTRL_T2 (0x2 << 5)
#define BMP280_CTRL_T4 (0x3 << 5)
#define BMP280_CTRL_T8 (0x4 << 5)
#define BMP280_CTRL_T16 (0x5 << 5)

#define BMP280_CONFIG_F0 (0x0 << 2) /* no filter */
#define BMP280_CONFIG_F2 (0x1 << 2)
#define BMP280_CONFIG_F4 (0x2 << 2)
#define BMP280_CONFIG_F8 (0x3 << 2)
#define BMP280_CONFIG_F16 (0x4 << 2)

#define BMP280_CTRL_MODE_SLEEP 0x0
#define BMP280_CTRL_MODE_FORCE 0x1 /* on demand, goes to sleep after */
#define BMP280_CTRL_MODE_NORMAL 0x3

#define BMP280_MT_INIT 6400 /* max measure time of initial p + t in us */
#define BMP280_MT 2300      /* max measure time of p or t in us */

#pragma pack(push, 1)
    struct calibration_s
    {
        uint16_t t1;
        int16_t t2;
        int16_t t3;

        uint16_t p1;
        int16_t p2;
        int16_t p3;
        int16_t p4;
        int16_t p5;
        int16_t p6;
        int16_t p7;
        int16_t p8;
        int16_t p9;
    } calibration; // calibration data

    struct data_s
    {
        uint8_t p_msb;
        uint8_t p_lsb;
        uint8_t p_xlsb;

        uint8_t t_msb;
        uint8_t t_lsb;
        uint8_t t_xlsb;
    } data; // data
#pragma pack(pop)

    struct fcalibration_s
    {
        float t1;
        float t2;
        float t3;

        float p1;
        float p2;
        float p3;
        float p4;
        float p5;
        float p6;
        float p7;
        float p8;
        float p9;
    } fcal;

    constexpr uint8_t address = 0x76;

    uint8_t readRegister(uint8_t reg)
    {
        uint8_t value;

        imuI2C.begin();
        imuI2C.startTx(address, 1, false);
        imuI2C.write(static_cast<uint8_t>(reg));
        imuI2C.waitTransferComplete();
        imuI2C.startRx(address, 1);
        value = imuI2C.read();
        imuI2C.stop();
        imuI2C.end();

        return value;
    }

    void writeRegister(uint8_t value, uint8_t reg)
    {
        imuI2C.begin();
        imuI2C.startTx(address, 2);
        imuI2C.write(static_cast<uint8_t>(reg));
        imuI2C.write(value);
        imuI2C.waitStop();
        imuI2C.end();
    }

    void getData(data_s &data)
    {
        imuI2C.begin();
        imuI2C.startTx(address, 1, false);
        imuI2C.write(BMP280_ADDR_DATA);
        imuI2C.waitTransferComplete();
        imuI2C.startRx(address, sizeof(data_s));
        uint8_t *const ptr = reinterpret_cast<uint8_t *>(&data);
        for (unsigned i = 0; i < sizeof(data_s); i++)
            ptr[i] = imuI2C.read();
        imuI2C.stop();
        imuI2C.end();
    }

    void getCalibration(calibration_s &calibration)
    {
        imuI2C.begin();
        imuI2C.startTx(address, 1, false);
        imuI2C.write(BMP280_ADDR_CAL);
        imuI2C.waitTransferComplete();
        imuI2C.startRx(address, sizeof(calibration_s));
        uint8_t *const ptr = reinterpret_cast<uint8_t *>(&calibration);
        for (unsigned i = 0; i < sizeof(calibration_s); i++)
            ptr[i] = imuI2C.read();
        imuI2C.stop();
        imuI2C.end();
    }

    enum class State : uint8_t
    {
        RESET,
        WAIT_FOR_RESET,
        READ_CALIBRATION,
        IDLE,
        COLLECT,
        MEASURE,
    } state = State::RESET;

    static constexpr uint8_t measureConfig = BMP280_CTRL_P2 | BMP280_CTRL_T2;
    static constexpr uint32_t measureInterval = (BMP280_MT_INIT + BMP280_MT * (16 - 1 + 2 - 1)) / 1'000 + 3;

    void handler()
    {
        static uint32_t delayTime = 0, lastTime = 0;
        const uint32_t now = millis();

        if (millis() - lastTime < delayTime)
            return;
        lastTime = now;

        switch (state)
        {
        case State::RESET:
        {
            writeRegister(BMP280_VALUE_RESET, BMP280_ADDR_RESET);

            delayTime = 10;
            state = State::WAIT_FOR_RESET;
            break;
        }
        case State::WAIT_FOR_RESET:
        {
            if (readRegister(BMP280_ADDR_ID) != BMP280_VALUE_ID)
            {
                // __BKPT(0);
                state = State::RESET;
                delayTime = 10;
                break;
            }
            delayTime = 0;
            state = State::READ_CALIBRATION;

            [[fallthrough]];
        }
        case State::READ_CALIBRATION:
        {
            writeRegister(measureConfig, BMP280_ADDR_CTRL);
            writeRegister(BMP280_CONFIG_F16, BMP280_ADDR_CONFIG);

            getCalibration(calibration);
            fcal.t1 = calibration.t1 * powf(2, 4);
            fcal.t2 = calibration.t2 * powf(2, -14);
            fcal.t3 = calibration.t3 * powf(2, -34);

            fcal.p1 = calibration.p1 * (powf(2, 4) / -100000.0f);
            fcal.p2 = calibration.p1 * calibration.p2 * (powf(2, -31) / -100000.0f);
            fcal.p3 = calibration.p1 * calibration.p3 * (powf(2, -51) / -100000.0f);

            fcal.p4 = calibration.p4 * powf(2, 4) - powf(2, 20);
            fcal.p5 = calibration.p5 * powf(2, -14);
            fcal.p6 = calibration.p6 * powf(2, -31);

            fcal.p7 = calibration.p7 * powf(2, -4);
            fcal.p8 = calibration.p8 * powf(2, -19) + 1.0f;
            fcal.p9 = calibration.p9 * powf(2, -35);

            delayTime = 0;
            state = State::MEASURE;
            break;
        }
        case State::COLLECT:
        {
            getData(data);

            // convert data to number 20 bit
            uint32_t p_raw = data.p_msb << 12 | data.p_lsb << 4 | data.p_xlsb >> 4;
            uint32_t t_raw = data.t_msb << 12 | data.t_lsb << 4 | data.t_xlsb >> 4;

            // Temperature
            float ofs = (float)t_raw - fcal.t1;
            float t_fine = (ofs * fcal.t3 + fcal.t2) * ofs;
            const float T = t_fine * (1.0f / 5120.0f);

            // Pressure
            float tf = t_fine - 128000.0f;
            float x1 = (tf * fcal.p6 + fcal.p5) * tf + fcal.p4;
            float x2 = (tf * fcal.p3 + fcal.p2) * tf + fcal.p1;

            float pf = ((float)p_raw + x1) / x2;
            const float P = (pf * fcal.p9 + fcal.p8) * pf + fcal.p7;

            AHRS::updateByPressure(P);
            AHRS::updateByTemperature(T);

            state = State::MEASURE;
            delayTime = 0;

            [[fallthrough]];
        }
        case State::MEASURE:
        {
            writeRegister(measureConfig | BMP280_CTRL_MODE_FORCE, BMP280_ADDR_CTRL);
            delayTime = measureInterval;
            state = State::COLLECT;
            break;
        }
        default:
            break;
        }
    }

    void isr()
    {
        handler();
    }
}