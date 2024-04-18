#include "I2C/I2C.hpp"
#include "Common.hpp"
#include "InvenSense_ICM20948_registers.hpp"
#include "SRT/SRT.hpp"

using namespace InvenSense_ICM20948;

I2C &i2cBus = i2c1;
static constexpr uint8_t address = 0x68;

uint8_t lastUsedBank = 0;
void SelectRegisterBank(enum REG_BANK_SEL_BIT bank, bool force = false)
{
    if (bank != lastUsedBank or force)
    {
        i2cBus.begin();
        i2cBus.startTx(address, 2);
        i2cBus.write(static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL));
        i2cBus.write(bank);
        i2cBus.waitTransferComplete();
        i2cBus.end();

        lastUsedBank = bank;
    }
}

void SelectRegisterBank(Register::BANK_0 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0); }
void SelectRegisterBank(Register::BANK_2 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_2); }
void SelectRegisterBank(Register::BANK_3 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_3); }

template <typename T>
uint8_t RegisterRead(T reg)
{
    uint8_t value;
    SelectRegisterBank(reg);

    i2cBus.begin();
    i2cBus.startTx(address, 1, false);
    i2cBus.write(static_cast<uint8_t>(reg));
    i2cBus.waitTransferComplete();
    i2cBus.startRx(address, 1);
    value = i2cBus.read();
    i2cBus.stop();
    i2cBus.end();

    return value;
}

template <typename T>
void RegisterWrite(T reg, uint8_t value)
{
    SelectRegisterBank(reg);

    i2cBus.begin();
    i2cBus.startTx(address, 2, false);
    i2cBus.write(static_cast<uint8_t>(reg));
    i2cBus.write(value);
    i2cBus.waitTransferComplete();
    i2cBus.end();
}

template <typename T>
void RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
    const uint8_t orig_val = RegisterRead(reg);

    uint8_t val = (orig_val & ~clearbits) | setbits;

    if (orig_val != val)
    {
        RegisterWrite(reg, val);
    }
}

template <typename T>
bool RegisterCheck(const T &reg_cfg)
{
    bool success = true;

    const uint8_t reg_value = RegisterRead(reg_cfg.reg);

    if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits))
    {
        // PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
        success = false;
    }

    if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0))
    {
        // PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
        success = false;
    }

    return success;
}

struct register_bank0_config_t
{
    Register::BANK_0 reg;
    uint8_t set_bits{0};
    uint8_t clear_bits{0};
};

struct register_bank2_config_t
{
    Register::BANK_2 reg;
    uint8_t set_bits{0};
    uint8_t clear_bits{0};
};

struct register_bank3_config_t
{
    Register::BANK_3 reg;
    uint8_t set_bits{0};
    uint8_t clear_bits{0};
};

static constexpr register_bank0_config_t _register_bank0_cfg[] = {
    // Register                             | Set bits, Clear bits
    {Register::BANK_0::USER_CTRL, USER_CTRL_BIT::I2C_MST_EN | USER_CTRL_BIT::DMP_EN, USER_CTRL_BIT::I2C_IF_DIS},
    {Register::BANK_0::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0, PWR_MGMT_1_BIT::DEVICE_RESET | PWR_MGMT_1_BIT::SLEEP | PWR_MGMT_1_BIT::TEMP_DIS},
    {Register::BANK_0::INT_PIN_CFG, 0, INT_PIN_CFG_BIT::BYPASS_EN},
};

static constexpr register_bank2_config_t _register_bank2_cfg[] = {
    // Register                             | Set bits, Clear bits
    {Register::BANK_2::GYRO_CONFIG_1, GYRO_CONFIG_1_BIT::GYRO_FS_SEL_2000_DPS | GYRO_CONFIG_1_BIT::GYRO_FCHOICE, GYRO_CONFIG_1_BIT::GYRO_DLPFCFG},
    {Register::BANK_2::ACCEL_CONFIG, ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G | ACCEL_CONFIG_BIT::ACCEL_FCHOICE, ACCEL_CONFIG_BIT::ACCEL_DLPFCFG},
};

static constexpr register_bank3_config_t _register_bank3_cfg[] = {
    // Register                             | Set bits, Clear bits
    {Register::BANK_3::I2C_MST_CTRL, 0, 0},
    {Register::BANK_3::I2C_MST_DELAY_CTRL, 0, 0},
    {Register::BANK_3::I2C_SLV4_CTRL, 0, 0},
};

enum class STATE : uint8_t
{
    RESET,
    WAIT_FOR_RESET,
    CONFIGURE,
    FIFO_READ,
} state{STATE::RESET};

void init()
{
}

void enable()
{
}

void handler()
{
    static uint32_t delayTime = 0, lastTime = 0;
    const uint32_t now = millis();

    if (millis() - lastTime < delayTime)
        return;
    lastTime = now;

    switch (state)
    {
    case STATE::RESET:
        // PWR_MGMT_1: Device Reset
        RegisterWrite(Register::BANK_0::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
        state = STATE::WAIT_FOR_RESET;
        delayTime = 100;
        break;

    case STATE::WAIT_FOR_RESET:
    {
        uint8_t whoim = RegisterRead(Register::BANK_0::WHO_AM_I);
        // The reset value is 0x00 for all registers other than the registers below
        if ((whoim == WHOAMI) && (RegisterRead(Register::BANK_0::PWR_MGMT_1) == 0x41))
        {

            // Wakeup and reset
            RegisterWrite(Register::BANK_0::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
            RegisterWrite(Register::BANK_0::USER_CTRL,
                          USER_CTRL_BIT::I2C_MST_EN | USER_CTRL_BIT::I2C_IF_DIS | USER_CTRL_BIT::SRAM_RST | USER_CTRL_BIT::I2C_MST_RST);

            // if reset succeeded then configure
            state = STATE::CONFIGURE;
            delayTime = 100;
        }
        else
        {
            __BKPT(0);
        }
    }

    break;

    case STATE::CONFIGURE:
    {
        for (const auto &reg_cfg : _register_bank0_cfg)
            RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

        for (const auto &reg_cfg : _register_bank2_cfg)
            RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

        for (const auto &reg_cfg : _register_bank3_cfg)
            RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

        // now check that all are configured
        bool success = true;

        for (const auto &reg_cfg : _register_bank0_cfg)
            if (!RegisterCheck(reg_cfg))
                success = false;

        for (const auto &reg_cfg : _register_bank2_cfg)
            if (!RegisterCheck(reg_cfg))
                success = false;

        for (const auto &reg_cfg : _register_bank3_cfg)
            if (!RegisterCheck(reg_cfg))
                success = false;

        if (success)
        {

            // start AK09916 magnetometer (I2C aux)
            // if (_slave_ak09916_magnetometer) {
            // 	_slave_ak09916_magnetometer->Reset();
            // }

            // if configure succeeded then start reading from FIFO
            state = STATE::FIFO_READ;
        }
        else
            __BKPT(0);
        break;
    }

    default:
        break;
    }
}
REGISTER_SRT_MODULE(icm20948, init, enable, handler);