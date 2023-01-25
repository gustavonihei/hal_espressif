/*
 * SPDX-FileCopyrightText: 2020-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#elif defined(__NuttX__)
#include <nuttx/irq.h>
#endif

#include "regi2c_ctrl.h"
#include "esp_attr.h"
#include <stdint.h>

#ifdef __ZEPHYR__
#define ENTER_CRITICAL_SECTION(state)   do { (state) = irq_lock(); } while(0)
#define LEAVE_CRITICAL_SECTION(state)   irq_unlock((state))

static unsigned int state;
#elif defined(__NuttX__)
#define ENTER_CRITICAL_SECTION(state)   do { (state) = enter_critical_section(); } while(0)
#define LEAVE_CRITICAL_SECTION(state)   leave_critical_section((state))

static irqstate_t state;
#endif

uint8_t IRAM_ATTR regi2c_ctrl_read_reg(uint8_t block, uint8_t host_id, uint8_t reg_add)
{
    ENTER_CRITICAL_SECTION(state);
    uint8_t value = i2c_read_reg_raw(block, host_id, reg_add);
    LEAVE_CRITICAL_SECTION(state);
    return value;
}

uint8_t IRAM_ATTR regi2c_ctrl_read_reg_mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb)
{
    ENTER_CRITICAL_SECTION(state);
    uint8_t value = i2c_read_reg_mask_raw(block, host_id, reg_add, msb, lsb);
    LEAVE_CRITICAL_SECTION(state);
    return value;
}

void IRAM_ATTR regi2c_ctrl_write_reg(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t data)
{
    ENTER_CRITICAL_SECTION(state);
    i2c_write_reg_raw(block, host_id, reg_add, data);
    LEAVE_CRITICAL_SECTION(state);
}

void IRAM_ATTR regi2c_ctrl_write_reg_mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb, uint8_t data)
{
    ENTER_CRITICAL_SECTION(state);
    i2c_write_reg_mask_raw(block, host_id, reg_add, msb, lsb, data);
    LEAVE_CRITICAL_SECTION(state);
}

void IRAM_ATTR regi2c_enter_critical(void)
{
    ENTER_CRITICAL_SECTION(state);
}

void IRAM_ATTR regi2c_exit_critical(void)
{
    LEAVE_CRITICAL_SECTION(state);
}

/**
 * Restore regi2c analog calibration related configuration registers.
 * This is a workaround, and is fixed on later chips
 */
#if REGI2C_ANA_CALI_PD_WORKAROUND

static DRAM_ATTR uint8_t reg_val[REGI2C_ANA_CALI_BYTE_NUM];

void IRAM_ATTR regi2c_analog_cali_reg_read(void)
{
    for (int i = 0; i < REGI2C_ANA_CALI_BYTE_NUM; i++) {
        reg_val[i] = regi2c_ctrl_read_reg(I2C_SAR_ADC, I2C_SAR_ADC_HOSTID, i);
    }
}

void IRAM_ATTR regi2c_analog_cali_reg_write(void)
{
    for (int i = 0; i < REGI2C_ANA_CALI_BYTE_NUM; i++) {
        regi2c_ctrl_write_reg(I2C_SAR_ADC, I2C_SAR_ADC_HOSTID, i, reg_val[i]);
    }
}

#endif   //#if ADC_CALI_PD_WORKAROUND
