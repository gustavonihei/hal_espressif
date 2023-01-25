/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hal/clk_gate_ll.h"
#include "esp_attr.h"
#include "driver/periph_ctrl.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#elif defined(__NuttX__)
#include <nuttx/irq.h>
#endif

#ifdef __ZEPHYR__
#define ENTER_CRITICAL_SECTION(state)   do { (state) = irq_lock(); } while(0)
#define LEAVE_CRITICAL_SECTION(state)   irq_unlock((state))

static unsigned int state;
#elif defined(__NuttX__)
#define ENTER_CRITICAL_SECTION(state)   do { (state) = enter_critical_section(); } while(0)
#define LEAVE_CRITICAL_SECTION(state)   leave_critical_section((state))

static irqstate_t state;
#endif

static uint8_t ref_counts[PERIPH_MODULE_MAX + 1] = {0};

void periph_module_enable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    ENTER_CRITICAL_SECTION(state);
    if (ref_counts[periph] == 0) {
        periph_ll_enable_clk_clear_rst(periph);
    }
    ref_counts[periph]++;
    LEAVE_CRITICAL_SECTION(state);
}

void periph_module_disable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    ENTER_CRITICAL_SECTION(state);
    ref_counts[periph]--;
    if (ref_counts[periph] == 0) {
        periph_ll_disable_clk_set_rst(periph);
    }
    LEAVE_CRITICAL_SECTION(state);
}

void periph_module_reset(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    ENTER_CRITICAL_SECTION(state);
    periph_ll_reset(periph);
    LEAVE_CRITICAL_SECTION(state);
}

#if CONFIG_WIFI_ESP32 || CONFIG_BT
IRAM_ATTR void wifi_bt_common_module_enable(void)
{
    ENTER_CRITICAL_SECTION(state);
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_enable_clk_clear_rst();
    }
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]++;
    LEAVE_CRITICAL_SECTION(state);
}

IRAM_ATTR void wifi_bt_common_module_disable(void)
{
    ENTER_CRITICAL_SECTION(state);
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]--;
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_disable_clk_set_rst();
    }
    LEAVE_CRITICAL_SECTION(state);
}

void wifi_module_enable(void)
{
    ENTER_CRITICAL_SECTION(state);
    periph_ll_wifi_module_enable_clk_clear_rst();
    LEAVE_CRITICAL_SECTION(state);
}

void wifi_module_disable(void)
{
    ENTER_CRITICAL_SECTION(state);
    periph_ll_wifi_module_disable_clk_set_rst();
    LEAVE_CRITICAL_SECTION(state);
}
#endif // CONFIG_WIFI_ESP32 || CONFIG_BT
