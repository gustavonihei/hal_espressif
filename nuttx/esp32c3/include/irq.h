/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Matrix
 *
 * The Interrupt Matrix embedded in the ESP32-C3 independently allocates
 * peripheral interrupt sources to the CPUs’ peripheral interrupts.
 * This configuration is highly flexible in order to meet many different
 * needs.
 *
 * Features
 * - Accepts 62 peripheral interrupt sources as input.
 * - Generate 31 peripheral interrupts to CPU as output.
 * - Queries current interrupt status of peripheral interrupt sources.
 */

/* RESERVED interrupts: 0 to 14 */

#define ESP_PERIPH_MAC                 0  /* Reserved, but needed by WiFi driver */
#define ESP_PERIPH_MAC_NMI             1  /* Reserved, but needed by WiFi driver */

#define ESP_PERIPH_BT_BB               5 /* Reserved, but needed by BLE driver */
#define ESP_PERIPH_RWBLE               8 /* Reserved, but needed by BLE driver */

#define ESP_PERIPH_UHCI0               15
#define ESP_PERIPH_GPIO                16
#define ESP_PERIPH_GPIO_NMI            17

/* RESERVED interrupt 18 */

#define ESP_PERIPH_SPI2                19

#define ESP_PERIPH_I2S1                20
#define ESP_PERIPH_UART0               21
#define ESP_PERIPH_UART1               22
#define ESP_PERIPH_LEDC                23
#define ESP_PERIPH_EFUSE               24
#define ESP_PERIPH_TWAI                25
#define ESP_PERIPH_USB                 26
#define ESP_PERIPH_RTC_CORE            27
#define ESP_PERIPH_RMT                 28
#define ESP_PERIPH_I2C_EXT0            29

/* RESERVED interrupts 30-31 */

#define ESP_PERIPH_TG0_T0              32
#define ESP_PERIPH_TG0_WDT             33
#define ESP_PERIPH_TG1_T0              34
#define ESP_PERIPH_TG1_WDT             35

/* RESERVED interrupt 36 */

#define ESP_PERIPH_SYSTIMER_T0         37
#define ESP_PERIPH_SYSTIMER_T1         38
#define ESP_PERIPH_SYSTIMER_T2         39

/* RESERVED interrupts 40-42 */

#define ESP_PERIPH_APB_ADC             43
#define ESP_PERIPH_DMA_CH0             44
#define ESP_PERIPH_DMA_CH1             45
#define ESP_PERIPH_DMA_CH2             46
#define ESP_PERIPH_RSA                 47
#define ESP_PERIPH_AES                 48
#define ESP_PERIPH_SHA                 49

#define ESP_PERIPH_FROM_CPU_INT0       50
#define ESP_PERIPH_FROM_CPU_INT1       51
#define ESP_PERIPH_FROM_CPU_INT2       52
#define ESP_PERIPH_FROM_CPU_INT3       53
#define ESP_PERIPH_ASSIST_DEBUG        54
#define ESP_PERIPH_DMA_APBPERI_PMS     55
#define ESP_PERIPH_CORE0_IRAM0_PMS     56
#define ESP_PERIPH_CORE0_DRAM0_PMS     57
#define ESP_PERIPH_CORE0_PIF_PMS       58
#define ESP_PERIPH_CORE0_PIF_PMS_SZIE  59

/* RESERVED interrupts 60-61 */

/* Total number of peripherals */

#define ESP_NPERIPHERALS               62

/* CPU Interrupts.
 *
 * The ESP32-C3 CPU interrupt controller accepts 31 asynchronous interrupts.
 */

#define ESP_CPUINT_MIN             1
#define ESP_CPUINT_MAX             31

#define ESP_NCPUINTS               32

#define ESP_CPUINT_MAC             0
#define ESP_CPUINT_MAC_NMI         1

#define ESP_CPUINT_BT_BB           5
#define ESP_CPUINT_RWBLE_IRQ       8

#define ESP_CPUINT_PERIPHSET       0xffffffff

/* Reserved CPU interrupt for specific drivers */

#define ESP_CPUINT_WMAC            1  /* Wi-Fi MAC */
#define ESP_CPUINT_BT_BB           5  /* BT BB */
#define ESP_CPUINT_RWBLE           8  /* RW BLE */

/* IRQ numbers. */

/* ecall is dispatched like normal interrupts.  It occupies an IRQ number. */

#define RISCV_NIRQ_INTERRUPTS       16  /* Number of RISC-V dispatched interrupts. */
#define ESP_IRQ_FIRSTPERIPH     16  /* First peripheral IRQ number */

/* IRQ numbers for peripheral interrupts coming through the Interrupt
 * Matrix.
 */

#define ESP_IRQ2PERIPH(irq)       ((irq) - ESP_IRQ_FIRSTPERIPH)
#define ESP_PERIPH2IRQ(id)        ((id) + ESP_IRQ_FIRSTPERIPH)

/* Peripheral IRQs */

#define ESP_IRQ_MAC                 (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_MAC)
#define ESP_IRQ_MAC_NMI             (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_MAC_NMI)

#define ESP_IRQ_BT_BB               (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_BT_BB)
#define ESP_IRQ_RWBLE               (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_RWBLE)

#define ESP_IRQ_UHCI0               (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_UHCI0)
#define ESP_IRQ_GPIO                (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_GPIO)
#define ESP_IRQ_GPIO_NMI            (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_GPIO_NMI)

#define ESP_IRQ_SPI2                (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_SPI2)
#define ESP_IRQ_I2S1                (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_I2S1)
#define ESP_IRQ_UART0               (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_UART0)
#define ESP_IRQ_UART1               (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_UART1)
#define ESP_IRQ_LEDC                (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_LEDC)
#define ESP_IRQ_EFUSE               (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_EFUSE)
#define ESP_IRQ_TWAI                (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_TWAI)
#define ESP_IRQ_USB                 (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_USB)
#define ESP_IRQ_RTC_CORE            (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_RTC_CORE)
#define ESP_IRQ_RMT                 (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_RMT)
#define ESP_IRQ_I2C_EXT0            (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_I2C_EXT0)

#define ESP_IRQ_TG0_T0              (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_TG0_T0)
#define ESP_IRQ_TG0_WDT             (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_TG0_WDT)
#define ESP_IRQ_TG1_T0              (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_TG1_T0)
#define ESP_IRQ_TG1_WDT             (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_TG1_WDT)

#define ESP_IRQ_SYSTIMER_T0         (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_SYSTIMER_T0)
#define ESP_IRQ_SYSTIMER_T1         (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_SYSTIMER_T1)
#define ESP_IRQ_SYSTIMER_T2         (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_SYSTIMER_T2)

#define ESP_IRQ_APB_ADC             (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_APB_ADC)
#define ESP_IRQ_DMA_CH0             (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_DMA_CH0)
#define ESP_IRQ_DMA_CH1             (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_DMA_CH1)
#define ESP_IRQ_DMA_CH2             (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_DMA_CH2)
#define ESP_IRQ_RSA                 (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_RSA)
#define ESP_IRQ_AES                 (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_AES)
#define ESP_IRQ_SHA                 (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_SHA)
#define ESP_IRQ_FROM_CPU_INT0       (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_FROM_CPU_INT0)
#define ESP_IRQ_FROM_CPU_INT1       (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_FROM_CPU_INT1)
#define ESP_IRQ_FROM_CPU_INT2       (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_FROM_CPU_INT2)
#define ESP_IRQ_FROM_CPU_INT3       (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_FROM_CPU_INT3)
#define ESP_IRQ_ASSIST_DEBUG        (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_ASSIST_DEBUG)
#define ESP_IRQ_DMA_APBPERI_PMS     (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_DMA_APBPERI_PMS)
#define ESP_IRQ_CORE0_IRAM0_PMS     (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_CORE0_IRAM0_PMS)
#define ESP_IRQ_CORE0_DRAM0_PMS     (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_CORE0_DRAM0_PMS)
#define ESP_IRQ_CORE0_PIF_PMS       (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_CORE0_PIF_PMS)
#define ESP_IRQ_CORE0_PIF_PMS_SZIE  (ESP_IRQ_FIRSTPERIPH + ESP_PERIPH_CORE0_PIF_PMS_SZIE)

#define ESP_NIRQ_PERIPH             ESP_NPERIPHERALS

/* Second level GPIO interrupts.  GPIO interrupts are decoded and dispatched
 * as a second level of decoding:  The first level dispatches to the GPIO
 * interrupt handler.  The second to the decoded GPIO interrupt handler.
 */

#ifdef CONFIG_ESP_GPIO_IRQ
#  define ESP_NIRQ_GPIO           22
#  define ESP_FIRST_GPIOIRQ       (RISCV_NIRQ_INTERRUPTS + ESP_NIRQ_PERIPH)
#  define ESP_LAST_GPIOIRQ        (ESP_FIRST_GPIOIRQ + ESP_NIRQ_GPIO - 1)
#  define ESP_PIN2IRQ(p)          ((p) + ESP_FIRST_GPIOIRQ)
#  define ESP_IRQ2PIN(i)          ((i) - ESP_FIRST_GPIOIRQ)
#else
#  define ESP_NIRQ_GPIO           0
#endif

/* Total number of IRQs: ecall + Number of peripheral IRQs + GPIOs IRQs. */

#define NR_IRQS  (RISCV_NIRQ_INTERRUPTS + ESP_NIRQ_PERIPH + ESP_NIRQ_GPIO)
