/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

#include <stdint.h>
#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stm32_gpio.h>

/* LEDs */
#define GPIO_LED_REDn       /* PE3 */   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN3)
#define GPIO_LED_GREENn     /* PE4 */   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN4)
#define GPIO_LED_BLUEn      /* PE5 */   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN5)

#define BOARD_HAS_CONTROL_STATUS_LEDS       1
#define BOARD_OVERLOAD_LED                  LED_RED
#define BOARD_ARMED_STATE_LED               LED_BLUE

/* ADC channels */
#define GPIO_ADC1_INP16     /* PA0  */  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC1_INP18     /* PA4  */  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)
#define GPIO_ADC3_INP4      /* PF5  */  (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN5)
#define GPIO_ADC1_INP6      /* PF12 */  (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN12)
#define GPIO_ADC3_INP15     /* PH4  */  (GPIO_ANALOG | GPIO_PORTH | GPIO_PIN4)
#define GPIO_ADC1_INP2      /* PF11 */  (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN11)
#define GPIO_ADC3_INP14     /* PH3  */  (GPIO_ANALOG | GPIO_PORTH | GPIO_PIN3)
#define GPIO_ADC1_INP10     /* PC0  */  (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN0)
#define GPIO_ADC3_INP8      /* PF6  */  (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN6)
#define GPIO_ADC1_INP13     /* PC3  */  (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN3)

#define PX4_ADC_GPIO_LIST           \
    /* PA0  */  GPIO_ADC1_INP16,    \
    /* PA4  */  GPIO_ADC1_INP18,    \
    /* PF5  */  GPIO_ADC3_INP4,     \
    /* PF12 */  GPIO_ADC1_INP6,     \
    /* PH4  */  GPIO_ADC3_INP15,    \
    /* PF11 */  GPIO_ADC1_INP2,     \
    /* PH3  */  GPIO_ADC3_INP14,    \
    /* PC0  */  GPIO_ADC1_INP10,    \
    /* PF6  */  GPIO_ADC3_INP8,     \
    /* PC3  */  GPIO_ADC1_INP13

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define BATT_VOLTAGE_SENS       /* PA0  */  16
#define BATT_CURRENT_SENS       /* PA4  */  18
#define BATT2_VOLTAGE_SENS      /* PF5  */  4
#define BATT2_CURRENT_SENS      /* PF12 */  6
#define VDD_5V_SENS             /* PH4  */  15
#define SCALED_V3V3             /* PF11 */  2
#define FMU_SERVORAIL_VCC_SENS  /* PH3  */  14
#define RSSI_IN                 /* PC0  */  10
#define ADC3_6V6                /* PF6  */  8
#define ADC1_3V3                /* PC3  */  13

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY1_VOLTAGE_CHANNEL    BATT_VOLTAGE_SENS   /* PA0 : BATT_VOLTAGE_SENS */
#define ADC_BATTERY1_CURRENT_CHANNEL    BATT_CURRENT_SENS   /* PA4 : BATT_CURRENT_SENS */
#define ADC_BATTERY2_VOLTAGE_CHANNEL    BATT2_VOLTAGE_SENS  /* PF5 : BATT2_VOLTAGE_SENS */
#define ADC_BATTERY2_CURRENT_CHANNEL    BATT2_CURRENT_SENS  /* PF12: BATT2_CURRENT_SENS */
#define ADC_VDD_5V_CHANNEL              VDD_5V_SENS         /* PH4 : VDD_5V_SENS */

#define ADC_CHANNELS                    \
    ((1 << BATT_VOLTAGE_SENS)      |    \
     (1 << BATT_CURRENT_SENS)      |    \
     (1 << BATT2_VOLTAGE_SENS)     |    \
     (1 << BATT2_CURRENT_SENS)     |    \
     (1 << VDD_5V_SENS)            |    \
     (1 << SCALED_V3V3)            |    \
     (1 << FMU_SERVORAIL_VCC_SENS) |    \
     (1 << RSSI_IN)                |    \
     (1 << ADC3_6V6)               |    \
     (1 << ADC1_3V3))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V    (5.6f)

/* UAVCAN */
#define UAVCAN_NUM_IFACES_RUNTIME   1

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  16
#define BOARD_NUM_IO_TIMERS 5

/* Power supply control and monitoring GPIOs */
#define GPIO_VDD_3V3_SENSORS_EN     /* PJ2  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN2)
#define GPIO_VDD_3V3_GPS_EN         /* PG0  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTG | GPIO_PIN0)
#define GPIO_BUZZER_VCC_EN          /* PJ4  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN4)
#define GPIO_VDD_5V_PERIPH_CEn      /* PK1  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTK | GPIO_PIN1)
#define GPIO_VDD_5V_HIPOWER_CEn     /* PG10 */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN10)
#define GPIO_VDD_3V3_SPEKTRUM_EN    /* PA15 */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN15)

#define BOARD_NUMBER_BRICKS     2   /* BRICKS信号は3つあるが、ADC変換で測定できるのは、BRICKとBRICK2だけなのため「2」とする。
                                     * この定義で、電圧と電源のADC変換CHを変換テーブルを定義するため、ADC変換がない場合CHを割り当てられないだめである。 */
#define GPIO_VDD_BRICK1_VALIDn      /* PG1 */   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN1)
#define GPIO_VDD_BRICK2_VALIDn      /* PG2 */   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN2)
#define GPIO_VDD_BRICK3_VALIDn      /* PG3 */   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN3)
#define GPIO_VDD_5V_PERIPH_FAULTn   /* PJ15 */  (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTJ | GPIO_PIN15)
#define GPIO_VDD_5V_HIPOWER_FAULTn  /* PF10 */  (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTF | GPIO_PIN10)
#define GPIO_VBUS_RESERVED          /* PA9 */   (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN9)   /* for USB_CONNECTED */
#define GPIO_5V_HEATER_FAULTn       /* PD11 */  (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTD | GPIO_PIN11)

/* Others GPIO */
#define GPIO_BUZZER_1               /* PF9  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTF | GPIO_PIN9)    /* ALARM */
#define GPIO_WDOG                   /* PI0  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTI | GPIO_PIN0)
#define GPIO_CAN_OEn                /* PK4  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTK | GPIO_PIN4)
#define GPIO_ETH_RMII_POWER_EN      /* PH2  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTH | GPIO_PIN2)
#define GPIO_SAFETY_LEDn            /* PD10 */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN10)
#define GPIO_PWM_OEn                /* PK2  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTK | GPIO_PIN2)
#define GPIO_PWM_VOLTAGE_SEL_5V_3Vn /* PJ6  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN6)
#define GPIO_TERMCAN1_EN            /* PG7  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN7)
#define GPIO_TERMCAN2_EN            /* PG8  */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN8)
#define GPIO_HEATER_OUTPUT          /* PI12 */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTI | GPIO_PIN12)
#define GPIO_nARMED                 /* PB10 */  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN10)

#define GPIO_FMU_CAP1               /* PE11 */  (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTE | GPIO_PIN11)
#define GPIO_FMU_CAP2               /* PB11 */  (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTB | GPIO_PIN11)
#define GPIO_SAFETY_SWITCH_IN       /* PG9  */  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTG | GPIO_PIN9)
#define GPIO_SD_CARD_IN             /* PC13 */  (GPIO_INPUT | GPIO_PORTC | GPIO_PIN13)

/* Define True logic Power Control in arch agnostic form */
#define VDD_5V_PERIPH_EN(on_true)           px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_CEn, !(on_true))      /* PK1  */
#define VDD_5V_HIPOWER_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_CEn, !(on_true))     /* PG10 */
#define VDD_3V3_SENSORS_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))      /* PJ2  */
#define VDD_3V3_SENSORS2_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_GPS_EN, (on_true))          /* PG0  */
#define VDD_3V3_SENSORS3_EN(on_true)        px4_arch_gpiowrite(GPIO_BUZZER_VCC_EN, (on_true))           /* PJ4  */
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true)  px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_EN, (on_true))     /* PA15 */

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED     (px4_arch_gpioread(GPIO_VBUS_RESERVED))             /* PA9  */
#define BOARD_ADC_USB_VALID         (!px4_arch_gpioread(GPIO_VDD_BRICK3_VALIDn))        /* PG3  */
#define BOARD_ADC_BRICK1_VALID      (!px4_arch_gpioread(GPIO_VDD_BRICK1_VALIDn))        /* PG1  */
#define BOARD_ADC_BRICK2_VALID      (!px4_arch_gpioread(GPIO_VDD_BRICK2_VALIDn))        /* PG2  */
#define BOARD_ADC_BRICK3_VALID      BOARD_ADC_USB_VALID                                 /* PG3  */
#define BOARD_ADC_PERIPH_5V_OC      (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_FAULTn))     /* PJ15 */
#define BOARD_ADC_HIPOWER_5V_OC     (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_FAULTn))    /* PF10 */

/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY             GPIO_SAFETY_SWITCH_IN   /* Enable the FMU to control it if there is no px4io */

/* BUFFER */
#define BUFFER_OE_EN(on_true)       px4_arch_gpiowrite(GPIO_PWM_OEn, (on_true))

/* ARMED */
#define ARMED_EN(on_true)           px4_arch_gpiowrite(GPIO_nARMED, (on_true))

/* HEATER */
#define HEATER_OUTPUT_EN(on_true)   px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* Tone alarm output */
#define TONE_ALARM_TIMER        14  /* PF9 TIM14_CH1 */
#define TONE_ALARM_CHANNEL      1   /* PF9 TIM14_CH1 */
#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1       /* PF9 */
#define GPIO_TONE_ALARM         GPIO_TIM14_CH1OUT_2 /* PF9 */

/* PWM input driver. timer12 channel 1 */
#define PWMIN_TIMER             12  /* PH6 TIM12_CH1 */
#define PWMIN_TIMER_CHANNEL     1   /* PH6 TIM12_CH1 */
#define GPIO_PWM_IN             GPIO_TIM12_CH1IN_2  /* PH6 */

/* USB
 *  OTG FS: PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         GPIO_VBUS_RESERVED  /* PA9 */

/* High-resolution timer */
#define HRT_TIMER               5   /* use timer5 for the HRT */
#define HRT_TIMER_CHANNEL       1   /* use capture/compare channel 1 */

/* SD card */
#define SDIO_SLOTNO             0   /* Only one slot */
#define SDIO_MINOR              0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */
#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE   5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET          1

//Add a new setting instead of removing BOARD_HAS_HW_VERSIONING
#define BOARD_HAS_STATIC_MANIFEST   2

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST {            \
		/* ADC */                       \
		PX4_ADC_GPIO_LIST,              \
		/* FDCAN */                     \
		GPIO_FDCAN1_TX,                 \
		GPIO_FDCAN1_RX,                 \
		GPIO_FDCAN2_TX,                 \
		GPIO_FDCAN2_RX,                 \
		/* Power */                     \
		GPIO_VDD_BRICK1_VALIDn,         \
		GPIO_VDD_BRICK2_VALIDn,         \
		GPIO_VDD_BRICK3_VALIDn,         \
		GPIO_VDD_5V_PERIPH_FAULTn,      \
		GPIO_VDD_5V_HIPOWER_FAULTn,     \
		GPIO_VBUS_RESERVED,             \
		GPIO_5V_HEATER_FAULTn,          \
		GPIO_VDD_3V3_SENSORS_EN,        \
		GPIO_VDD_3V3_GPS_EN,            \
		GPIO_BUZZER_VCC_EN,             \
		GPIO_VDD_5V_PERIPH_CEn,         \
		GPIO_VDD_5V_HIPOWER_CEn,        \
		GPIO_VDD_3V3_SPEKTRUM_EN,       \
		/* I2C */                       \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SDA),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SDA),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SDA),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SDA),  \
		/* GPIO */                      \
		GPIO_FMU_CAP1,                  \
		GPIO_FMU_CAP2,                  \
		GPIO_SAFETY_SWITCH_IN,          \
		GPIO_SD_CARD_IN,                \
		GPIO_BUZZER_1,                  \
		GPIO_WDOG,                      \
		GPIO_CAN_OEn,                   \
		GPIO_ETH_RMII_POWER_EN,         \
		GPIO_SAFETY_LEDn,               \
		GPIO_PWM_OEn,                   \
		GPIO_PWM_VOLTAGE_SEL_5V_3Vn,    \
		GPIO_TERMCAN1_EN,               \
		GPIO_TERMCAN2_EN,               \
		GPIO_HEATER_OUTPUT,             \
		GPIO_nARMED,                    \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

int stm32_sdio_initialize(void);
extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
