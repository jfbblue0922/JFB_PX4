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
// #include <platforms/common/include/px4_platform_common/px4_config.h>
// #include <platforms/nuttx/NuttX/nuttx/include/nuttx/compiler.h>
// #include <platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/hardware/stm32_gpio.h>
#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stm32_gpio.h>

/* LEDs */
#define GPIO_LED_REDn       (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN3)    /* PE3 */
#define GPIO_LED_GREENn     (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN4)    /* PE4 */
#define GPIO_LED_BLUEn      (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN5)    /* PE5 */

#define BOARD_HAS_CONTROL_STATUS_LEDS       1
#define BOARD_OVERLOAD_LED                  LED_RED
#define BOARD_ARMED_STATE_LED               LED_BLUE

/* ADC channels */
#define GPIO_ADC1_INP16     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)  /* PA0  */
#define GPIO_ADC1_INP18     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)  /* PA4  */
#define GPIO_ADC3_INP4      (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN5)  /* PF5  */
#define GPIO_ADC1_INP6      (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN12) /* PF12 */
#define GPIO_ADC3_INP15     (GPIO_ANALOG | GPIO_PORTH | GPIO_PIN4)  /* PH4  */
#define GPIO_ADC1_INP2      (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN11) /* PF11 */
#define GPIO_ADC3_INP14     (GPIO_ANALOG | GPIO_PORTH | GPIO_PIN3)  /* PH3  */
#define GPIO_ADC1_INP10     (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN0)  /* PC0  */
#define GPIO_ADC3_INP8      (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN6)  /* PF6  */
#define GPIO_ADC1_INP13     (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN3)  /* PC3  */

#define PX4_ADC_GPIO_LIST   GPIO_ADC1_INP16,    /* PA0  */  \
                            GPIO_ADC1_INP18,    /* PA4  */  \
                            GPIO_ADC3_INP4,     /* PF5  */  \
                            GPIO_ADC1_INP6,     /* PF12 */  \
                            GPIO_ADC3_INP15,    /* PH4  */  \
                            GPIO_ADC1_INP2,     /* PF11 */  \
                            GPIO_ADC3_INP14,    /* PH3  */  \
                            GPIO_ADC1_INP10,    /* PC0  */  \
                            GPIO_ADC3_INP8,     /* PF6  */  \
                            GPIO_ADC1_INP13     /* PC3  */

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define BATT_VOLTAGE_SENS       16  /* PA0  */
#define BATT_CURRENT_SENS       18  /* PA4  */
#define BATT2_VOLTAGE_SENS      4   /* PF5  */
#define BATT2_CURRENT_SENS      6   /* PF12 */
#define VDD_5V_SENS             15  /* PH4  */
#define SCALED_V3V3             2   /* PF11 */
#define FMU_SERVORAIL_VCC_SENS  14  /* PH3  */
#define RSSI_IN                 10  /* PC0  */
#define ADC3_6V6                8   /* PF6  */
#define ADC1_3V3                13  /* PC3  */

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY1_VOLTAGE_CHANNEL        BATT_VOLTAGE_SENS  /* PA0:  BATT_VOLTAGE_SENS */
#define ADC_BATTERY1_CURRENT_CHANNEL        BATT_CURRENT_SENS  /* PA4:  BATT_CURRENT_SENS */
#define ADC_BATTERY2_VOLTAGE_CHANNEL        BATT2_VOLTAGE_SENS /* PF5:  BATT2_VOLTAGE_SENS */
#define ADC_BATTERY2_CURRENT_CHANNEL        BATT2_CURRENT_SENS /* PF12: BATT2_CURRENT_SENS */
#define ADC_BATTERY3_VOLTAGE_CHANNEL        -1
#define ADC_BATTERY3_CURRENT_CHANNEL        -1
#define ADC_VDD_5V_CHANNEL                  VDD_5V_SENS        /* PH4:  VDD_5V_SENS */

#define ADC_CHANNELS        ((1 << BATT_VOLTAGE_SENS) | \
                             (1 << BATT_CURRENT_SENS) | \
                             (1 << BATT2_VOLTAGE_SENS) | \
                             (1 << BATT2_CURRENT_SENS) | \
                             (1 << VDD_5V_SENS) | \
                             (1 << SCALED_V3V3) | \
                             (1 << FMU_SERVORAIL_VCC_SENS) | \
                             (1 << RSSI_IN) | \
                             (1 << ADC3_6V6) | \
                             (1 << ADC1_3V3))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V    (5.6f)

/* UAVCAN */
#define UAVCAN_NUM_IFACES_RUNTIME   1

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  16
#define BOARD_NUM_IO_TIMERS 4

/* Power supply control and monitoring GPIOs */
#define GPIO_VDD_3V3_SENSORS_EN     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN2)    /* PJ2  VDD_3V3_SENSORS_EN */
#define GPIO_VDD_3V3_GPS_EN         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTG | GPIO_PIN0)    /* PG0  VDD_3V3_SENSORS2_EN */
#define GPIO_VDD_BUZZER_VCC_EN      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN4)    /* PJ4  VDD_3V3_SENSORS3_EN */
#define GPIO_VDD_5V_PERIPH_CEn      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTK | GPIO_PIN1)      /* PK1  VDD_5V_PERIPH_EN */
#define GPIO_VDD_5V_HIPOWER_CEn     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN10)     /* PG10 */
#define GPIO_VDD_3V3_SPEKTRUM_EN    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN15)   /* PA15 */

#define BOARD_NUMBER_BRICKS     3
// #define BOARD_NUMBER_BRICKS     2 /* 【ToDo】3がダメだったら、2にする */
#define GPIO_VDD_BRICK1_VALIDn      (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN1)     /* PG1 VDD_BRICK_VALIDn */
#define GPIO_VDD_BRICK2_VALIDn      (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN2)     /* PG2 VDD_BRICK2_VALIDn */
#define GPIO_VDD_BRICK3_VALIDn      (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN3)     /* PG3 VBUS_VALIDn */
#define GPIO_VDD_5V_PERIPH_FAULTn   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTJ | GPIO_PIN15)    /* PJ15 VDD_5V_PERIPH_OC */
#define GPIO_VDD_5V_HIPOWER_FAULTn  (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTF | GPIO_PIN10)    /* PF10 VDD_5V_HIPOWER_OC */
#define GPIO_VBUS_RESERVED          (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN9)   /* PA9 */
#define GPIO_5V_HEATER_FAULTn       (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTD | GPIO_PIN11)    /* PD11 */

/* Others GPIO */
#define GPIO_WDOG                   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTI | GPIO_PIN0)    /* PI0  */
#define GPIO_FMU_CAP1               (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTE | GPIO_PIN11)                                            /* PE11 */
#define GPIO_FMU_CAP2               (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTB | GPIO_PIN11)                                            /* PB11 */
#define GPIO_CAN_OEn                (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTK | GPIO_PIN4)    /* PK4  */
#define GPIO_ETH_RMII_POWER_EN      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTH | GPIO_PIN2)    /* PH2  */
#define GPIO_SAFETY_LEDn            (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN10)     /* PD10 */
#define GPIO_PWM_OEn                (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTK | GPIO_PIN2)      /* PK2  */
#define GPIO_PWM_VOLTAGE_SEL_5V_3Vn (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN6)    /* PJ6  */
#define GPIO_SAFETY_SWITCH_IN       (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTG | GPIO_PIN9)                                           /* PG9  */
#define GPIO_SD_CARD_IN             (GPIO_INPUT | GPIO_PORTC | GPIO_PIN13)                                                          /* PC13 */
#define GPIO_TERMCAN1_EN            (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN7)      /* PG7  */
#define GPIO_TERMCAN2_EN            (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN8)      /* PG8  */
#define GPIO_HEATER_OUTPUT          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTI | GPIO_PIN12)   /* PI12 */
#define GPIO_nARMED                 (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_CLEAR | GPIO_PORTB|GPIO_PIN10)                       /* PB10 */
#define GPIO_BUZZER_1               (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz |GPIO_OUTPUT_CLEAR | GPIO_PORTF | GPIO_PIN9)     /* PF9 ALARM */

/* Define True logic Power Control in arch agnostic form */
#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_CEn, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_CEn, !(on_true))
#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
#define VDD_3V3_SENSORS2_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_GPS_EN, (on_true))
#define VDD_3V3_SENSORS3_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_BUZZER_VCC_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_EN, (on_true))

/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY         GPIO_SAFETY_SWITCH_IN   /* Enable the FMU to control it if there is no px4io */

/* BUFFER */
#define BUFFER_OE_EN(on_true)              px4_arch_gpiowrite(GPIO_PWM_OEn, (on_true))

/* ARMED */
#define ARMED_EN(on_true)                  px4_arch_gpiowrite(GPIO_nARMED, (on_true))

/* HEATER */
#define HEATER_OUTPUT_EN(on_true)   px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* Tone alarm output */
#define TONE_ALARM_TIMER        14  /* timer 14 */
#define TONE_ALARM_CHANNEL      1   /* PF9 TIM14_CH1 */
#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM14_CH1OUT_2

/* PWM input driver. timer12 channel 1 */
#define PWMIN_TIMER             12   /* PH6 TIM12_CH1 */
#define PWMIN_TIMER_CHANNEL     1    /* PH6 TIM12_CH1 */
#define GPIO_PWM_IN             GPIO_TIM12_CH1IN_2   /* PH6 */

/* USB
 *  OTG FS: PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         GPIO_VBUS_RESERVED  /* PA9 */

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

#define SDIO_SLOTNO             0  /* Only one slot */
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

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_VBUS_RESERVED))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_VDD_BRICK3_VALIDn))
#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_VDD_BRICK1_VALIDn))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_VDD_BRICK2_VALIDn))
#define BOARD_ADC_BRICK3_VALID  (BOARD_ADC_USB_VALID)
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_FAULTn))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_FAULTn))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

//Add a new setting instead of removing BOARD_HAS_HW_VERSIONING
#define BOARD_HAS_STATIC_MANIFEST 2

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		/* ADC */                           \
		PX4_ADC_GPIO_LIST,                  \
		/* FDCAN */                         \
		GPIO_FDCAN1_TX,                     \
		GPIO_FDCAN1_RX,                     \
		GPIO_FDCAN2_TX,                     \
		GPIO_FDCAN2_RX,                     \
		/* Power */                         \
		GPIO_VDD_BRICK1_VALIDn,             \
		GPIO_VDD_BRICK2_VALIDn,             \
		GPIO_VDD_BRICK3_VALIDn,             \
		GPIO_VDD_3V3_SENSORS_EN,            \
		GPIO_VDD_3V3_GPS_EN,                \
		GPIO_VDD_BUZZER_VCC_EN,             \
		GPIO_VDD_3V3_SPEKTRUM_EN,           \
		GPIO_VDD_5V_PERIPH_CEn,             \
		GPIO_VDD_5V_PERIPH_FAULTn,          \
		GPIO_VDD_5V_HIPOWER_CEn,            \
		GPIO_VDD_5V_HIPOWER_FAULTn,         \
		GPIO_VBUS_RESERVED,                 \
		GPIO_5V_HEATER_FAULTn,              \
		/* I2C */                           \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SDA),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SDA),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SDA),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SCL),  \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SDA),  \
		/* GPIO */                          \
		GPIO_WDOG,                          \
		GPIO_FMU_CAP1,                      \
		GPIO_FMU_CAP2,                      \
	    GPIO_CAN_OEn,                       \
		GPIO_ETH_RMII_POWER_EN,             \
		GPIO_SAFETY_LEDn,                   \
		GPIO_PWM_OEn,                       \
		GPIO_PWM_VOLTAGE_SEL_5V_3Vn,        \
		GPIO_SAFETY_SWITCH_IN,              \
		GPIO_SD_CARD_IN,                    \
		GPIO_TERMCAN1_EN,                   \
		GPIO_TERMCAN2_EN,                   \
		GPIO_TERMCAN2_EN,                   \
		GPIO_nARMED,                        \
		GPIO_BUZZER_1,                      \
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
