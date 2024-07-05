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

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* LEDs */
#define GPIO_nLED_RED        /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_nLED_GREEN      /* PE4 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_nLED_BLUE       /* PK6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTK|GPIO_PIN6)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* ADC channels */
#define GPIO_ADC1_INP16           (GPIO_ANALOG|GPIO_PORTA|GPIO_PIN0)
#define GPIO_ADC12_INP18          (GPIO_ANALOG|GPIO_PORTA|GPIO_PIN4)
#define GPIO_ADC123_INP10         (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN0)
#define GPIO_ADC12_INP13          (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN3)
#define GPIO_ADC3_INP5            (GPIO_ANALOG|GPIO_PORTF|GPIO_PIN3)
#define GPIO_ADC3_INP9            (GPIO_ANALOG|GPIO_PORTF|GPIO_PIN4)
#define GPIO_ADC3_INP4            (GPIO_ANALOG|GPIO_PORTF|GPIO_PIN5)
#define GPIO_ADC3_INP8            (GPIO_ANALOG|GPIO_PORTF|GPIO_PIN6)
#define GPIO_ADC1_INP2            (GPIO_ANALOG|GPIO_PORTF|GPIO_PIN11)
#define GPIO_ADC1_INP6            (GPIO_ANALOG|GPIO_PORTF|GPIO_PIN12)
#define GPIO_ADC3_INP15           (GPIO_ANALOG|GPIO_PORTH|GPIO_PIN4)
#define GPIO_ADC3_INP14           (GPIO_ANALOG|GPIO_PORTH|GPIO_PIN3)


#define PX4_ADC_GPIO  \
	/* PA0  */  GPIO_ADC1_INP16,   \
	/* PA4  */  GPIO_ADC12_INP18,  \
	/* PF5  */  GPIO_ADC3_INP4,    \
	/* PF12 */  GPIO_ADC1_INP6,    \
	/* PH4  */  GPIO_ADC3_INP15,   \
	/* PF11 */  GPIO_ADC1_INP2,    \
	/* PC0  */  GPIO_ADC123_INP10, \
	/* PF6  */  GPIO_ADC3_INP8,    \
	/* PC3  */  GPIO_ADC12_INP13,  \
	/* PF4  */  GPIO_ADC3_INP9,    \
	/* PF3  */  GPIO_ADC3_INP5,    \
	/* PH3  */  GPIO_ADC3_INP14


/* Define Channel numbers must match above GPIO pin IN(n)*/
#define BATT_VOLTAGE_SENS      /* PA0  */  16
#define BATT_CURRENT_SENS      /* PA4  */  18
#define BATT2_VOLTAGE_SENS     /* PF5  */  4
#define BATT2_CURRENT_SENS     /* PF12 */  6
#define VDD_5V_SENS            /* PH4  */  15
#define SCALED_V3V3            /* PF11 */  2
#define RSSI_IN                /* PC0  */  10
#define ADC1_6V6               /* PF6  */  8
#define ADC1_3V3               /* PC3  */  13
#define HW_VER_SENSE           /* PF4  */  9
#define HW_REV_SENSE           /* PF3  */  5
#define FMU_SERVORAIL_VCC      /* PH3  */  14

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY1_VOLTAGE_CHANNEL        BATT_VOLTAGE_SENS  /* PA0:  BATT_VOLTAGE_SENS */
#define ADC_BATTERY1_CURRENT_CHANNEL        BATT_CURRENT_SENS  /* PA4: BATT_CURRENT_SENS */
#define ADC_BATTERY2_VOLTAGE_CHANNEL        BATT2_VOLTAGE_SENS /* PF5:  FMU_AUX_POWER_ADC1 */
#define ADC_BATTERY2_CURRENT_CHANNEL        BATT2_CURRENT_SENS /* PF12:  FMU_AUX_ADC2 */
#define ADC_SCALED_V5_CHANNEL               VDD_5V_SENS        /* PH4:  VDD_5V_SENS */
#define ADC_HW_VER_SENSE_CHANNEL            HW_VER_SENSE       /* PF4:  HW_VER_SENSE */
#define ADC_HW_REV_SENSE_CHANNEL            HW_REV_SENSE       /* PF3:  HW_REV_SENSE */

#define ADC_CHANNELS \
	((1 << BATT_VOLTAGE_SENS) | \
	 (1 << BATT_CURRENT_SENS) | \
	 (1 << BATT2_VOLTAGE_SENS) | \
	 (1 << BATT2_CURRENT_SENS) | \
	 (1 << VDD_5V_SENS) | \
	 (1 << SCALED_V3V3) | \
	 (1 << RSSI_IN) | \
	 (1 << ADC1_6V6) | \
	 (1 << ADC1_3V3) | \
	 (1 << HW_VER_SENSE) | \
	 (1 << HW_REV_SENSE) | \
	 (1 << FMU_SERVORAIL_VCC))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* UAVCAN */
#define UAVCAN_NUM_IFACES_RUNTIME  1

/* HEATER
 * PWM in future
 */
#define GPIO_HEATER_OUTPUT   /* PI12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN12)
#define HEATER_CTL(on_true)	       px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  16
#define BOARD_NUM_IO_TIMERS 6

/* Power supply control and monitoring GPIOs */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_BRICK1_VALID          /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1) // VDD_BRICK_VALID
#define GPIO_nVDD_BRICK2_VALID          /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2) // VDD_BACKUP_VALID
#define GPIO_nVDD_USB_VALID             /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3) // VBUS_VALID
#define GPIO_VDD_3V3_SENSORS_EN         /* PJ1  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTJ|GPIO_PIN1) // VDD_3V3_SENSORS_EN
#define GPIO_VDD_3V3_SENSORS2_EN        /* PJ5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTJ|GPIO_PIN5) // VDD_3V3_SENSORS2_EN
#define GPIO_VDD_3V3_SENSORS3_EN        /* PJ4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTJ|GPIO_PIN4) // VDD_3V3_SENSORS3_EN
#define GPIO_VDD_3V3_SENSORS4_EN        /* PG8  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN8) // VDD_3V3_SENSORS4_EN
#define GPIO_nVDD_5V_PERIPH_EN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4) // VDD_5V_PERIPH_EN
#define GPIO_nVDD_5V_PERIPH_OC          /* PJ15 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTJ|GPIO_PIN15) // VDD_5V_PERIPH_OC
#define GPIO_nVDD_5V_HIPOWER_EN         /* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_nVDD_5V_HIPOWER_OC         /* PF13 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTF|GPIO_PIN13) // VDD_5V_HIPOWER_OC
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PJ14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTJ|GPIO_PIN14)
#define GPIO_SD_CARD_EN                 /* PC13 */ (GPIO_INPUT|GPIO_PORTC|GPIO_PIN13)
#define GPIO_EXT_WDOG                   /* PG5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)
#define GPIO_SCHA63T_RESET              /* PI15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN15)
#define GPIO_HW_VER_REV_DRIVE           /* PG0  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN0)

#define GPIO_5V_HEATER_EN               /* PI14 */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN14)
#define GPIO_5V_HEATER_OC               /* PI13 */ (GPIO_INPUT|GPIO_PORTI|GPIO_PIN13)

#define GPIO_CAN1_SILENT                /* PK5  */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTK|GPIO_PIN5)
#define GPIO_CAN2_SILENT                /* PK4  */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTK|GPIO_PIN4)

#define GPIO_BUFFER_OE_EN               /* PK2  */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTK|GPIO_PIN2)   // PWM_OE
#define GPIO_BUFFER_OE2_EN              /* PJ12 */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTJ|GPIO_PIN12)  // PWM_OE2

#define GPIO_FMU_CAP1                   /* PE11 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_FMU_CAP2                   /* PB11 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN11)

#define GPIO_nARMED                     /* PB10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)

#define GPIO_LED_SAFETY                 /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN10)
#define GPIO_SAFETY_SWITCH_IN           /* PJ13 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTJ|GPIO_PIN13) // SAFWTY_IN
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* ETHERNET GPIO */
#define GPIO_ETH_POWER_EN              /* PJ0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTJ|GPIO_PIN0)

/* DRDY2 */
#define GPIO_DRDY2_IIM42652_1  /* PF2 */  (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN2)
#define GPIO_DRDY2_IIM42652_2  /* PK7 */  (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTK|GPIO_PIN7)

/* Tone alarm output */
#define TONE_ALARM_TIMER        14  /* timer 14 */
#define TONE_ALARM_CHANNEL      1  /* PF9 TIM14_CH1 */
#define GPIO_BUZZER_1  /* PF9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)
#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM14_CH1OUT_2

/* PWM input driver. timer3 channel 1 */
#define PWMIN_TIMER                       3
#define PWMIN_TIMER_CHANNEL    /* T3C1 */ 1
#define GPIO_PWM_IN            /* PH6  */ GPIO_TIM12_CH1IN_2 // RCININT PULLDOWN LOW

/* Define True logic Power Control in arch agnostic form */
#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, !(on_true))
#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
#define VDD_3V3_SENSORS2_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS2_EN, (on_true))
#define VDD_3V3_SENSORS3_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS3_EN, (on_true))
#define VDD_3V3_SENSORS4_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS3_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define BUFFER_OE_EN(on_true)              px4_arch_gpiowrite(GPIO_BUFFER_OE_EN, (on_true))
#define BUFFER_OE2_EN(on_true)             px4_arch_gpiowrite(GPIO_BUFFER_OE2_EN, (on_true))
#define ARMED_EN(on_true)                  px4_arch_gpiowrite(GPIO_nARMED, (on_true))
#define VDD_3V3_ETH_POWER_EN(on_true)      px4_arch_gpiowrite(GPIO_ETH_POWER_EN, (on_true))

/* USB
 *  OTG FS: PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)  // VBUS INPUT OPENDRAIN

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

#define GPIO_SDMMC2_CK GPIO_SDMMC2_CK_1
#define GPIO_SDMMC2_CMD GPIO_SDMMC2_CMD_1
#define GPIO_SDMMC2_D2 GPIO_SDMMC2_D2_1

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

//Add a new setting instead of removing BOARD_HAS_HW_VERSIONING
#define BOARD_HAS_STATIC_MANIFEST 2

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_nVDD_BRICK1_VALID,           \
		GPIO_nVDD_BRICK2_VALID,           \
		GPIO_nVDD_USB_VALID,              \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_VDD_3V3_SENSORS2_EN,         \
		GPIO_VDD_3V3_SENSORS3_EN,         \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_nVDD_5V_PERIPH_EN,           \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_nVDD_5V_HIPOWER_EN,          \
		GPIO_nVDD_5V_HIPOWER_OC,          \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SDA), \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_OTGFS_VBUS,                  \
		GPIO_EXT_WDOG,                    \
		GPIO_SCHA63T_RESET,               \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_CAN1_SILENT,                 \
		GPIO_CAN2_SILENT,                 \
		GPIO_BUFFER_OE_EN,                \
		GPIO_BUFFER_OE2_EN,               \
		GPIO_FMU_CAP1,                    \
		GPIO_FMU_CAP2,                    \
		GPIO_nARMED,                      \
		GPIO_LED_SAFETY,                  \
		GPIO_SAFETY_SWITCH_IN,            \
		GPIO_ETH_POWER_EN,                \
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

