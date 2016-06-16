/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32l1xx.h"
#include "stm32l1xx_hal.h"
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "radio.h"
#include "sx1272/sx1272.h"
#include "gps.h"
#include "gps-board.h"
#include "rtc-board.h"
#include "sx1272-board.h"
#include "uart-board.h"
#include "mpc9808.h"

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PA_2

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5
#define RADIO_NSS                                   PB_0

#define RADIO_DIO_0                                 PB_1
#define RADIO_DIO_1                                 PB_10
#define RADIO_DIO_2                                 PB_11
#define RADIO_DIO_3                                 PB_7
#define RADIO_DIO_4                                 PB_5
#define RADIO_DIO_5                                 PB_4

#define RADIO_ANT_SWITCH_RX                         PC_13
#define RADIO_ANT_SWITCH_TX                         PA_4

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9

#define LED_1                                       PA_1
#define LED_2                                       PA_8
#define LED_3                                       PA_0
#define LED_4					    PA_3

#define GPS_WAKE				    PA_11
#define GPS_RESET				    PA_12

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define PIN_3					    PB_3
#define PIN_4					    PA_15
#define PIN_7					    PB_14
#define PIN_8					    PB_15
#define PIN_9					    PB_13
#define PIN_10					    PB_12


/*!
 * GPIO pins objects
 */
extern Gpio_t GpsReset;
extern Gpio_t Led1;
extern Gpio_t Led2;
extern Gpio_t Led3;
extern Gpio_t Led4;

/*!
 * MCU objects
 */
extern I2c_t I2c;
extern Uart_t Uart1;
#if defined( USE_USB_CDC )
extern Uart_t UartUsb;
#endif

enum BoardPowerSource
{
    USB_POWER = 0,
    BATTERY_POWER
};

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source ( 0: USB_POWER,  1: BATTERY_POWER )
 */
uint8_t GetBoardPowerSource( void );

void BoardSleepPeriph( void );
void BoardWakePeriph( void );

#endif // __BOARD_H__
