/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic driver for GPS receiver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __GPS_BOARD_H__
#define __GPS_BOARD_H__


bool GpsGotMsg( void );

/*!
 * \brief Low level Initialisation of the UART and IRQ for the GPS
 */
void GpsMcuInit( void );

/*!
 * \brief IRQ handler for the UART receiver
 */
void GpsMcuIrqNotify( UartNotifyId_t id );

void GpsMcuSleep( void );
void GpsMcuWake( void );
void GpsMcuReset( void );


#endif  // __GPS_BOARD_H__
