/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic low level driver for GPS receiver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"

/*!
 * FIFO buffers size
 */
#define FIFO_TX_SIZE                                128
#define FIFO_RX_SIZE                                128

uint8_t TxBuffer[FIFO_TX_SIZE];
uint8_t RxBuffer[FIFO_RX_SIZE];

/*!
 * \brief Buffer holding the  raw data received from the gps
 */
uint8_t NmeaString[128];

/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */
uint8_t NmeaStringSize = 0;

bool asleep = false;
uint8_t wakeup_retries = 5;

TimerEvent_t GpsLedTimer, GpsResetTimer, GpsWakeupTimer;

static void OnGpsLedTimerEvent( void )
{
    GpioWrite( &Led4, 0 );
}

static void OnGpsResetTimerEvent( void )
{
    GpioWrite( &GpsReset, 0 );
    GpioWrite( &Led4, 0 );

    wakeup_retries = 5;
    GpsMcuWake();
}

static void OnGpsWakeupTimerEvent( void )
{
    if ( asleep )
    {
	if ( wakeup_retries-- )
	    GpsMcuWake();
	else
	    GpsMcuReset();
    }
}

void GpsMcuInit( void )
{
    NmeaStringSize = 0;

    TimerInit( &GpsLedTimer, OnGpsLedTimerEvent );
    TimerSetValue( &GpsLedTimer, 25 );
    TimerInit( &GpsResetTimer, OnGpsResetTimerEvent );
    TimerSetValue( &GpsResetTimer, 500 );
    TimerInit( &GpsWakeupTimer, OnGpsWakeupTimerEvent );
    TimerSetValue( &GpsWakeupTimer, 2000 );

    GpsMcuReset( );
}

/* NMEA checksums from http://www.hhhh.org/wiml/proj/nmeaxor.html */

void GpsMcuWake( void )
{
    char sleep_cmd[] = "$PMTK225,0*2B\r\n";

    FifoInit( &Uart1.FifoRx, RxBuffer, FIFO_RX_SIZE );
    FifoInit( &Uart1.FifoTx, TxBuffer, FIFO_TX_SIZE );
    UartInit( &Uart1, UART_1, UART_TX, UART_RX );
    UartConfig( &Uart1, RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    Uart1.IrqNotify = GpsMcuIrqNotify;
    UartPutBuffer( &Uart1, (uint8_t*)sleep_cmd, 15);

    TimerStart( &GpsWakeupTimer );
}

void GpsMcuReset( void )
{
    GpioWrite( &Led4, 1 );
    GpioWrite( &GpsReset, 1 );
    TimerStart( &GpsResetTimer );
}

void GpsMcuSleep( void )
{
    if ( !asleep ) {
	char wake_cmd[] = "$PMTK161,0*28\r\n";
	UartPutBuffer( &Uart1, (uint8_t*)wake_cmd, 15 );
	UartDeInit( &Uart1 );
	asleep = true;
	GpsResetCounter();
    }
}

void GpsMcuIrqNotify( UartNotifyId_t id )
{
    uint8_t data;
    if( id == UART_NOTIFY_RX )
    {
	asleep = false;
	if( UartGetChar( &Uart1, &data ) == 0 )
        {
            if( ( data == '$' ) || ( NmeaStringSize >= 128 ) )
            {
                NmeaStringSize = 0;
		memset(NmeaString, 0, 128);
            }

            NmeaString[NmeaStringSize++] = ( int8_t )data;

            if( data == '\n' )
            {
                NmeaString[NmeaStringSize] = '\0';
	        if (GpsParseGpsData( ( int8_t* )NmeaString, NmeaStringSize )) {
		    GpioWrite(&Led4, 1);
		    TimerStart( &GpsLedTimer );
		}
	    }
        }
    }
}
