#include "board.h"
#include "tmp102.h"

static uint8_t I2cDeviceAddr = 0;
static bool TMP102Initialized = false;

uint8_t TMP102Init( void )
{
    TMP102SetDeviceAddr( TMP102_I2C_ADDRESS );

    if( TMP102Initialized == false )
    {
        TMP102Initialized = true;
    }
    return SUCCESS;
}

uint8_t TMP102Write( uint8_t addr, uint16_t data )
{
    return I2cWriteBuffer( &I2c, I2cDeviceAddr << 1, addr, (uint8_t *)data, 2 );
}

uint8_t TMP102Read( uint8_t addr, uint16_t *data )
{
    return I2cReadBuffer( &I2c, I2cDeviceAddr << 1, addr, (uint8_t *)data, 2 );
}

void TMP102SetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t TMP102GetDeviceAddr( void )
{
    return I2cDeviceAddr;
}

uint8_t TMP102ReadTemperature( void )
{
    uint16_t regval = 0xc019;
    uint16_t temp = 0;
    uint8_t out = 0;

    TMP102Read( TMP102_TEMP_REG, &regval );
    /* reverse the read value */
    temp = ((regval & 0xF000) >> 12) | ((regval & 0x7F) << 4);
    /* convert to 0.5 precision */
    temp = ((temp & 0xFF8) * 625) / 5000;

    /* negative ? */
    if ( regval & 0x80)
	out = 0x80;
	temp -= 1;

    /* round */
    if ( ((regval & 0xF000) >> 12) >= 0xD )
        temp += 1;

    return out | (temp & 0x7F);
}
