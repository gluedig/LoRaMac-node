#include "board.h"
#include "mpc9808.h"

/*!
 * I2C device address
 */
static uint8_t MPC9808DeviceAddr = 0;
/*!
 * Indicates if the MPL3115 is initialized or not
 */
static bool MPC9808Initialized = false;

/*!
 * \brief Writes a word at specified address in the device
 *
 * \param [IN]:    addr
 * \param [IN]:    data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MPC9808Write( uint8_t addr, uint16_t data );

/*!
 * \brief Reads a word at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MPC9808Read( uint8_t addr, uint16_t *data );

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void MPC9808SetDeviceAddr( uint8_t addr );

uint8_t MPC9808Init( void )
{
    uint16_t regVal = 0;

    MPC9808SetDeviceAddr( MPC9808_I2C_ADDRESS );

    if( MPC9808Initialized == false )
    {
        MPC9808Read( MPC9808_REG_MANUF_ID, &regVal );
        if( regVal != 0x54 )
        {
            return FAIL;
        }
	regVal = 0;
        MPC9808Read( MPC9808_REG_DEV_ID, &regVal );
        if( (regVal & 0xFF00 ) != 0x0400 )
        {
            return FAIL;
        }
	MPC9808Write( MPC9808_REG_CONFIG, 0 );
	MPC9808Initialized = true;
    }
    return SUCCESS;
}

uint8_t MPC9808Write( uint8_t addr, uint16_t data )
{
    uint8_t out[2];
    out[0] = (data & 0xFF00) >> 8;
    out[1] = data & 0x00FF;
    return I2cWriteBuffer( &I2c, MPC9808DeviceAddr, addr, out, 2 );
}

uint8_t MPC9808Read( uint8_t addr, uint16_t *data )
{
    uint8_t in[2], retval;

    retval = I2cReadBuffer( &I2c, MPC9808DeviceAddr, addr, in, 2 );
    *data = in[0] << 8;
    *data |= in[1];
    return retval;
}

void MPC9808SetDeviceAddr( uint8_t addr )
{
    MPC9808DeviceAddr = (addr << 1 ) & 0x3E;
}

int16_t MPC9808ReadTemperature( void )
{
    uint16_t regval = 0;
    int16_t temp = 0;
    if ( !MPC9808Initialized )
	return 0;

    MPC9808Read( MPC9808_REG_TEMP, &regval);
    temp = ((regval & 0xF00) >> 4) | ((regval & 0xF0) >> 4);
    temp *= 100;
    temp += ((regval & 0xF) * 625) / 100;

    /* negative ? */
    if ( regval & 0x1000)
	temp = -temp;

    return temp;
}

void MPC9808Shutdown( bool down)
{
    uint16_t val;
    MPC9808Read( MPC9808_REG_CONFIG, &val);
    if ( down ) {
	MPC9808Write( MPC9808_REG_CONFIG, val | 0x0100 );
    } else {
	MPC9808Write( MPC9808_REG_CONFIG, val ^ 0x0100 );
    }
}

