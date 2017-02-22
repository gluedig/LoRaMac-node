#ifndef __TMP102_H__
#define __TMP102_H__

/*!
 * TMP102 I2C address
 */
#define TMP102_I2C_ADDRESS                             0x48

/*!
 * TMP102 Registers
 */
#define TMP102_TEMP_REG                                 0x0
#define TMP102_CONF_REG                                 0x01

/*!
 * \brief Initializes the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP102Init( void );

/*!
 * \brief Writes a word at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP102Write( uint8_t addr, uint16_t data );


/*!
 * \brief Reads a word at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP102Read( uint8_t addr, uint16_t *data );

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void TMP102SetDeviceAddr( uint8_t addr );

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t TMP102GetDeviceAddr( void );

uint8_t TMP102ReadTemperature( void );
#endif  // __TMP102_H__
