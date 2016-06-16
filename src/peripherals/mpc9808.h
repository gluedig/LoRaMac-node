#ifndef __MPC9808_H__
#define __MPC9808_H__

/*
 * I2C address
 */
#define MPC9808_I2C_ADDRESS	0x18


/* Registers */
#define MPC9808_REG_CONFIG	0x1
#define MPC9808_REG_LIMIT_UP	0x2
#define MPC9808_REG_LIMIT_LOW	0x3
#define MPC9808_REG_LIMIT_CRIT	0x4
#define MPC9808_REG_TEMP	0x5
#define MPC9808_REG_MANUF_ID	0x6
#define MPC9808_REG_DEV_ID	0x7
#define MPC9808_REG_RES		0x8

/*!
 * \brief Initializes the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MPC9808Init( void );

/*!
 * \brief Reads the Temperature from the MPC9808
 *
 * \retval temperature Measured temperature
 */
int16_t MPC9808ReadTemperature( void );

void MPC9808Shutdown( bool );

#endif /* __MPC9808_H__ */

