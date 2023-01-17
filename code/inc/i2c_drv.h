/*****************************************************************************
* i2c_drv.h
*
*  Driver hearder for AVT TWI (I2C) interface for
*  bus controller 'i2c_c_*' and peripheral 'i2c_p_*'.
*  This is driver can be use as interrupt driven.
*
*  Created: December 2014
*
*****************************************************************************/

#ifndef __I2C_DRV_H__
#define __I2C_DRV_H__

#include    <stdint.h>

/****************************************************************************
  Definitions
****************************************************************************/
#define TWI_BUFF_LEN    14 // data byte buffer

/****************************************************************************
  Function prototypes
****************************************************************************/
void    i2c_c_initialize(void);
int     i2c_c_rx_data(uint8_t, uint8_t *, int);
int     i2c_c_tx_data(uint8_t, uint8_t *, int);
int     i2c_c_tx_byte(uint8_t, uint8_t, uint8_t);
int     i2c_c_rx_byte(uint8_t, uint8_t, uint8_t *);
int     i2c_c_burst_read(uint8_t, uint8_t, uint8_t, uint8_t *);

void    i2c_p_initialize(uint8_t, uint8_t);
int     i2c_p_rx_data(uint8_t *, uint8_t);
int     i2c_p_tx_data(uint8_t *, uint8_t);

#endif /* __I2C_DRV_H__ */
