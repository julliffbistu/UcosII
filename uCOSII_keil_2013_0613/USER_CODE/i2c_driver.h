/*****************************************************************************
 *   i2c.h:  Header file for NXP LPC17xx Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.26  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __I2C_H 
#define __I2C_H



#define I2CMASTER		0x01
#define I2CSLAVE		0x02
#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */

#define I2SCLH_SCLH			0x00000080  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL			0x00000080  /* I2C SCL Duty Cycle Low Reg */

extern void I2C0_IRQHandler( void );
extern INT32U I2CInit(void);
INT8U I2C_WriteNByte(INT8U sla, INT8U suba_type, INT32U suba, INT8U *s, INT32U num);
INT8U I2C_ReadNByte (INT8U sla, INT8U suba_type, INT32U suba, INT8U *s, INT32U num);


#endif /* end __I2C_H */
/****************************************************************************
**                            End Of File
*****************************************************************************/
