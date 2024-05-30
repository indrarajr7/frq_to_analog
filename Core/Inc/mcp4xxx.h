/*
 * mcp4xxx.h
 *
 *  Created on: May 25, 2024
 *      Author: Jewel James
 */

#ifndef INC_MCP4XXX_H_
#define INC_MCP4XXX_H_

#include "main.h"

#define I2CP				I2C1

#define DEV_CODE			0b1100
#define ADDR_BITS			0b001 	/* 00 factory default, 1 for A0 pulled HIGH */

#define CMD_WRITE_DAC_REG	0b010
#define CMD_WRITE_FAST_M	0b000 	/* 0 0 X */

#define CFG_PWDN_NORMAL		0b00
#define CFG_PWDN_1K			0b01

/* Address Byte Format
 * D3 D2 D1 D0 A2 A1 A0 R/W
 */
#define ADDR_BYTE_READ		(DEV_CODE << 4 | ADDR_BITS << 1 | 1)
#define ADDR_BYTE_WRITE		(DEV_CODE << 4 | ADDR_BITS << 1 | 0)

void dac_gencallreset(void);
void dac_write(uint16_t value);
uint16_t dac_read(void);

#endif /* INC_MCP4XXX_H_ */
