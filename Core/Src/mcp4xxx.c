/*
 * mcp4xxx.c
 *
 *  Created on: May 25, 2024
 *      Author: Jewel James
 */
#include "mcp4xxx.h"

uint8_t RDY, POR, pdwn_cfg;

void i2c_init();
void i2c_transmit(uint8_t* byte, uint8_t count);
uint16_t i2c_receive();

void dac_gencallreset(void) {

}

void dac_write(uint16_t value) {
	uint8_t byte[2];
	byte[0] = CMD_WRITE_FAST_M << 6 | CFG_PWDN_NORMAL << 4;
	byte[0] = byte[0] | (uint8_t)((value & 0xF00) >> 8); /* upper 4 bits */
	byte[1] = (uint8_t)(value & 0xFF); /* lower 8 bits */

	i2c_transmit(byte, 2);
}

uint16_t dac_read() {
	uint16_t word = i2c_receive();

	pdwn_cfg = (word & 0x30) >> 4;
	RDY = (word & 0x80) ? 1:0;
	POR = (word & 0x40) ? 1:0;

	word = word & 0xFFF;

	return word;
}

void i2c_init() {
  	I2CP->CR1 |= (1 << 18); /* Gencall enable */
  	I2CP->CR1 |= I2C_CR1_PE; /* make sure the peripheral enabled*/
}

void i2c_transmit(uint8_t* byte, uint8_t count) {
	uint8_t* p = byte;
  	I2CP->CR2 |= ADDR_BYTE_WRITE << 1; /* Load 7-bit addr */
  	I2CP->CR2 &= ~I2C_CR2_RD_WRN; /* Write bit */
 	I2CP->CR2 |= (count << I2C_CR2_NBYTES_Pos); /* count bytes to transfer */
  	I2CP->CR2 &= ~I2C_CR2_RELOAD; /* 0 for <255 bytes */
  	I2CP->CR2 &= ~I2C_CR2_AUTOEND; /* 0 for no auto-stop */

 	I2CP->CR2 |= (1 << I2C_CR2_START_Pos); /* Generate start */
 	for(size_t i = 0; i < count; i++) {
	  while(!(I2CP->ISR & I2C_ISR_TXIS)); /* Wait till TXDR can be written */
	  I2CP->TXDR = p[i];
	  p = p + 1;
 	}
  	while(!(I2CP->ISR & I2C_ISR_TC)); /* Block till tx complete */
  	I2CP->CR2 |= I2C_CR2_STOP;

  	I2CP->ICR &= ~I2C_ICR_STOPCF; /* Clear stopcf */
  	I2CP->CR2 = 0; /* Clear config */
}

uint16_t i2c_receive() {
	uint16_t word = 0;
  	I2CP->CR2 |= ADDR_BYTE_WRITE << 1; /* Load 7-bit addr */
  	I2CP->CR2 |= I2C_CR2_RD_WRN; /* Read bit */
 	I2CP->CR2 |= (2 << I2C_CR2_NBYTES_Pos); /* count bytes to read */
  	I2CP->CR2 &= ~I2C_CR2_RELOAD; /* 0 for <255 bytes */
  	I2CP->CR2 &= ~I2C_CR2_AUTOEND; /* 0 for no auto-stop */

 	I2CP->CR2 |= (1 << I2C_CR2_START_Pos); /* Generate start */
 	for(size_t i = 0; i < 2; i++) {
 		while(!(I2CP->ISR & I2C_ISR_RXNE)); /* Wait till RXNE is set */
 		word |= (I2CP->RXDR << (8 * (2-i-1)));
 	}
  	while(!(I2CP->ISR & I2C_ISR_TC)); /* Block till rx complete */
  	while(!(I2CP->ISR & I2C_ISR_STOPF)); /* Block till rx complete */

  	I2CP->ICR &= ~I2C_ICR_STOPCF; /* Clear stopcf */
  	I2CP->CR2 = 0; /* Clear config */

	return word;
}
