/*
 * MCP2515_spi.h
 *
 *  Created on: Jan 5, 2014
 *      Author: Jenn
 */
#ifndef SPI_H_
#define SPI_H_

extern void SR_SPI(unsigned int length, unsigned int *buf);
extern void SR2_SPI(unsigned int byte1, unsigned int byte2, unsigned int length, unsigned int *buf);
extern void MCP2515_reset(unsigned int rst);
extern void MCP2515_spi_init();

#endif /* SPI_H_ */
