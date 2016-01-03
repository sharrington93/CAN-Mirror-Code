/*
 * MCP2515_spi.h
 *
 *  Created on: Jan 5, 2014
 *      Author: Jenn
 */
#ifndef SPI_H_
#define SPI_H_

extern void SR_SPI(unsigned int length, unsigned int *buf);
extern int SR2_SPI(unsigned int byte1, unsigned int byte2, unsigned int rx_length, unsigned int *message_buf);
extern int Read_RX_SPI(unsigned int address, unsigned int *message_buf, unsigned int rx_length);
extern void MCP2515_reset(unsigned int rst);
extern void MCP2515_spi_init();

#endif /* SPI_H_ */
