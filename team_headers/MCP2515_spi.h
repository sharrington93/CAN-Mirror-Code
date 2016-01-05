/*
 * MCP2515_spi.h
 *
 *  Created on: Jan 5, 2014
 *      Author: Jenn
 */
#ifndef SPI_H_
#define SPI_H_

extern void Send_SPI(unsigned int *tx_buffer, unsigned int tx_length, unsigned int *rx_buffer, unsigned int rx_length, void (*function_ptr)());
extern void Fill_SPI_FIFO();
extern void MCP2515_reset(unsigned int rst);
extern void MCP2515_spi_init();

#endif /* SPI_H_ */
