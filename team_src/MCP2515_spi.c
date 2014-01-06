/*
 * MCP2515_spi.c
 *
 *  Created on: Jan 5, 2014
 *      Author: Jenn
 */

#include "all.h"

#define	SPI_CS_LOW()	GpioDataRegs.GPACLEAR.bit.GPIO15 = 0
#define	SPI_CS_HIGH()	GpioDataRegs.GPACLEAR.bit.GPIO15 = 1

//function prototypes
void SR_SPI(unsigned int length, unsigned int *buf);
void SR2_SPI(unsigned int byte1, unsigned int byte2, unsigned int length, unsigned int *buf);
void MCP2515_reset(unsigned int rst);

void SR_SPI(unsigned int length, unsigned int *buf)
{
	unsigned int i;

	//todo figure out the C2000 equiv of this //while (!(UCSRnA & (1<<UDRE0)));		//wait for transmitter ready	
	SPI_CS_LOW();						//set CS low
	for(i=0;i<length;i++)					//loop over length
	{
		SpiaRegs.SPITXBUF=(buf[i]<<8);			//send byte		
		while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);	//wait for response
		buf[i] = SpiaRegs.SPIRXBUF;			//save response
	}
	SPI_CS_HIGH();						//set CS high
}

void SR2_SPI(unsigned int byte1, unsigned int byte2, unsigned int length, unsigned int *buf)
{
	unsigned int i;

	//todo figure out the C2000 equiv of this //while (!(UCSRnA & (1<<UDRE0)));		//wait for transmitter ready	
	SPI_CS_LOW();						//set CS low

	SpiaRegs.SPITXBUF=(byte1<<8);			//send byte		
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);	//wait for response
	i = SpiaRegs.SPIRXBUF;				//dummy read

	SpiaRegs.SPITXBUF=(byte2<<8);			//send byte		
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);	//wait for response
	i = SpiaRegs.SPIRXBUF;				//dummy read

	for(i=0;i<length;i++)					//loop over length
	{
		SpiaRegs.SPITXBUF=(buf[i]<<8);			//send byte		
		while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);	//wait for response
		buf[i] = SpiaRegs.SPIRXBUF;			//save response
	}
	SPI_CS_HIGH();						//set CS high
}

void MCP2515_reset(unsigned int rst)
{
	if (rst)
	{
		GpioDataRegs.GPACLEAR.bit.GPIO11 = 0;
	}
	else
	{
		GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	}
}

void MCP2515_spi_init()
{
// Initialize SPI FIFO registers
   SpiaRegs.SPICCR.bit.SPISWRESET=0;     // Reset SPI

   SpiaRegs.SPICCR.all=0x0007;           //8-bit no loopback

   SpiaRegs.SPICTL.all=0x0006;           //// Enable master mode, normal phase,enable talk, and SPI int disabled.

   SpiaRegs.SPISTS.all=0x0000;
   SpiaRegs.SPIBRR = 127;                //Baudrate is slow as possible
  //SpiaRegs.SPIBRR=0x0063;              // Baud rate

   SpiaRegs.SPIFFTX.all=0xC021;          // Enable FIFO's, set TX FIFO level to 1 CHOOSE LEVEL ACCORDING TO APPLICATION

   SpiaRegs.SPIFFRX.all=0x0021;          // Set RX FIFO level to 1
   SpiaRegs.SPIFFCT.all=0x00;

   SpiaRegs.SPIPRI.bit.FREE=1;

   SpiaRegs.SPICCR.bit.SPISWRESET=1;      // Enable SPI

  SpiaRegs.SPIFFTX.bit.TXFIFO=1;
  SpiaRegs.SPIFFTX.bit.SPIRST=1;
  SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

  //initialize GPIO pins ~CS, MCP2515 reset, MCP2515 int, rx0bf, rx1bf

    EALLOW;
    // SPI Chip Select line
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;          // output
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 0;        //Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;          //disable pull up

    // MCP2515 reset line
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;          // output
    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0;        //Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;          //disable pull up

    // MCP2515 interrupt line
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;          // input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;        //Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;          //enable pull up

    // MCP2515 !RX0BF line
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;          // input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0;        //Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;          //enable pull up

    // MCP2515 !RX1BF line
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;         // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;          // input
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;        //Synch to SYSCLKOUT only
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;          //enable pull up
    EDIS;

}

