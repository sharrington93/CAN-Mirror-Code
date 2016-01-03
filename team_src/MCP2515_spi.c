/*
 * MCP2515_spi.c
 *
 *  Created on: Jan 5, 2014
 *      Author: Jenn
 */

#include "all.h"

#define	SPI_CS_LOW()	GpioDataRegs.GPACLEAR.bit.GPIO15 = 1
#define	SPI_CS_HIGH()	GpioDataRegs.GPASET.bit.GPIO15 = 1

//function prototypes
void SR_SPI(unsigned int length, unsigned int *buf);
void SR2_SPI(unsigned int byte1, unsigned int byte2, unsigned int length, unsigned int *buf);
void MCP2515_reset(unsigned int rst);

extern buffer_struct Buf_Ato2;

unsigned int *current_message;
unsigned int byte_num;
unsigned int tx_bytes;

void SR_SPI(unsigned int length, unsigned int *buf)
{
	unsigned int i;

	DINT;										//disable interrupts during sending

	SPI_CS_LOW();								//set CS low
	for(i=0;i<length;i++)						//loop over length
	{
		SpibRegs.SPITXBUF=(buf[i]<<8);			//send byte
		while(SpibRegs.SPIFFRX.bit.RXFFST !=1);	//wait for response
		buf[i] = SpibRegs.SPIRXBUF;				//save response
	}

	EINT;										//re-enable interrupts
}

// Deprecated
void SR2_SPI(unsigned int byte1, unsigned int byte2, unsigned int length, unsigned int *buf)
{
	unsigned int i;

	DINT;										//disable interrupts during sending

	SPI_CS_LOW();								//set CS low

	SpibRegs.SPITXBUF=(byte1<<8);				//send byte
	while(SpibRegs.SPIFFRX.bit.RXFFST !=1);		//wait for response
	i = SpibRegs.SPIRXBUF;						//dummy read

	SpibRegs.SPITXBUF=(byte2<<8);				//send byte
	while(SpibRegs.SPIFFRX.bit.RXFFST !=1);		//wait for response
	i = SpibRegs.SPIRXBUF;						//dummy read

	for(i=0;i<length;i++)						//loop over length
	{
		SpibRegs.SPITXBUF=(buf[i]<<8);			//send byte
		while(SpibRegs.SPIFFRX.bit.RXFFST !=1);	//wait for response
		buf[i] = SpibRegs.SPIRXBUF;				//save response
	}
	SPI_CS_HIGH();								//set CS high

	EINT;										//enable interrupts
}

void READ_RX_SPI(unsigned int address, unsigned int *buf, unsigned int tx_length)
{
	current_message = buf;
	byte_num = 0;
	tx_bytes = tx_length;
	DINT;

	SPI_CS_LOW();
	SpibRegs.SPITXBUF=(address<<8);				//send byte

	EINT;										//enable interrupts
}

void MCP2515_reset(unsigned int rst)
{
	if (rst)
	{
		GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
	}
	else
	{
		GpioDataRegs.GPASET.bit.GPIO11 = 1;
	}
}

void MCP2515_spi_init()
{
// Initialize SPI FIFO registers
   SpibRegs.SPICCR.bit.SPISWRESET=0;     // Reset SPI

   SpibRegs.SPICCR.all=0x0047;           //8-bit no loopback

   SpibRegs.SPICTL.all=0x0006;           //// Enable master mode, normal phase,enable talk, and SPI int disabled.

   SpibRegs.SPISTS.all=0x0000;
   SpibRegs.SPIBRR = 6;                	 //MCP2515 can take 10MHz SPI, set to 8.5MHz ( 60MHz/(SPIBRR+1) ) to be safe

   SpibRegs.SPIFFTX.all=0xC021;          // Enable FIFO's, set TX FIFO level to 1 CHOOSE LEVEL ACCORDING TO APPLICATION

   SpibRegs.SPIFFRX.all=0x0021;          // Set RX FIFO level to 1
   SpibRegs.SPIFFCT.all=0x00;

   SpibRegs.SPIPRI.bit.FREE=1;

   SpibRegs.SPICCR.bit.SPISWRESET=1;      // Enable SPI

   SpibRegs.SPIFFTX.bit.TXFIFO=1;
   SpibRegs.SPIFFTX.bit.SPIRST=1;
   SpibRegs.SPIFFRX.bit.RXFIFORESET=1;

  //initialize GPIO pins ~CS, MCP2515 reset, MCP2515 int, rx0bf, rx1bf

    EALLOW;
    // SPI Chip Select line
    GpioDataRegs.GPADAT.bit.GPIO15 = 1;			//ensure CS is high
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;          // output
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 0;        //Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;          //disable pull up
    GpioDataRegs.GPADAT.bit.GPIO15 = 1;			//ensure CS is high

    // MCP2515 reset line
    GpioDataRegs.GPADAT.bit.GPIO11 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;         // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;          // output
    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0;        //Synch to SYSCLKOUT only
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;          //disable pull up
    GpioDataRegs.GPADAT.bit.GPIO11 = 0;

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

 /* Enable internal pull-up for the selected pins */
 // Pull-ups can be enabled or disabled disabled by the user.
 // This will enable the pullups for the specified pins.
 // Comment out other unwanted lines.

 //  GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;     // Enable pull-up on GPIO12 (SPISIMOB)
     GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;     // Enable pull-up on GPIO24 (SPISIMOB)

     GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;     // Enable pull-up on GPIO13 (SPISOMIB)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;     // Enable pull-up on GPIO25 (SPISOMIB)

     GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;     // Enable pull-up on GPIO14 (SPICLKB)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;     // Enable pull-up on GPIO26 (SPICLKB)

 /* Set qualification for selected pins to asynch only */
 // This will select asynch (no qualification) for the selected pins.
 // Comment out other unwanted lines.

 //  GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;   // Asynch input GPIO12 (SPISIMOB)
     GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3;   // Asynch input GPIO24 (SPISIMOB)

     GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;   // Asynch input GPIO13 (SPISOMIB)
 //  GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;   // Asynch input GPIO25 (SPISOMIB)

     GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;   // Asynch input GPIO14 (SPICLKB)
 //  GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3;   // Asynch input GPIO26 (SPICLKB)

 /* Configure SPI-B pins using GPIO regs*/
 // This specifies which of the possible GPIO pins will be SPI functional pins.
 // Comment out other unwanted lines.

 //  GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 3;    // Configure GPIO12 as SPISIMOB
     GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;    // Configure GPIO24 as SPISIMOB

     GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 3;    // Configure GPIO13 as SPISOMIB
 //  GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;    // Configure GPIO25 as SPISOMIB

     GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;    // Configure GPIO14 as SPICLKB
 //  GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;    // Configure GPIO26 as SPICLKB
    EDIS;

}

// INT6.3
__interrupt void SPIRXINTB_ISR(void)    // SPI-B
{
	current_message[byte_num] = SpibRegs.SPIRXBUF;

	if(byte_num == tx_bytes)		//We have sent and now received all bytes.
	{
		SPI_CS_HIGH();				//set CS high
		// Increase buffer length so that message can be mirrored now
		if (++Buf_Ato2.in == CANQUEUEDEPTH) Buf_Ato2->in = 0;					//increment with wrap
		if (Buf_Ato2->in == Buf_Ato2->out) Buf_Ato2->full = 1;							//test for full
		Buf_Ato2->empty = 0;													//just wrote, can't be empty
		Buf_Ato2->count +=1;
		XINT1_ISR();				//Return to XINT1 to ensure no more flags present
	}

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}
