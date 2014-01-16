//This library is non-portable, it uses C2000 specific _byte() intrinsic for bytewise access to memory

// this file contain functions to use the Microchip MCP2515 CAN controller
// User must provide SR_SPI(uint16 length, uint16 *buf); which sends the bytes in buf out on the SPI port, and returns the response in buf. (each byte sent out is stored in a uint16)
// User must provide SR2_SPI which works the same as SR_SPI except the first two bytes are passed as arguments.
// User must provide MCP2515_reset(uint16 rst); which sets the pin used as the MCP2515 reset high or low (rst = 1 is pin low, rst = 0  is pin high)

#include "MCP2515_DEFS.h"
#include "MCP2515_spi.h"

extern void DSP28x_usDelay(unsigned long Count);
#define CPU_RATE   16.667L   // for a 60MHz CPU clock speed (SYSCLKOUT)
#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)

//function prototypes
void MCP2515Mode(unsigned int Mode);
int MCP2515SendMessage(unsigned int ID, unsigned long ExtID, unsigned int DataL, unsigned int* Data);
int MCP2515GetMessage(unsigned int *dmesg);
int MCP2515GetMessageAvailable(void);
void MCP2515Mode(unsigned int Mode);
void PgmInitMCP2515(unsigned int cnf1, const unsigned int *data);
void MCP2515LoadTx(unsigned int n, unsigned int sid, unsigned long eid, unsigned int dl, unsigned int *data);
unsigned int MCP2515Read(unsigned int Addr);
void MCP2515Write(unsigned int Addr, unsigned int Data);

//*****************************************************************************
//sends a CAN message.
//ID[15:5] are the std ID[10:0], bit[3] is the Ext ID bit. if this is set the frame is extended. bits [1:0] are extended ID bits 17:16]
//ExtID[15:0] are the extended identifier bits [15:0] 
//DataL bit [6] is the RTR bit, bits [3:0] are the data length bits.
//Data is a buffer of length (DataL & 0x0f), ie the 4 bit data length code. max of 8 bytes.
int MCP2515SendMessage(unsigned int ID, unsigned long ExtID, unsigned int DataL, unsigned int* Data)
{
	int i;
	int tmp;
	MCP2515LoadTx(0, ID, ExtID, DataL, Data);						//load tx buffer
	MCP2515Write(MCP_TXB0CTRL,0x0B);							//flag message for sending
	i=0;											//basic timeout. would be better to use a system tick
	do
	{
		tmp = (0x78 & MCP2515Read(MCP_TXB0CTRL));					//check status of message
		if (tmp == 0x00) return 0;							//success
		i++;
	}
	while(i < 5000);

	MCP2515Write(MCP_TXB0CTRL,0x00);					//clear message send flag
	return 1;
}

//*****************************************************************************
//Reads a single register of the MCP2515
unsigned int MCP2515Read(unsigned int Addr)
{
	unsigned int buf[3];
	buf[0] = MCP_READ;
	buf[1] = Addr;
	buf[2] = 0x00;
	SR_SPI(3,buf);
	return buf[2];
}

//*****************************************************************************
//writes a single register of the MCP2515
void MCP2515Write(unsigned int Addr, unsigned int Data)
{
	unsigned int buf[3];
	buf[0] = MCP_WRITE;
	buf[1] = Addr;
	buf[2] = Data;
	SR_SPI(3,buf);
}

//*****************************************************************************
//This function sets a Mask
//must put MCP2515 in configuration mode first.
//configuration mode is handled outside function to avoid unnecessary mode switching if other config functions are called.
//if function is called when MCP2515 is not in configuration mode, returns 1 and does nothing, otherwise returns 0 and writes the Mask data
// buf is assumed to point to 4 consecutive bytes of Mask configuration data, first byte = RXMnSIDH, last byte is RXMnEID0
int MCP2515SetMask(unsigned int N, unsigned int *buf)
{
	unsigned int addr;

	//check the current mode is configuration
	if ((MCP2515Read(MCP_CANSTAT) >> 5) == 0x04)
	{
		//write filter data
		switch(N)
		{
		case 0:
			addr = MCP_RXM0SIDH;	
		break;
		default:
			addr = MCP_RXM1SIDH;	
		break;
		}
		SR2_SPI(MCP_WRITE, addr, 4, buf);
		return 0;
	}
	else
	{
		return 1;
	}
}

//*****************************************************************************
//this function sets a filter
//must put MCP2515 in configuration mode first.
//configuration mode is handled outside function to avoid unnecessary mode switching if other config functions are called.
//if function is called when MCP2515 is not in configuration mode, returns 1 and does nothing, otherwise returns 0 and writes the Filter data
// buf is assumed to point to 4 consecutive bytes of Filter configuration data, first byte = RXFnSIDH, last byte is RXFnEID0
int MCP2515SetFilter(unsigned int N, unsigned int *buf)
{
	unsigned int addr;

	//check the current mode is configuration
	if ( (MCP2515Read(MCP_CANSTAT) >> 5) == 0x04)
	{
		switch(N)
		{
		case 0:
			addr = MCP_RXF0SIDH;	
		break;
		case 1:
			addr = MCP_RXF1SIDH;	
		break;
		case 2:
			addr = MCP_RXF2SIDH;	
		break;
		case 3:
			addr = MCP_RXF3SIDH;	
		break;
		case 4:
			addr = MCP_RXF4SIDH;	
		break;
		default:
			addr = MCP_RXF5SIDH;	
		break;
		}
		SR2_SPI(MCP_WRITE, addr, 4, buf);
		return 0;
	}
	else
	{
		return 1;
	}
}


//*****************************************************************************
//this function sets the bit timing for the MCP2515
//must put MCP2515 in configuration mode first.
//configuration mode is handled outside function to avoid unnecessary mode switching if other config functions are called.
//if function is called when MCP2515 is not in configuration mode, returns 1 and does nothing, otherwise returns 0 and writes the bit timing data
unsigned int MCP2515SetBitTiming(unsigned int cnf1, unsigned int cnf2, unsigned int cnf3)
{
	unsigned int buf[5];

	//check the current mode aginst the requested one
	if ( (MCP2515Read(MCP_CANSTAT) >> 5) == 0x04)
	{
		//set bit timing (CNF1, CNF2, CNF3) 
		// CNF 3,2,1 are sequential so can all be written by one SPI transaction
		buf[0] = MCP_WRITE;
		buf[1] = MCP_CNF3;
		buf[2] = cnf3;			//value to write to CNF3
		buf[3] = cnf2;			//value to write to CNF2
		buf[4] = cnf1;
		SR_SPI(5,buf);
		return 0;
	}
	else
	{
		return 1;
	}
}

//*****************************************************************************
//This function initializes the MCP2515, with values stored in program memory.
//there are a total of 4bytes/mask or filter and 8 masks/filters for a total of 32 bytes
// the order of registers is mask0,mask1,filter0..filter5. So the first 8 bytes are RXM0SIDH,RXM0SIDL,RXM0EID8,RXM0EID0,RXM1SIDH,RXM1SIDL,RXM1EID8,RXM1EID0 ...
//Mask bits which are 0 accept all message bits, mask bits which are 1 require the message bit to match a filter bit
//Each Mask or filter has a total of 32 bits in 4 registers.
// for standard frames, the extended ID bits in array[2,3] are applied to the first two data bytes
// mask0 and filter0, filter1 are associated with recieve buffer 0
// mask1 and filters 2-5 are associated with recieve buffer 1
void PgmInitMCP2515(unsigned int cnf1, const unsigned int *data)
{

	unsigned int buf[12];
	unsigned int i;

	MCP2515_reset(1);			//hold MCP2515 in reset
	MCP2515_reset(1);			//hold MCP2515 in reset

	DELAY_US(100);

	MCP2515_reset(0);			//release MCP2515 from reset
	MCP2515_reset(0);			//release MCP2515 from reset

	DELAY_US(100);

	//put MCP2515 is in configuration mode
	MCP2515Mode(0x04);

	//set bit timing (CNF1, CNF2, CNF3). arrange for exactly 10tq in a bittime
	//CNF2 controls sample point, prop and PH1. is always 0xD2. sample 3 times, prop = 3tq, phase1=3tq
	//CNF3 controls sof output, wake filter and phase2. is always 0x82, sof enabled, no wake filter, phase2=3tq
	MCP2515SetBitTiming(cnf1,0xD2,0x82);
	
	//set TXRTSCTRL	(set pins to general input, use spi to initiate transmission => TXRTSCTL = 0x00)
	MCP2515Write(MCP_TXRTSCTRL, 0x00);

	//set BFPCTRL (set pins to interrupt outputs => 0x0F)
	MCP2515Write(MCP_BFPCTRL, 0x0F);

	//set CANINTE for interrupt enable (want interrupts on RX0 & RX1, => 0x03)
	MCP2515Write(MCP_CANINTE,0x03);
	
	//set mask/filter registers
	//masks 0,1 are contiguous, can set them with a single write
	for(i=0;i<8;i++) buf[i]=data[i];
	SR2_SPI(MCP_WRITE, MCP_RXM0SIDH, 8, buf);	//will take values data[0] - data[7]
	 //filters 0,1,2 are contiguous, can set them with a single write
	for(i=0;i<12;i++) buf[i]=data[i+8];
	SR2_SPI(MCP_WRITE, MCP_RXF0SIDH, 12, buf);	//will take values data[8] - data[19]
	//filters 3,4,5 are contiguous, can set them with a single write
	for(i=0;i<12;i++) buf[i]=data[i+20];
	SR2_SPI(MCP_WRITE, MCP_RXF3SIDH, 12, buf);	//will take values data[20] - data[31]

	//set normal mode
	MCP2515Mode(0x00);
}

//*****************************************************************************
//this function places the MCP2515 in the specified mode.
//function will block untill the mode switch is sucessful.
void MCP2515Mode(unsigned int Mode)
{
	unsigned int Flag;
	Flag = 1;
	unsigned int buf[4];
	do
	{
		//check the current mode aginst the requested one
		if ( (MCP2515Read(MCP_CANSTAT) >> 5)!= (Mode & 0x07))
		{
			//current mode is different from the one requested, send mode switch command
			//prepare write transaction
			buf[0] = MCP_BITMOD;		//spi command
			buf[1] = MCP_CANCTRL;		//register address
			buf[2] = 0xE0;			//mask
			buf[3] = ((0x07 & Mode) << 5); 	//mode request
			SR_SPI(4,buf);			//send transaction
			//pause

			DELAY_US(100);
		}
		else
		{
			//current mode is the one requested, finished.
			Flag = 0;
		}
	}	
	while(Flag);	//check result
}

//*****************************************************************************
//this function loads a transmit buffer
// must make sure the TXBnCTRL.TXREQ bit is clear before calling this function
//note only bits 10:0 in sid are valid identifier bits. bit 15 in sid is used for the EXIDE flag
//only bits 17:0 in eid are valid identifier bits

void MCP2515LoadTx(unsigned int n, unsigned int sid, unsigned long eid, unsigned int dl, unsigned int *data)
{
	unsigned int i;
	unsigned int buf[13];					//buffer for all the possible bytes in the transmit buffer

	buf[0] = ((sid >> 3) & 0xFF);		//first reg is bits 10:3 of SID
	buf[1] = ((sid & 0x07) << 5);		//put sid 2:0 in buf[1] 7:5
	buf[1] |= ((sid & 0x8000) >> 12);		//sid:16 is EXIDE flag
	buf[1] |= ((eid & 0x00030000 ) >> 16);	//sid1:0 are eid 17:16
	buf[2] = ((eid >> 8) & 0xFF);
	buf[3] = (eid & 0xFF);
	buf[4] = dl & 0x0F;			//data length
	for(i=0;i<dl;i++)				//data bytes
		buf[5+i] = data[i];

	switch(n)
	{
	case 0:	SR2_SPI(MCP_WRITE, MCP_TXB0SIDH, 5+buf[4], buf); break;	//write values to buffer
	case 1:	SR2_SPI(MCP_WRITE, MCP_TXB1SIDH, 5+buf[4], buf); break;	//write values to buffer
	case 2:	SR2_SPI(MCP_WRITE, MCP_TXB2SIDH, 5+buf[4], buf); break;	//write values to buffer
	}
}

int MCP2515GetMessageAvailable(void)
{
	unsigned int c;
	//returns -1 if no message available, 0 if message in RXB0, 1 if message in RXB1
	c = MCP2515Read(MCP_CANINTF);		//get interrupt flag register
	if ( c & 0x01) return 0;
	if ( c & 0x02) return 1;
	return -1;
}

int MCP2515GetMessage(unsigned int *dmesg)
{

	switch(MCP2515GetMessageAvailable())
	{
	case 0:
		SR2_SPI(MCP_READ, MCP_RXB0SIDH, 13, dmesg);	//read raw can message from RXB0
		MCP2515Write(MCP_CANINTF, 0x00);		//clear interrupt
		return 0;
	case 1:
		SR2_SPI(MCP_READ, MCP_RXB1SIDH, 13, dmesg);	//read raw can message from RXB1
		MCP2515Write(MCP_CANINTF, 0x00);		//clear interrupt
		return 1;
	default:
		return -1;
	}
}

