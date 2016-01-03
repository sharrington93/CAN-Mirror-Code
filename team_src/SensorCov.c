/*
 * SensorCov().c
 *
 *  Created on: Jan 5, 2014
 *      Author: Jenn
 */

#include "all.h"
#include "MCP2515_spi.h"	//SPI initialization functions
#include "MCP2515.h"		//MCP2515 functions
#include "MCP2515_DEFS.h"

//The Default CAN frequency, index to list of standard values
//0 = 1Mbit
//1 = 500Kbit
//2 = 250Kbit
//3 = 125Kbit
//4 = 62.5Kbit
#define CANFREQ 0

const unsigned int MaskConfig[32] = {0x0000,0x0000,0x0000,0x0000,
									 0x0000,0x0000,0x0000,0x0000,
									 0x0000,0x0000,0x0000,0x0000,
									 0x0000,0x0000,0x0000,0x0000,
									 0x0000,0x0000,0x0000,0x0000,
									 0x0000,0x0000,0x0000,0x0000,
									 0x0000,0x0000,0x0000,0x0000,
									 0x0000,0x0000,0x0000,0x0000};

void MCP2515_Total_Reset(int delay);								//does a total reset of the MCP2515 system. stays off bus for delay before coming back on line.


stopwatch_struct* mirror_Ato2_watch;
stopwatch_struct* mirror_2toA_watch0;
stopwatch_struct* mirror_2toA_watch1;
stopwatch_struct* mirror_2toA_watch2;
stopwatch_struct* canA_watch;
stopwatch_struct* can2_watch;


ops_struct ops_temp;
data_struct data_temp;

//buffers
buffer_struct Buf_Ato2;
buffer_struct Buf_2toA;

int MCP_TXn_Ready = 0;


void SensorCov()
{
	SensorCovInit();
	while (ops.State == STATE_SENSOR_COV)
	{
		LatchStruct();
		SensorCovMeasure();
	//	UpdateStruct();
	//	FillCANData();
	}
	SensorCovDeInit();
}

void SensorCovInit()
{

	/**************** config ISR for MCP2515 */

	// MCP2515 !int is on GPIO20 is XINT1
	EALLOW;
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 20;   // XINT1 is GPIO20
	//GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 22;   // XINT1 is GPIO22
	//GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 32;   // XINT1 is GPIO20
	EDIS;

	// Configure XINT1
	XIntruptRegs.XINT1CR.bit.POLARITY = 0;      // Falling edge interrupt

	// Enable XINT1
	XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1


	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
	IFR &= ~M_INT1;
	IER |= M_INT1;
	/********************/


	//CONFIG GP_BUTTON
	ConfigGPButton();
	ConfigLED0();
	ConfigLED1();
	SETLED0();
	SETLED1();

	MCP2515_Total_Reset(0);

	mirror_Ato2_watch = StartStopWatch(500);		//start stopwatch for Ato2 timeout
	mirror_2toA_watch0 = StartStopWatch(500);    	//start stopwatch for 2toA timeout
	mirror_2toA_watch1 = StartStopWatch(500);    	//start stopwatch for 2toA timeout
	mirror_2toA_watch2 = StartStopWatch(500);    	//start stopwatch for 2toA timeout
	canA_watch = StartStopWatch(50000);				//start stopwatch for canA bus status
	can2_watch = StartStopWatch(50000);				//start stopwatch for can2 bus status
}


void LatchStruct()
{
	memcpy(&ops_temp, &ops, sizeof(struct OPERATIONS));
	memcpy(&data_temp, &data, sizeof(struct DATA));
	ops.Change.all = 0;	//clear change states
}

void SensorCovMeasure()
{
	int tmp;
	struct ECAN_REGS ECanaShadow;	//shadow structure for modifying CAN registers
	unsigned int tmp_buffer[13];	//temporary buffer for messages

//*********************************************************************MCP2515 service*******************************************************************************************

	/*
	//send messages from 2 to A
	if (MCP_TXn_Ready != 0)																//check that at least one of the transmit buffers is ready for sending
		if (Buf_2toA.empty == 0)														//check that we have a message to send
		{
			if (MCP_TXn_Ready & 0x01)													//try to send using TXB0
			{
				ops.can2toA.fields.Buffer_level = Buffer_MCPFillMessage(&Buf_2toA, 0);	//add message from buffer
				MCP_TXn_Ready &= 0x06;													//clear TXB0 ready flag
				MCP2515Write(MCP_TXB0CTRL,0x09);										//flag message for transmission
				StopWatchRestart(mirror_2toA_watch0);									//start timeout counter
			}
			else if (MCP_TXn_Ready & 0x02)												//try to send using TXB1
			{
				ops.can2toA.fields.Buffer_level = Buffer_MCPFillMessage(&Buf_2toA, 1);	//add message from buffer
				MCP_TXn_Ready &= 0x05;													//clear TXB0 ready flag
				MCP2515Write(MCP_TXB1CTRL,0x0A);										//flag message for transmission
				StopWatchRestart(mirror_2toA_watch1);									//start timeout counter
			}
			else if (MCP_TXn_Ready & 0x04)												//try to send using TXB2
			{
				ops.can2toA.fields.Buffer_level = Buffer_MCPFillMessage(&Buf_2toA, 2);	//add message from buffer
				MCP_TXn_Ready &= 0x03;													//clear TXB0 ready flag
				MCP2515Write(MCP_TXB2CTRL,0x0B);										//flag message for transmission
				StopWatchRestart(mirror_2toA_watch2);									//start timeout counter
			}
			else
				tmp=0;	//serious problem if it ever gets here
		}

	//watch for send timeouts
	if ((MCP_TXn_Ready & 0x01) == 0)														//check if TXB0 has a pending message
	{
		if(isStopWatchComplete(mirror_2toA_watch0))
		{
			ops.can2toA.fields.Write_Timeouts += 1;
			MCP2515Write(MCP_TXB0CTRL, 0x01);													//clear send request
			MCP_TXn_Ready |= 0x01;															//flag transmitter as ready
		}
	}

	if ((MCP_TXn_Ready & 0x02) == 0)														//check if TXB1 has a pending message
	{
		if(isStopWatchComplete(mirror_2toA_watch1))
		{
			ops.can2toA.fields.Write_Timeouts += 1;
			MCP2515Write(MCP_TXB1CTRL, 0x02);													//clear send request
			MCP_TXn_Ready |= 0x02;															//flag transmitter as ready
		}
	}
	if ((MCP_TXn_Ready & 0x04) == 0)														//check if TXB1 has a pending message
	{
		if(isStopWatchComplete(mirror_2toA_watch2))
		{
			ops.can2toA.fields.Write_Timeouts += 1;
			MCP2515Write(MCP_TXB2CTRL, 0x03);													//clear send request
			MCP_TXn_Ready |= 0x04;															//flag transmitter as ready
		}
	}


	//error checking
	if (ops.canAto2.fields.flags & 0x10)				//check for off-bus condition
		MCP2515_Total_Reset(500);						//reset the MCP2515 after 500 ms

	*/
//*****************************************************************end MCP2515 service********************************************************************************************

//***********************************************This code takes care of sending messages out on CAN bus 2************************************************************************
	ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;			//get CAN transmit status register
	if ((ECanaShadow.CANTRS.all & (1 << 0x04)) == 0)		//check if mailbox 4 is not sending
	{
		tmp = Buffer_Read(&Buf_Ato2, tmp_buffer);			//get can message
		if(tmp != -1)										//check if there are messages to send
		{
			StopWatchRestart(mirror_Ato2_watch);					//restart stopwatch for timeout

			//set up the actual CAN transmission
			EALLOW;
				//set up mailbox(4)
				//some values are dependent on message, ie the DLC and the message ID
				ECanaShadow.CANME.all = ECanaRegs.CANME.all;	//get current CAN registers
				ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;	//get current CAN registers

				ECanaShadow.CANME.bit.ME4 = 0;
				ECanaRegs.CANME.all = ECanaShadow.CANME.all;	//disable mailbox so we can change the ID

				//message ID is: bit 31 = EXID, bit 30,29 = acceptance bits, 0 for transmit, 28:0 ID, ID is left aligned
				ECanaMboxes.MBOX4.MSGID.all = 0;
				if (tmp_buffer[1] & 0x0008)
				{//extended ID
					ECanaMboxes.MBOX4.MSGID.all = (0x80000000L | (tmp_buffer[1] & 0x00000003L) << 27 | (tmp_buffer[2] & 0x000000FFL) << 19 | (tmp_buffer[3] & 0x000000FFL) << 10 | (tmp_buffer[0] & 0x000000FFL) << 3 | (tmp_buffer[1] & 0x000000E0L) >> 5);
				}

				else
				{//standard ID
					ECanaMboxes.MBOX4.MSGID.bit.STDMSGID = (0x00000000L | (tmp_buffer[0] & 0x000000FFL) << 3 | (tmp_buffer[1] & 0x000000E0L) >> 5);
				}
				if(ECanaMboxes.MBOX4.MSGID.bit.STDMSGID == 358)
				{
					ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = tmp_buffer[4] & 0x000F;			//DLC
				}
				ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = tmp_buffer[4] & 0x000F;			//DLC
				ECanaShadow.CANMD.bit.MD4 = 0; 			//transmit
				ECanaShadow.CANME.bit.ME4 = 1;			//enable

				ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;
				ECanaRegs.CANME.all = ECanaShadow.CANME.all;

				//fill mailbox with data from buf
				ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;	//get regs
				ECanaShadow.CANMC.bit.MBNR = 0x04;				//request write to mailbox 4
				ECanaShadow.CANMC.bit.CDR = 1;					//change data field request
				ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;	//set regs

				//set data
				ECanaMboxes.MBOX4.MDL.all = (tmp_buffer[5] & 0x000000FFL) | (tmp_buffer[6] & 0x000000FFL) << 8 | (tmp_buffer[7] & 0x000000FFL) << 16 | (tmp_buffer[8] & 0x000000FFL) << 24;					//fill data
				ECanaMboxes.MBOX4.MDH.all = (tmp_buffer[9] & 0x000000FFL) | (tmp_buffer[10] & 0x000000FFL) << 8 | (tmp_buffer[11] & 0x000000FFL) << 16 | (tmp_buffer[12] & 0x000000FFL) << 24;

				ECanaShadow.CANMC.bit.MBNR = 0;					//clear request for mailbox 4
				ECanaShadow.CANMC.bit.CDR = 0;					//clear change data field request
				ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;	//set regs again

			EDIS;

			//request that the mailbox be sent
			ECanaShadow.CANTRS.all = 1 << 0x04;				//mark mailbox 4 for transmit
			ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;	//set in real registers
		}
	}
	else	//mailbox 4 is sending
	{
		if(isStopWatchComplete(mirror_Ato2_watch) == 1)
		{
			//error on CAN send timeout
			//increment timeout counter and clear mailbox 4
			ops.canAto2.fields.Write_Timeouts += 1;

			ECanaShadow.CANTRR.all = ECanaRegs.CANTRR.all;
			ECanaShadow.CANTRR.bit.TRR4 = 1;
			ECanaRegs.CANTRR.all = ECanaShadow.CANTRR.all;

		    ECanaMboxes.MBOX4.MDH.all = 0;
		    ECanaMboxes.MBOX4.MDL.all = 0;
		}
	}


//****************************check for can busses down**************************
    if (isStopWatchComplete(canA_watch) == 1)
	{
		//indicate CAN bus A down
    	ops.canAto2.fields.can_error = 1;
    	SETLED0();
	}
    else
    {
    	//indicate CAN bus A up
    	ops.canAto2.fields.can_error = 0;
    	CLEARLED0();
    }

    if (isStopWatchComplete(can2_watch) == 1)
    {
    	//indicate CAN bus 2 down
       	ops.can2toA.fields.can_error = 1;
       	SETLED1();
    }
    else
    {
    	//indicate CAN bus 2 up
    	ops.can2toA.fields.can_error = 0;
    	CLEARLED0();
    }

}

void SensorCovDeInit()
{
	MCP2515_reset(1);					//hold MCP2515 in reset
	StopStopWatch(mirror_Ato2_watch);
	StopStopWatch(mirror_2toA_watch0);
	StopStopWatch(mirror_2toA_watch1);
	StopStopWatch(mirror_2toA_watch2);
	StopStopWatch(canA_watch);
	StopStopWatch(can2_watch);
	SETLED0();
	SETLED1();
}

void MCP2515_Total_Reset(int delay)
{
	int i,j;
	if (delay > 0)
		for(j=0;j<delay;j++)
			for (i=0;i<10;i++)
				DELAY_US(100);

	MCP2515_spi_init();								//initialize SPI port and GPIO associated with the MCP2515
	PgmInitMCP2515(((1<<CANFREQ)-1), MaskConfig);	//initialize MCP2515

	Buffer_Clear(&Buf_Ato2);
	ops.canAto2.all = 0;

	Buffer_Clear(&Buf_2toA);
	ops.can2toA.all = 0;

	MCP_TXn_Ready = 0x07;
}
