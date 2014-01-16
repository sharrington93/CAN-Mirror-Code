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

#define CANQUEUEDEPTH 10

stopwatch_struct* mirror_can_watch;

ops_struct ops_temp;
data_struct data_temp;
stopwatch_struct* conv_watch;

//variables for the CAN message queue
int CANQueueIN = 0;
int CANQueueOUT = 0;
int CANQueueFULL = 0;
int CANQueueEMPTY = 1;
unsigned int CANQueue_raw[CANQUEUEDEPTH][13];

void SensorCov()
{
	SensorCovInit();
	while (ops.State == STATE_SENSOR_COV)
	{
		SensorCovMeasure();
	}
	SensorCovDeInit();
}

void SensorCovInit()
{

	MCP2515_spi_init();								//initialize SPI port and GPIO associated with the MCP2515
	PgmInitMCP2515(((1<<CANFREQ)-1), MaskConfig);	//initialize MCP2515

	//CONFIG GP_BUTTON
	ConfigGPButton();

	//clear CAN Queue
	CANQueueIN = 0;
	CANQueueOUT = 0;
	CANQueueFULL = 0;
	CANQueueEMPTY = 1;

	mirror_can_watch = StartStopWatch(100);		//start stopwatch for timeout
	conv_watch = StartStopWatch(50000);
}


void LatchStruct()
{
	memcpy(&ops_temp, &ops, sizeof(struct OPERATIONS));
	memcpy(&data_temp, &data, sizeof(struct DATA));
	ops.Change.all = 0;	//clear change states
}

void SensorCovMeasure()
{
	struct ECAN_REGS ECanaShadow;	//shadow structure for modifying CAN registers

	//this code takes care of getting messages from the MCP2515
	if(GpioDataRegs.GPADAT.bit.GPIO20 == 0 ) // poll MCP2515 interrupt pin(active low)
		if(CANQueueFULL == 0)
		{
			//read CAN message from MCP2515
			SR2_SPI(MCP_READ, MCP_RXB0SIDH, 13, &CANQueue_raw[CANQueueIN][0]);	//read raw can message
			MCP2515Write(MCP_CANINTF, 0x00);									//clear interrupt
			if (++CANQueueIN == CANQUEUEDEPTH) CANQueueIN = 0;					//increment with wrap
			if (CANQueueIN == CANQueueOUT) CANQueueFULL = 1;					//test for full

			//reset Stopwatch
			StopWatchRestart(conv_watch);
			ops.Flags.fields.CANA_status = 1;									//indicate CAN bus A status
		}
		else			//CAN Queue overflow do some error stuff
		{
			//reset Stopwatch (CAN bus A is still active, even if we overflowed
			StopWatchRestart(conv_watch);
			ops.Flags.fields.CANA_status = 1;									//indicate CAN bus A status

			//clear MCP2515 receive interrupt, so we don't block future reception
			MCP2515Write(MCP_CANINTF, 0x00);									//clear interrupt

			//increment overflow counter in ops
			ops.Flags.fields.Overflow += 1;
		}

	//This code takes care of sending messages out on CAN bus 2
	ECanaShadow.CANTRS.all = ECanaRegs.CANTRS.all;			//get CAN transmit status register
	if ((ECanaShadow.CANTRS.all & (1 << 0x04)) == 0)		//check if mailbox 4 is not sending
	{
		if(CANQueueEMPTY == 0)								//check if there are messages to send
		{
			StopWatchRestart(mirror_can_watch);				//restart stopwatch for timeout

			//set up the actual CAN transmission
			EALLOW;
				//set up mailbox(4)
				//some values are dependent on message, ie the DLC and the message ID
				ECanaShadow.CANME.all = ECanaRegs.CANME.all;	//get current CAN registers
				ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;	//get current CAN registers

				ECanaShadow.CANME.bit.ME4 = 0;
				ECanaRegs.CANME.all = ECanaShadow.CANME.all;	//disable mailbox so we can change the ID

				//message ID is bit 31 = EXID, bit 30,29 = acceptance bits, 0 for transmit, 28:0 ID, ID is left aligned
				if (CANQueue_raw[CANQueueOUT][1] & 0x0008)
				{//extended ID
					ECanaMboxes.MBOX4.MSGID.all = (0x80000000L | (CANQueue_raw[CANQueueOUT][1] & 0x00000003L) << 27 | (CANQueue_raw[CANQueueOUT][2] & 0x000000FFL) << 19 | (CANQueue_raw[CANQueueOUT][3] & 0x000000FFL) << 10 | (CANQueue_raw[CANQueueOUT][0] & 0x000000FFL) << 3 | (CANQueue_raw[CANQueueOUT][1] & 0x000000E0L) >> 5);
				}
				else
				{//standard ID
					ECanaMboxes.MBOX4.MSGID.bit.STDMSGID = (0x00000000L | (CANQueue_raw[CANQueueOUT][0] & 0x000000FFL) << 3 | (CANQueue_raw[CANQueueOUT][1] & 0x000000E0L) >> 5);
				}

				ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = CANQueue_raw[CANQueueOUT][4] & 0x000F;			//DLC
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
				ECanaMboxes.MBOX4.MDL.all = (CANQueue_raw[CANQueueOUT][5] & 0x000000FFL) | (CANQueue_raw[CANQueueOUT][6] & 0x000000FFL) << 8 | (CANQueue_raw[CANQueueOUT][7] & 0x000000FFL) << 16 | (CANQueue_raw[CANQueueOUT][8] & 0x000000FFL) << 24;					//fill data
				ECanaMboxes.MBOX4.MDH.all = (CANQueue_raw[CANQueueOUT][9] & 0x000000FFL) | (CANQueue_raw[CANQueueOUT][10] & 0x000000FFL) << 8 | (CANQueue_raw[CANQueueOUT][11] & 0x000000FFL) << 16 | (CANQueue_raw[CANQueueOUT][12] & 0x000000FFL) << 24;

				ECanaShadow.CANMC.bit.MBNR = 0;					//clear request for mailbox 4
				ECanaShadow.CANMC.bit.CDR = 0;					//clear change data field request
				ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;	//set regs again

			EDIS;

			//request that the mailbox be sent
			ECanaShadow.CANTRS.all = 1 << 0x04;				//mark mailbox 4 for transmit
			ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;	//set in real registers

			if (++CANQueueOUT == CANQUEUEDEPTH) CANQueueOUT = 0;					//increment with wrap
			if (CANQueueIN == CANQueueOUT) CANQueueEMPTY = 1;						//test for empty
		}
	}
	else	//mailbox 4 is sending
	{
		if(isStopWatchComplete(mirror_can_watch) == 1)
		{
			//error on CAN send timeout
			//increment timeout counter and clear mailbox 4
			ops.Flags.fields.Timeout += 1;

			ECanaShadow.CANTRR.all = ECanaRegs.CANTRR.all;
			ECanaShadow.CANTRR.bit.TRR4 = 1;
			ECanaRegs.CANTRR.all = ECanaShadow.CANTRR.all;

		    ECanaMboxes.MBOX4.MDH.all = 0;
		    ECanaMboxes.MBOX4.MDL.all = 0;
		}
	}

    if (isStopWatchComplete(conv_watch) == 1)
	{
		//indicate CAN bus A down
    	ops.Flags.fields.CANA_status = 0;
	}
}
/*
void UpdateStruct()
{
	memcpy(&data, &data_temp, sizeof(struct DATA));

	//todo USER: UpdateStruct
	//update with node specific op changes

	//if ops is not changed outside of sensor conversion copy temp over, otherwise don't change

	//Change bit is only set by ops changes outside of SensorCov.
	if (ops.Change.bit.State == 0)
	{
		ops.State = ops_temp.State;
	}

	if (ops.Change.bit.Flags == 0)
	{
		//only cov error happens inside of conversion so all other changes are considered correct.
		//update accordingly to correct cov_errors
		ops.Flags.bit.cov_error = ops_temp.Flags.bit.cov_error;
	}
	ops.Change.all = 0;	//clear change states
}
*/

void SensorCovDeInit()
{
	//todo USER: SensorCovDeInit()
	MCP2515_reset(1);				//hold MCP2515 in reset
	StopStopWatch(conv_watch);
	CLEARLED0();
	CLEARLED1();
	StopStopWatch(mirror_can_watch);
}
