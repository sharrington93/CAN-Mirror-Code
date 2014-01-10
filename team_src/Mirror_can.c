/*
 * Mirror_can.c
 *
 *  Created on: Jan 6, 2014
 *      Author: jennifer
 */

#include "all.h"

int SendGeneralCANMessage(unsigned int timeout, unsigned int* buf);	//sends a CAN message immediately, with timeout

//The format for the data in buf follows the format of the MCP2515, in all cases bits 15:8 of the uint16 are ignored
//buf[0] = SIDH, bits 7:0 are Standard identifier bits 10:3.
//buf[1] = SIDL, bits 7:5 are Standard identifier bits 2:0. bit 3 is EXIDE, bits 1:0 are EID 17:16
//buf[2] = EID8, bits 7:0 are Extended identifier bits 15:8
//buf[3] = EID0, bits 7:0 are Extended identifier bits 7:0
//buf[4] = DLC.	bits 3:0 are the 4-bit data length code, all other bits are ignored
//buf[5..12] = data, each data byte is the 7:0 bits of buf[n]
//timeout uses the stopwatch function to abort the transmission if it doesn't succeed in a specified time
int SendGeneralCANMessage(unsigned int timeout, unsigned int* buf)
{
	struct ECAN_REGS ECanaShadow;	//shadow structure for modifying CAN registers
	stopwatch_struct* mirror_can_watch;
	int ret;

	EALLOW;
	//set up mailbox(4)
	//some values are dependent on buf, ie the DLC and the message ID
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;	//get current CAN registers
	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;	//get current CAN registers

	ECanaShadow.CANME.bit.ME4 = 0;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;	//disable mailbox so we can change the ID

	//message ID is bit 31 = EXID, bit 30,29 = acceptance bits, 0 for transmit, 28:0 ID, ID is left aligned
	if (buf[1] & 0x0008)
	{//extended ID
		ECanaMboxes.MBOX4.MSGID.all = (0x80000000 | (buf[1] & 0x00000003) << 27 | (buf[2] & 0x000000FF) << 19 | (buf[3] & 0x000000FF) << 10 | (buf[0] & 0x000000FF) << 3 | (buf[1] & 0x000000E0) >> 5);
	}
	else
	{//standard ID
		ECanaMboxes.MBOX4.MSGID.all = (0x00000000 | (buf[0] & 0x000000FF) << 3 | (buf[1] & 0x000000E0) >> 5);
	}

	ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = buf[4] & 0x000F;			//DLC
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
	ECanaMboxes.MBOX4.MDL.all = (buf[5] & 0x000000FF) | (buf[6] & 0x000000FF) << 8 | (buf[7] & 0x000000FF) << 16 | (buf[8] & 0x000000FF) << 24;					//fill data
	ECanaMboxes.MBOX4.MDH.all = (buf[9] & 0x000000FF) | (buf[10] & 0x000000FF) << 8 | (buf[11] & 0x000000FF) << 16 | (buf[12] & 0x000000FF) << 24;

	ECanaShadow.CANMC.bit.MBNR = 0;					//clear request for mailbox 4
	ECanaShadow.CANMC.bit.CDR = 0;					//clear change data field request
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;	//set regs again

	EDIS;

	//request that the mailbox be sent
	ECanaShadow.CANTRS.all = 1 << 0x04;				//mark mailbox 4 for transmit
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;	//set in real registers

	mirror_can_watch = StartStopWatch(timeout);		//start stopwatch for timeout

	do { ECanaShadow.CANTA.all = ECanaRegs.CANTA.all; } while(((ECanaShadow.CANTA.all & (1 << 0x04)) != (1 << 0x04)) && (isStopWatchComplete(mirror_can_watch) == 0)); //wait to send or hit stop watch

		ECanaShadow.CANTA.all = 1 << 0x04;
		ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;						//clear flag

		ret = isStopWatchComplete(mirror_can_watch);
		StopStopWatch(mirror_can_watch);
		return ret;
}


