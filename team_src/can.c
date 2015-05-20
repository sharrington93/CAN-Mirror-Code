/*
 * can.c
 *
 *  Created on: Nov 12, 2013
 *      Author: Nathan
 */
#include "all.h"

/*
//variables for the 2->A CAN message queue
extern int CANQueueIN2;
extern int CANQueueOUT2;
extern int CANQueueFULL2;
extern int CANQueueEMPTY2;
extern unsigned int CANQueue_raw2[CANQUEUEDEPTH][13];
*/

unsigned int mask;
stopwatch_struct* can_watch;
struct ECAN_REGS ECanaShadow;

void CANSetup()
{

	InitECanaGpio();
	InitECana();

	ClearMailBoxes();

	ECanaShadow.CANMIM.all = 0;
	ECanaShadow.CANMIL.all = 0;
	ECanaShadow.CANGIM.all = 0;
	ECanaShadow.CANGAM.bit.AMI = 0; //must be standard
	ECanaShadow.CANGIM.bit.I1EN = 1;  // enable I1EN
	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;

	//todo USER: Node specifc CAN setup
	EALLOW;

	// create mailbox for all Receive and transmit IDs
	// MBOX0 - MBOX31

	//Command RECEIVE
	ECanaMboxes.MBOX0.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX0.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX0.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = COMMAND_ID;
	ECanaShadow.CANMD.bit.MD0 = 1;			//receive
	ECanaShadow.CANME.bit.ME0 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM0  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL0  = 1;  		// Int.-Level MB#0  -> I1EN

	//Heart TRANSMIT
	ECanaMboxes.MBOX1.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX1.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX1.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = HEARTBEAT_ID;
	ECanaShadow.CANMD.bit.MD1 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME1 = 1;			//enable


	//SOMETHING ODD ABOUT ORDER HERE AND RTR BIT...

	//adc TRANSMIT
	ECanaMboxes.MBOX2.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX2.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX2.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX2.MSGID.bit.STDMSGID = ADC_ID;
	ECanaShadow.CANMD.bit.MD2 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME2 = 1;			//enable

	//gp_button TRANSMIT
	ECanaMboxes.MBOX3.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX3.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX3.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX3.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX3.MSGID.bit.STDMSGID = GP_BUTTON_ID;
	ECanaShadow.CANMD.bit.MD3 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME3 = 1;			//enable

	//Mailbox 4 used by CAN mirror to send A traffic on 2
/*
	//Cell Temp 1 receive
	ECanaMboxes.MBOX5.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX5.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX5.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX5.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX5.MSGID.bit.STDMSGID = CELL_TEMP1_ID;
	ECanaShadow.CANMD.bit.MD5 = 1;			//receive
	ECanaShadow.CANME.bit.ME5 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM5  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL5  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 2 receive
	ECanaMboxes.MBOX6.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX6.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX6.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX6.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX6.MSGID.bit.STDMSGID = CELL_TEMP2_ID;
	ECanaShadow.CANMD.bit.MD6 = 1;			//receive
	ECanaShadow.CANME.bit.ME6 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM6  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL6  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 3 receive
	ECanaMboxes.MBOX7.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX7.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX7.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX7.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX7.MSGID.bit.STDMSGID = CELL_TEMP3_ID;
	ECanaShadow.CANMD.bit.MD7 = 1;			//receive
	ECanaShadow.CANME.bit.ME7 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM7  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL7  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 4 receive
	ECanaMboxes.MBOX8.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX8.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX8.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX8.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX8.MSGID.bit.STDMSGID = CELL_TEMP4_ID;
	ECanaShadow.CANMD.bit.MD8 = 1;			//receive
	ECanaShadow.CANME.bit.ME8 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM8  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL8  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 5 receive
	ECanaMboxes.MBOX9.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX9.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX9.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX9.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX9.MSGID.bit.STDMSGID = CELL_TEMP5_ID;
	ECanaShadow.CANMD.bit.MD9 = 1;			//receive
	ECanaShadow.CANME.bit.ME9 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM9  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL9  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 6 receive
	ECanaMboxes.MBOX10.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX10.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX10.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX10.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX10.MSGID.bit.STDMSGID = CELL_TEMP6_ID;
	ECanaShadow.CANMD.bit.MD10 = 1;			//receive
	ECanaShadow.CANME.bit.ME10 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM10  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL10  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 7 receive
	ECanaMboxes.MBOX11.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX11.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX11.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX11.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX11.MSGID.bit.STDMSGID = CELL_TEMP7_ID;
	ECanaShadow.CANMD.bit.MD11 = 1;			//receive
	ECanaShadow.CANME.bit.ME11 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM11  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL11  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 8 receive
	ECanaMboxes.MBOX12.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX12.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX12.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX12.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX12.MSGID.bit.STDMSGID = CELL_TEMP8_ID;
	ECanaShadow.CANMD.bit.MD12 = 1;			//receive
	ECanaShadow.CANME.bit.ME12 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM12  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL12  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 9 receive
	ECanaMboxes.MBOX13.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX13.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX13.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX13.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX13.MSGID.bit.STDMSGID = CELL_TEMP9_ID;
	ECanaShadow.CANMD.bit.MD13 = 1;			//receive
	ECanaShadow.CANME.bit.ME13 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM13  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL13  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 10 receive
	ECanaMboxes.MBOX14.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX14.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX14.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX14.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX14.MSGID.bit.STDMSGID = CELL_TEMP10_ID;
	ECanaShadow.CANMD.bit.MD14 = 1;			//receive
	ECanaShadow.CANME.bit.ME14 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM14  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL14  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 11 receive
	ECanaMboxes.MBOX15.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX15.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX15.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX15.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX15.MSGID.bit.STDMSGID = CELL_TEMP11_ID;
	ECanaShadow.CANMD.bit.MD15 = 1;			//receive
	ECanaShadow.CANME.bit.ME15 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM15  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL15  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 12 receive
	ECanaMboxes.MBOX16.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX16.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX16.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX16.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX16.MSGID.bit.STDMSGID = CELL_TEMP12_ID;
	ECanaShadow.CANMD.bit.MD16 = 1;			//receive
	ECanaShadow.CANME.bit.ME16 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM16  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL16  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 13 receive
	ECanaMboxes.MBOX17.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX17.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX17.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX17.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX17.MSGID.bit.STDMSGID = CELL_TEMP13_ID;
	ECanaShadow.CANMD.bit.MD17 = 1;			//receive
	ECanaShadow.CANME.bit.ME17 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM17  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL17  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 14 receive
	ECanaMboxes.MBOX18.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX18.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX18.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX18.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX18.MSGID.bit.STDMSGID = CELL_TEMP14_ID;
	ECanaShadow.CANMD.bit.MD18 = 1;			//receive
	ECanaShadow.CANME.bit.ME18 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM18  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL18  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 15 receive
	ECanaMboxes.MBOX19.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX19.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX19.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX19.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX19.MSGID.bit.STDMSGID = CELL_TEMP15_ID;
	ECanaShadow.CANMD.bit.MD19 = 1;			//receive
	ECanaShadow.CANME.bit.ME19 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM19  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL19  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 16 receive
	ECanaMboxes.MBOX20.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX20.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX20.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX20.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX20.MSGID.bit.STDMSGID = CELL_TEMP16_ID;
	ECanaShadow.CANMD.bit.MD20 = 1;			//receive
	ECanaShadow.CANME.bit.ME20 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM20  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL20  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 17 receive
	ECanaMboxes.MBOX21.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX21.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX21.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX21.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX21.MSGID.bit.STDMSGID = CELL_TEMP17_ID;
	ECanaShadow.CANMD.bit.MD21 = 1;			//receive
	ECanaShadow.CANME.bit.ME21 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM21  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL21  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 18 receive
	ECanaMboxes.MBOX22.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX22.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX22.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX22.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX22.MSGID.bit.STDMSGID = CELL_TEMP18_ID;
	ECanaShadow.CANMD.bit.MD22 = 1;			//receive
	ECanaShadow.CANME.bit.ME22 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM22  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL22  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 19 receive
	ECanaMboxes.MBOX23.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX23.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX23.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX23.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX23.MSGID.bit.STDMSGID = CELL_TEMP19_ID;
	ECanaShadow.CANMD.bit.MD23 = 1;			//receive
	ECanaShadow.CANME.bit.ME23 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM23 = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL23  = 1;  		// Int.-Level MB#0  -> I1EN

	//Cell Temp 20 receive
	ECanaMboxes.MBOX24.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX24.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX24.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX24.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX24.MSGID.bit.STDMSGID = CELL_TEMP20_ID;
	ECanaShadow.CANMD.bit.MD24 = 1;			//receive
	ECanaShadow.CANME.bit.ME24 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM24  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL24  = 1;  		// Int.-Level MB#0  -> I1EN

	//BIM Stat 1 receive
	ECanaMboxes.MBOX25.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX25.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX25.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX25.MSGID.bit.STDMSGID = BIM_STAT1_ID;
	ECanaShadow.CANMD.bit.MD25 = 1;			//receive
	ECanaShadow.CANME.bit.ME25 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM25  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL25  = 1;  		// Int.-Level MB#0  -> I1EN

	//BIM Stat 2 receive
	ECanaMboxes.MBOX26.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX26.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX26.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX26.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX26.MSGID.bit.STDMSGID = BIM_STAT2_ID;
	ECanaShadow.CANMD.bit.MD26 = 1;			//receive
	ECanaShadow.CANME.bit.ME26 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM26  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL26  = 1;  		// Int.-Level MB#0  -> I1EN

	//BIM Stat 3 receive
	ECanaMboxes.MBOX27.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX27.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX27.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX27.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX27.MSGID.bit.STDMSGID = BIM_STAT3_ID;
	ECanaShadow.CANMD.bit.MD27 = 1;			//receive
	ECanaShadow.CANME.bit.ME27 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM27  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL27  = 1;  		// Int.-Level MB#0  -> I1EN

	//BIM Stat 4 receive
	ECanaMboxes.MBOX28.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX28.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX28.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX28.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX28.MSGID.bit.STDMSGID = BIM_STAT4_ID;
	ECanaShadow.CANMD.bit.MD28 = 1;			//receive
	ECanaShadow.CANME.bit.ME28 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM28  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL25  = 1;  		// Int.-Level MB#0  -> I1EN

	//BIM Stat 5 receive
	ECanaMboxes.MBOX29.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX29.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX29.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX29.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX29.MSGID.bit.STDMSGID = BIM_STAT5_ID;
	ECanaShadow.CANMD.bit.MD29 = 1;			//receive
	ECanaShadow.CANME.bit.ME29 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM29  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL29  = 1;  		// Int.-Level MB#0  -> I1EN
*/


	ECanaRegs.CANGAM.all = ECanaShadow.CANGAM.all;
	ECanaRegs.CANGIM.all = ECanaShadow.CANGIM.all;
	ECanaRegs.CANMIM.all = ECanaShadow.CANMIM.all;
	ECanaRegs.CANMIL.all = ECanaShadow.CANMIL.all;
	ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.STM = 0;    // No self-test mode
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
    EDIS;

    //ENABLE PIE INTERRUPTS
    IER |= M_INT9;
    PieCtrlRegs.PIEIER9.bit.INTx6= 1;

    can_watch = StartStopWatch(SENDCAN_STOPWATCH);
}

void ClearMailBoxes()
{
    ECanaMboxes.MBOX0.MDH.all = 0;
    ECanaMboxes.MBOX0.MDL.all = 0;
    ECanaMboxes.MBOX1.MDH.all = 0;
    ECanaMboxes.MBOX1.MDL.all = 0;
    ECanaMboxes.MBOX2.MDH.all = 0;
    ECanaMboxes.MBOX2.MDL.all = 0;
    ECanaMboxes.MBOX3.MDH.all = 0;
    ECanaMboxes.MBOX3.MDL.all = 0;
    ECanaMboxes.MBOX4.MDH.all = 0;
    ECanaMboxes.MBOX4.MDL.all = 0;
    ECanaMboxes.MBOX5.MDH.all = 0;
    ECanaMboxes.MBOX5.MDL.all = 0;
    ECanaMboxes.MBOX6.MDH.all = 0;
    ECanaMboxes.MBOX6.MDL.all = 0;
    ECanaMboxes.MBOX7.MDH.all = 0;
    ECanaMboxes.MBOX7.MDL.all = 0;
    ECanaMboxes.MBOX8.MDH.all = 0;
    ECanaMboxes.MBOX8.MDL.all = 0;
    ECanaMboxes.MBOX9.MDH.all = 0;
    ECanaMboxes.MBOX9.MDL.all = 0;
    ECanaMboxes.MBOX10.MDH.all = 0;
    ECanaMboxes.MBOX10.MDL.all = 0;
    ECanaMboxes.MBOX11.MDH.all = 0;
    ECanaMboxes.MBOX11.MDL.all = 0;
    ECanaMboxes.MBOX12.MDH.all = 0;
    ECanaMboxes.MBOX12.MDL.all = 0;
    ECanaMboxes.MBOX13.MDH.all = 0;
    ECanaMboxes.MBOX13.MDL.all = 0;
    ECanaMboxes.MBOX14.MDH.all = 0;
    ECanaMboxes.MBOX14.MDL.all = 0;
    ECanaMboxes.MBOX15.MDH.all = 0;
    ECanaMboxes.MBOX15.MDL.all = 0;
    ECanaMboxes.MBOX16.MDH.all = 0;
    ECanaMboxes.MBOX16.MDL.all = 0;
    ECanaMboxes.MBOX17.MDH.all = 0;
    ECanaMboxes.MBOX17.MDL.all = 0;
    ECanaMboxes.MBOX18.MDH.all = 0;
    ECanaMboxes.MBOX18.MDL.all = 0;
    ECanaMboxes.MBOX19.MDH.all = 0;
    ECanaMboxes.MBOX19.MDL.all = 0;
    ECanaMboxes.MBOX20.MDH.all = 0;
    ECanaMboxes.MBOX20.MDL.all = 0;
    ECanaMboxes.MBOX21.MDH.all = 0;
    ECanaMboxes.MBOX21.MDL.all = 0;
    ECanaMboxes.MBOX22.MDH.all = 0;
    ECanaMboxes.MBOX22.MDL.all = 0;
    ECanaMboxes.MBOX23.MDH.all = 0;
    ECanaMboxes.MBOX23.MDL.all = 0;
    ECanaMboxes.MBOX24.MDH.all = 0;
    ECanaMboxes.MBOX24.MDL.all = 0;
    ECanaMboxes.MBOX25.MDH.all = 0;
    ECanaMboxes.MBOX25.MDL.all = 0;
    ECanaMboxes.MBOX26.MDH.all = 0;
    ECanaMboxes.MBOX26.MDL.all = 0;
    ECanaMboxes.MBOX27.MDH.all = 0;
    ECanaMboxes.MBOX27.MDL.all = 0;
    ECanaMboxes.MBOX28.MDH.all = 0;
    ECanaMboxes.MBOX28.MDL.all = 0;
    ECanaMboxes.MBOX29.MDH.all = 0;
    ECanaMboxes.MBOX30.MDL.all = 0;
    ECanaMboxes.MBOX30.MDH.all = 0;
    ECanaMboxes.MBOX31.MDL.all = 0;
    ECanaMboxes.MBOX31.MDH.all = 0;
}

char FillCAN(unsigned int Mbox)
{
	//todo USER: setup for all transmit MBOXs
	struct ECAN_REGS ECanaShadow;
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	switch (Mbox)								//choose mailbox
	{
	case HEARTBEAT_BOX:
		//todo Nathan define heartbeat
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX1.MDH.all = 0;
		ECanaMboxes.MBOX1.MDL.all = 0;
		ECanaMboxes.MBOX1.MDL.word.LOW_WORD = ops.Flags.all;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case ADC_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX2.MDH.all = 0;
		ECanaMboxes.MBOX2.MDL.all = 0;
		ECanaMboxes.MBOX2.MDL.all = data.adc;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case GP_BUTTON_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX3.MDH.all = 0;
		ECanaMboxes.MBOX3.MDL.all = 0;
		ECanaMboxes.MBOX3.MDL.all = data.gp_button;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	}
	return 0;
}

void FillSendCAN(unsigned Mbox)
{
	if (FillCAN(Mbox) == 1)
	{
		SendCAN(Mbox);
	}
}

void SendCAN(unsigned int Mbox)
{
	mask = 1 << Mbox;
	ECanaRegs.CANTRS.all = mask;

	//todo Nathan: calibrate sendcan stopwatch
	StopWatchRestart(can_watch);

	do{ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;}
	while(((ECanaShadow.CANTA.all & mask) != mask) && (isStopWatchComplete(can_watch) == 0)); //wait to send or hit stop watch

	ECanaRegs.CANTA.all = mask;									//clear flag
	if (isStopWatchComplete(can_watch) == 1)					//if stopwatch flag
	{
		ops.Flags.fields.can_error = 1;
	}
	else if (ops.Flags.fields.can_error == 1)		//if no stopwatch and flagged reset
	{
		ops.Flags.fields.can_error = 0;
	}
}


void FillCANData()
{
	//todo USER: use FillCAN to put data into correct mailboxes
	FillCAN(ADC_BOX);
	FillCAN(GP_BUTTON_BOX);
}

// INT9.6
__interrupt void ECAN1INTA_ISR(void)  // eCAN-A
{
	Uint32 ops_id;
	Uint32 dummy;
/*
	Uint32 canSID, canHighBytes, canLowBytes;
 */
  	unsigned int mailbox_nr;
  	ECanaShadow.CANGIF1.bit.MIV1 =  ECanaRegs.CANGIF1.bit.MIV1;
  	mailbox_nr = ECanaShadow.CANGIF1.bit.MIV1;
  	//todo USER: Setup ops command

  	switch(mailbox_nr)
  	{
  	case COMMAND_BOX:
  		//todo Nathan: Define Command frame
  		//proposed:
  		//HIGH 4 BYTES = Uint32 ID
  		//LOW 4 BYTES = Uint32 change to
  		ops_id = ECanaMboxes.MBOX0.MDH.all;
  		dummy = ECanaMboxes.MBOX0.MDL.all;
		switch (ops_id)
		{
		case OPS_ID_STATE:
			memcpy(&ops.State,&dummy,sizeof ops.State);
			ops.Change.bit.State = 1;
			break;
		case OPS_ID_STOPWATCHERROR:
			memcpy(&ops.Flags.all,&dummy,sizeof ops.Flags.all);
			ops.Change.bit.Flags = 1;
			break;
		}
		ECanaRegs.CANRMP.bit.RMP0 = 1;
	break;
/*
	case CELL_TEMP1_BOX:
		canSID = CELL_TEMP1_ID;
		canHighBytes = ECanaMboxes.MBOX5.MDH.all;
		canLowBytes = ECanaMboxes.MBOX5.MDL.all;
		ECanaRegs.CANRMP.bit.RMP5 = 1;
		break;
	case CELL_TEMP2_BOX:
		canSID = CELL_TEMP2_ID;
		canHighBytes = ECanaMboxes.MBOX6.MDH.all;
		canLowBytes = ECanaMboxes.MBOX6.MDL.all;
		ECanaRegs.CANRMP.bit.RMP6 = 1;
		break;
	case CELL_TEMP3_BOX:
		canSID = CELL_TEMP3_ID;
		canHighBytes = ECanaMboxes.MBOX7.MDH.all;
		canLowBytes = ECanaMboxes.MBOX7.MDL.all;
		ECanaRegs.CANRMP.bit.RMP7 = 1;
		break;
	case CELL_TEMP4_BOX:
		canSID = CELL_TEMP4_ID;
		canHighBytes = ECanaMboxes.MBOX8.MDH.all;
		canLowBytes = ECanaMboxes.MBOX8.MDL.all;
		ECanaRegs.CANRMP.bit.RMP8 = 1;
		break;
	case CELL_TEMP5_BOX:
		canSID = CELL_TEMP5_ID;
		canHighBytes = ECanaMboxes.MBOX9.MDH.all;
		canLowBytes = ECanaMboxes.MBOX9.MDL.all;
		ECanaRegs.CANRMP.bit.RMP9 = 1;
		break;
	case CELL_TEMP6_BOX:
		canSID = CELL_TEMP6_ID;
		canHighBytes = ECanaMboxes.MBOX10.MDH.all;
		canLowBytes = ECanaMboxes.MBOX10.MDL.all;
		ECanaRegs.CANRMP.bit.RMP10 = 1;
		break;
	case CELL_TEMP7_BOX:
		canSID = CELL_TEMP7_ID;
		canHighBytes = ECanaMboxes.MBOX11.MDH.all;
		canLowBytes = ECanaMboxes.MBOX11.MDL.all;
		ECanaRegs.CANRMP.bit.RMP11 = 1;
		break;
	case CELL_TEMP8_BOX:
		canSID = CELL_TEMP8_ID;
		canHighBytes = ECanaMboxes.MBOX12.MDH.all;
		canLowBytes = ECanaMboxes.MBOX12.MDL.all;
		ECanaRegs.CANRMP.bit.RMP12 = 1;
		break;
	case CELL_TEMP9_BOX:
		canSID = CELL_TEMP9_ID;
		canHighBytes = ECanaMboxes.MBOX13.MDH.all;
		canLowBytes = ECanaMboxes.MBOX13.MDL.all;
		ECanaRegs.CANRMP.bit.RMP13 = 1;
		break;
	case CELL_TEMP10_BOX:
		canSID = CELL_TEMP10_ID;
		canHighBytes = ECanaMboxes.MBOX14.MDH.all;
		canLowBytes = ECanaMboxes.MBOX14.MDL.all;
		ECanaRegs.CANRMP.bit.RMP14 = 1;
		break;
	case CELL_TEMP11_BOX:
		canSID = CELL_TEMP11_ID;
		canHighBytes = ECanaMboxes.MBOX15.MDH.all;
		canLowBytes = ECanaMboxes.MBOX15.MDL.all;
		ECanaRegs.CANRMP.bit.RMP15 = 1;
		break;
	case CELL_TEMP12_BOX:
		canSID = CELL_TEMP12_ID;
		canHighBytes = ECanaMboxes.MBOX16.MDH.all;
		canLowBytes = ECanaMboxes.MBOX16.MDL.all;
		ECanaRegs.CANRMP.bit.RMP16 = 1;
		break;
	case CELL_TEMP13_BOX:
		canSID = CELL_TEMP13_ID;
		canHighBytes = ECanaMboxes.MBOX17.MDH.all;
		canLowBytes = ECanaMboxes.MBOX17.MDL.all;
		ECanaRegs.CANRMP.bit.RMP17 = 1;
		break;
	case CELL_TEMP14_BOX:
		canSID = CELL_TEMP14_ID;
		canHighBytes = ECanaMboxes.MBOX18.MDH.all;
		canLowBytes = ECanaMboxes.MBOX18.MDL.all;
		ECanaRegs.CANRMP.bit.RMP18 = 1;
		break;
	case CELL_TEMP15_BOX:
		canSID = CELL_TEMP15_ID;
		canHighBytes = ECanaMboxes.MBOX19.MDH.all;
		canLowBytes = ECanaMboxes.MBOX19.MDL.all;
		ECanaRegs.CANRMP.bit.RMP19 = 1;
		break;
	case CELL_TEMP16_BOX:
		canSID = CELL_TEMP16_ID;
		canHighBytes = ECanaMboxes.MBOX20.MDH.all;
		canLowBytes = ECanaMboxes.MBOX20.MDL.all;
		ECanaRegs.CANRMP.bit.RMP20 = 1;
		break;
	case CELL_TEMP17_BOX:
		canSID = CELL_TEMP17_ID;
		canHighBytes = ECanaMboxes.MBOX21.MDH.all;
		canLowBytes = ECanaMboxes.MBOX21.MDL.all;
		ECanaRegs.CANRMP.bit.RMP21 = 1;
		break;
	case CELL_TEMP18_BOX:
		canSID = CELL_TEMP18_ID;
		canHighBytes = ECanaMboxes.MBOX22.MDH.all;
		canLowBytes = ECanaMboxes.MBOX22.MDL.all;
		ECanaRegs.CANRMP.bit.RMP22 = 1;
		break;
	case CELL_TEMP19_BOX:
		canSID = CELL_TEMP19_ID;
		canHighBytes = ECanaMboxes.MBOX23.MDH.all;
		canLowBytes = ECanaMboxes.MBOX23.MDL.all;
		ECanaRegs.CANRMP.bit.RMP23 = 1;
		break;
	case CELL_TEMP20_BOX:
		canSID = CELL_TEMP20_ID;
		canHighBytes = ECanaMboxes.MBOX24.MDH.all;
		canLowBytes = ECanaMboxes.MBOX24.MDL.all;
		ECanaRegs.CANRMP.bit.RMP24 = 1;
		break;
	case BIM_STAT1_BOX:
		canSID = BIM_STAT1_ID;
		canHighBytes = ECanaMboxes.MBOX25.MDH.all;
		canLowBytes = ECanaMboxes.MBOX25.MDL.all;
		ECanaRegs.CANRMP.bit.RMP25 = 1;
		break;
	case BIM_STAT2_BOX:
		canSID = BIM_STAT2_ID;
		canHighBytes = ECanaMboxes.MBOX26.MDH.all;
		canLowBytes = ECanaMboxes.MBOX26.MDL.all;
		ECanaRegs.CANRMP.bit.RMP26 = 1;
		break;
	case BIM_STAT3_BOX:
		canSID = BIM_STAT3_ID;
		canHighBytes = ECanaMboxes.MBOX27.MDH.all;
		canLowBytes = ECanaMboxes.MBOX27.MDL.all;
		ECanaRegs.CANRMP.bit.RMP27 = 1;
		break;
	case BIM_STAT4_BOX:
		canSID = BIM_STAT4_ID;
		canHighBytes = ECanaMboxes.MBOX28.MDH.all;
		canLowBytes = ECanaMboxes.MBOX28.MDL.all;
		ECanaRegs.CANRMP.bit.RMP28 = 1;
		break;
	case BIM_STAT5_BOX:
		canSID = BIM_STAT5_ID;
		canHighBytes = ECanaMboxes.MBOX29.MDH.all;
		canLowBytes = ECanaMboxes.MBOX29.MDL.all;
		ECanaRegs.CANRMP.bit.RMP29 = 1;
		break;
*/
  	}
/*
  	if(mailbox_nr >= CELL_TEMP1_BOX && mailbox_nr <= BIM_STAT5_BOX)
  		if(CANQueueFULL2 == 0)
			{
				//Todo copy all CAN message data into CANQueue_raw2[CANQueueIN2][0]
				//raw message format is: raw[0] = 10:3 of SID
				//						 raw[1] = {2:0 of sid}{X,EXIDE,X}{17:16 of EID}
				//						 raw[2] = 15:8 of EID
				//						 raw[3] = 7:0 of EID
				//						 raw[4] = {X,RTR,X,X}{3:0 of DLC}
				//						 raw[5]-raw[12] = Data bytes

				//update queue

  				CANQueue_raw2[CANQueueIN2][0] = (canSID>>3) & 0x000000FFL;				//bits 10:3 shifted to bits 7:0
  				CANQueue_raw2[CANQueueIN2][1] = (canSID<<5) & 0x000000FFL;				//bits 2:0 shifted to bits 7:5
  				CANQueue_raw2[CANQueueIN2][2] = 0;										//EXID is always 0
  				CANQueue_raw2[CANQueueIN2][3] = 0;										//EXID is always 0
  				CANQueue_raw2[CANQueueIN2][4] = 0x08;									//Always have 8 data bytes

  				memcpy(&canHighBytes,&CANQueue_raw2[CANQueueIN2][9],sizeof canHighBytes);	//Copy high data bytes
  				memcpy(&canLowBytes,&CANQueue_raw2[CANQueueIN2][5],sizeof canLowBytes);	//copy low data bytes

  				if (++CANQueueIN2 == CANQUEUEDEPTH) CANQueueIN2 = 0;				//increment with wrap
				if (CANQueueIN2 == CANQueueOUT2) CANQueueFULL2 = 1;					//test for full
				CANQueueEMPTY2 = 0;													//just got a message, can't be empty
			}
			else	//overflow error, do some flagging or something
			{

			}
*/
  	//todo USER: Setup other reads

  	//To receive more interrupts from this PIE group, acknowledge this interrupt
  	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

