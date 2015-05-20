/*
 * can.h
 *
 *  Created on: Nov 13, 2013
 *      Author: Nathan
 */

#ifndef CAN_H_
#define CAN_H_




struct CANmsg {
   char MBox;
   union CANMSGCTRL_REG   MSGCTRL;
   union CANMDL_REG       MDL;
   union CANMDH_REG       MDH;
};

struct TRS_REG {
	union CANTRS_REG	TRS;
};

void CANSetup();
char FillCAN(unsigned int Mbox);
void SendCAN(unsigned int Mbox);
void FillCANData();
void FillSendCAN(unsigned int Mbox);
void ClearMailBoxes();

//todo USER: DEFINE IDs and mailboxes for output
#define COMMAND_ID 		0x32
#define COMMAND_BOX 	0

#define HEARTBEAT_ID 	0x31
#define HEARTBEAT_BOX 	1

#define ADC_ID 			0x102
#define ADC_BOX 		2
#define ADC_TRS			TRS2

#define GP_BUTTON_ID 	0x103
#define GP_BUTTON_BOX 	3
#define GP_BUTTON_TRS	TRS3

//messages to mirror to CAN A

#define CELL_TEMP1_ID 	0x32C
#define CELL_TEMP1_BOX 	5

#define CELL_TEMP2_ID 	0x32D
#define CELL_TEMP2_BOX 	6

#define CELL_TEMP3_ID 	0x32E
#define CELL_TEMP3_BOX 	7

#define CELL_TEMP4_ID 	0x32F
#define CELL_TEMP4_BOX 	8

#define CELL_TEMP5_ID 	0x330
#define CELL_TEMP5_BOX 	9

#define CELL_TEMP6_ID 	0x331
#define CELL_TEMP6_BOX 	10

#define CELL_TEMP7_ID 	0x332
#define CELL_TEMP7_BOX 	11

#define CELL_TEMP8_ID 	0x333
#define CELL_TEMP8_BOX 	12

#define CELL_TEMP9_ID 	0x334
#define CELL_TEMP9_BOX 	13

#define CELL_TEMP10_ID 	0x335
#define CELL_TEMP10_BOX 14

#define CELL_TEMP11_ID 	0x336
#define CELL_TEMP11_BOX 15

#define CELL_TEMP12_ID 	0x337
#define CELL_TEMP12_BOX	16

#define CELL_TEMP13_ID 	0x338
#define CELL_TEMP13_BOX 17

#define CELL_TEMP14_ID 	0x339
#define CELL_TEMP14_BOX 18

#define CELL_TEMP15_ID 	0x33A
#define CELL_TEMP15_BOX 19

#define CELL_TEMP16_ID 	0x33B
#define CELL_TEMP16_BOX 20

#define CELL_TEMP17_ID 	0x33C
#define CELL_TEMP17_BOX 21

#define CELL_TEMP18_ID 	0x33D
#define CELL_TEMP18_BOX 22

#define CELL_TEMP19_ID 	0x33E
#define CELL_TEMP19_BOX 23

#define CELL_TEMP20_ID 	0x33F
#define CELL_TEMP20_BOX 24

#define BIM_STAT1_ID 	0x340
#define BIM_STAT1_BOX 	25

#define BIM_STAT2_ID 	0x341
#define BIM_STAT2_BOX 	26

#define BIM_STAT3_ID 	0x342
#define BIM_STAT3_BOX 	27

#define BIM_STAT4_ID 	0x343
#define BIM_STAT4_BOX 	28

#define BIM_STAT5_ID 	0x344
#define BIM_STAT5_BOX 	29


#endif /* CAN_H_ */
