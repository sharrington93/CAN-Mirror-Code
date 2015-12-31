/*
 * main.c
 *
 *  Created on: Dec 21, 2013
 *      Author: Nathan
 */

#include "Boot.h"
#include "all.h"

Uint16 MesgID = 0x30;

int main(void)
{
	StartUp();
	BootISRSetup();
	PowerDownISRSetup();
	ops.State = STATE_INIT;
	while(1)
	{
		NextState(MesgID);
	}
}

void NextState(Uint16 MesgID)
{
	switch (ops.State)
	{
	case STATE_INIT:
		Initilize();
		break;
	case STATE_SENSOR_COV:
		SensorCov();
		break;
	case STATE_BOOT:
		Boot(MesgID);
		break;
	case STATE_POWER_DOWN:
		PowerDown();
		break;
	default:
		Initilize();
	}
}


void BootISRSetup()
{
	   EALLOW;
	   GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;         // GPIO
	   GpioCtrlRegs.GPADIR.bit.GPIO28 = 0;          // input
	   GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 2;        // XINT1 Synch to SYSCLKOUT only
	   GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0; 			//enable pull up
	   EDIS;

	// GPIO28 is XINT1, GPIO1 is XINT2
	   EALLOW;
	   GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 28;   // XINT1 is GPIO28
	   EDIS;

	// Configure XINT1
	   XIntruptRegs.XINT1CR.bit.POLARITY = 0;      // Falling edge interrupt

	// Enable XINT1
	   XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1


	   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
	   IFR &= ~M_INT1;
	   IER |= M_INT1;
}

void StartUp()
{
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,  (unsigned long)&RamfuncsLoadSize);

	InitSysCtrl();

	InitGpio();
	DINT;

	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
	// This function is found in DSP2803x_PieVect.c.
	InitPieVectTable();

	//Initialize Flash
	//InitFlash();

	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
}

void Restart()
{
	EALLOW;
	SysCtrlRegs.WDCR = 0x0028;               // Enable watchdog module
	SysCtrlRegs.WDKEY = 0x00;                // wrong key should restart
	SysCtrlRegs.WDKEY = 0x00;
	EDIS;

	while(1){}
}


