//MCP2515 functions
//Register and bit definitions for MICROCHIP MCP2515

#ifndef MCP2515_DEFS_H

#define MCP2515_DEFS_H

// Define MCP2515 register addresses

#define MCP_RXF0SIDH	0x00
#define MCP_RXF0SIDL	0x01
#define MCP_RXF0EID8	0x02
#define MCP_RXF0EID0	0x03
#define MCP_RXF1SIDH	0x04
#define MCP_RXF1SIDL	0x05
#define MCP_RXF1EID8	0x06
#define MCP_RXF1EID0	0x07
#define MCP_RXF2SIDH	0x08
#define MCP_RXF2SIDL	0x09
#define MCP_RXF2EID8	0x0A
#define MCP_RXF2EID0	0x0B
#define MCP_BFPCTRL	0x0C
#define MCP_TXRTSCTRL	0x0D	//{U,U,B2RTS,B1RTS,B0RTS,B2RTSM,B1RTSM,B0RTSM} BnRTS reads pin state. BnRTSM selects mode, 1=pin request, 0=digital input
#define MCP_CANSTAT	0x0E
#define MCP_CANCTRL	0x0F
#define MCP_RXF3SIDH	0x10
#define MCP_RXF3SIDL	0x11
#define MCP_RXF3EID8	0x12
#define MCP_RXF3EID0	0x13
#define MCP_RXF4SIDH	0x14
#define MCP_RXF4SIDL	0x15
#define MCP_RXF4EID8	0x16
#define MCP_RXF4EID0	0x17
#define MCP_RXF5SIDH	0x18
#define MCP_RXF5SIDL	0x19
#define MCP_RXF5EID8	0x1A
#define MCP_RXF5EID0	0x1B
#define MCP_TEC		0x1C
#define MCP_REC		0x1D
#define MCP_RXM0SIDH	0x20
#define MCP_RXM0SIDL	0x21
#define MCP_RXM0EID8	0x22
#define MCP_RXM0EID0	0x23
#define MCP_RXM1SIDH	0x24
#define MCP_RXM1SIDL	0x25
#define MCP_RXM1EID8	0x26
#define MCP_RXM1EID0	0x27
#define MCP_CNF3	0x28	//bits 7:0 are {SOF,WAKFIL,x,x,x,PHSEG22,PHSEG21,PHSEG20}
#define MCP_CNF2	0x29	//bits 7:0 are {BTLMODE,SAM,PHSEG12,PHSEG11,PHSEG10,PRSEG2:PRSEG0}
#define MCP_CNF1	0x2A	//bits 7:0 are {SJW1,SJW0,BRP5:BRP0}
#define MCP_CANINTE	0x2B
#define MCP_CANINTF	0x2C
#define MCP_EFLG	0x2D

#define MCP_TXB0CTRL	0x30
#define MCP_TXB0SIDH	0x31
#define MCP_TXB0SIDL	0x32
#define MCP_TXB0EID8	0x33
#define MCP_TXB0EID0	0x34
#define MCP_TXB0DLC	0x35
#define MCP_TXB0D0	0x36
#define MCP_TXB0D1	0x37
#define MCP_TXB0D2	0x38
#define MCP_TXB0D3	0x39
#define MCP_TXB0D4	0x3A
#define MCP_TXB0D5	0x3B
#define MCP_TXB0D6	0x3C
#define MCP_TXB0D7	0x3D

#define MCP_TXB1CTRL	0x40
#define MCP_TXB1SIDH	0x41
#define MCP_TXB1SIDL	0x42
#define MCP_TXB1EID8	0x43
#define MCP_TXB1EID0	0x44
#define MCP_TXB1DLC	0x45
#define MCP_TXB1D0	0x46
#define MCP_TXB1D1	0x47
#define MCP_TXB1D2	0x48
#define MCP_TXB1D3	0x49
#define MCP_TXB1D4	0x4A
#define MCP_TXB1D5	0x4B
#define MCP_TXB1D6	0x4C
#define MCP_TXB1D7	0x4D

#define MCP_TXB2CTRL	0x50
#define MCP_TXB2SIDH	0x51
#define MCP_TXB2SIDL	0x52
#define MCP_TXB2EID8	0x53
#define MCP_TXB2EID0	0x54
#define MCP_TXB2DLC	0x55
#define MCP_TXB2D0	0x56
#define MCP_TXB2D1	0x57
#define MCP_TXB2D2	0x58
#define MCP_TXB2D3	0x59
#define MCP_TXB2D4	0x5A
#define MCP_TXB2D5	0x5B
#define MCP_TXB2D6	0x5C
#define MCP_TXB2D7	0x5D

#define MCP_RXB0CTRL	0x60
#define MCP_RXB0SIDH	0x61
#define MCP_RXB0SIDL	0x62
#define MCP_RXB0EID8	0x63
#define MCP_RXB0EID0	0x64
#define MCP_RXB0DLC	0x65
#define MCP_RXB0D0	0x66
#define MCP_RXB0D1	0x67
#define MCP_RXB0D2	0x68
#define MCP_RXB0D3	0x69
#define MCP_RXB0D4	0x6A
#define MCP_RXB0D5	0x6B
#define MCP_RXB0D6	0x6C
#define MCP_RXB0D7	0x6D

#define MCP_RXB1CTRL	0x70
#define MCP_RXB1SIDH	0x71
#define MCP_RXB1SIDL	0x72
#define MCP_RXB1EID8	0x73
#define MCP_RXB1EID0	0x74
#define MCP_RXB1DLC	0x75
#define MCP_RXB1D0	0x76
#define MCP_RXB1D1	0x77
#define MCP_RXB1D2	0x78
#define MCP_RXB1D3	0x79
#define MCP_RXB1D4	0x7A
#define MCP_RXB1D5	0x7B
#define MCP_RXB1D6	0x7C
#define MCP_RXB1D7	0x7D

//define Bit fields for status regs

//Message Error Interrupt Flag bit
#define MCP_CANINTF_MERRF	0x80
//Wakeup Interrupt Flag bit
#define MCP_CANINTF_WAKIF	0x40
//Error Interrupt Flag bit (multiple sources in EFLG register)
#define MCP_CANINTF_ERRIF	0x20
//Transmit Buffer 2 Empty Interrupt Flag bit
#define MCP_CANINTF_TX2IF	0x10
//Transmit Buffer 1 Empty Interrupt Flag bit
#define MCP_CANINTF_TX1IF	0x08
//Transmit Buffer 0 Empty Interrupt Flag bit
#define MCP_CANINTF_TX0IF	0x04
//Receive Buffer 1 Full Interrupt Flag bit
#define MCP_CANINTF_RX1IF	0x02
//Receive Buffer 0 Full Interrupt Flag bit
#define MCP_CANINTF_RX0IF	0x01

//Receive Buffer 1 Overflow Flag bit
#define MCP_EFLG_RX1OVR		0x80
//Receive Buffer 0 Overflow Flag bit
#define MCP_EFLG_RX0OVR		0x40
//Bus-Off Error Flag bit (set when error counter reaches 255, reset after bus recovery sequence)
#define MCP_EFLG_TXBO		0x20
//Transmit Error-Passive Flag bit (TEC>=128)
#define MCP_EFLG_TXEP		0x10
//Receive Error-Passive Flag bit  (REC>=128)
#define MCP_EFLG_RXEP		0x08
//Transmit Error Warning Flag bit (TEC>=96)
#define MCP_EFLG_TXWAR		0x04
//Receive Error Warning Flag bit  (REC>=96)
#define MCP_EFLG_RXWAR		0x02
//Error Warning Flag bit		  (TEC>=96 | REC>=96)
#define MCP_EFLG_EWARN		0x01

//OPMOD2..0 Operation mode
#define MCP_CANSTAT_OPMOD	0xE0
#define MCP_CANSTAT_OPMOD2	0x80
#define MCP_CANSTAT_OPMOD1	0x40
#define MCP_CANSTAT_OPMOD0	0x20
//mode values
#define MCP_CANSTAT_NORMAL	0x00
#define MCP_CANSTAT_SLEEP	0x20
#define MCP_CANSTAT_LOOP	0x40
#define MCP_CANSTAT_LISTEN	0x60
#define MCP_CANSTAT_CONFIG	0x80
//ICOD2..0 interrupt Code
#define MCP_CANSTAT_ICOD	0x0E
#define MCP_CANSTAT_ICOD2	0x08
#define MCP_CANSTAT_ICOD1	0x04
#define MCP_CANSTAT_ICOD0	0x02
//interrupt values
#define MCP_CANSTAT_NOINT	0x00
#define MCP_CANSTAT_ERR		0x02
#define MCP_CANSTAT_WAKE	0x04
#define MCP_CANSTAT_TXB0	0x06
#define MCP_CANSTAT_TXB1	0x08
#define MCP_CANSTAT_TXB2	0x0A
#define MCP_CANSTAT_RXB0	0x0C
#define MCP_CANSTAT_RXB1	0x0E

//REQOP2..0 Request Operation Mode
#define MCP_CANCTRL_REQOP	0xE0
#define MCP_CANCTRL_REQOP2	0x80
#define MCP_CANCTRL_REQOP1	0x40
#define MCP_CANCTRL_REQOP0	0x20
//Operation mode values
#define MCP_CANCTRL_NORMAL	0x00
#define MCP_CANCTRL_SLEEP	0x20
#define MCP_CANCTRL_LOOP	0x40
#define MCP_CANCTRL_LISTEN	0x60
#define MCP_CANCTRL_CONFIG	0x80
//abort all pending transmissions bit
#define MCP_CANCTRL_ABAT	0x10
//one-shot mode enable
#define MCP_CANCTRL_OSM		0x08
//CLKOUT enable
#define MCP_CANCTRL_CLKEN	0x04
//CLKOUT pin Pre-scale
#define MCP_CANCTRL_CLKPRE1	0x02
#define MCP_CANCTRL_CLKPRE0	0x01

//define SPI commands

#define MCP_RESET	0xC0
#define MCP_READ	0x03
#define MCP_READRX0	0x90	//Read RX n,m = 0,0
#define MCP_READRX1	0x92	//Read RX n,m = 0,1
#define MCP_READRX2	0x94	//Read RX n,m = 1,0
#define MCP_READRX3	0x96	//Read RX n,m = 1,1
#define MCP_WRITE	0x02
#define MCP_LOADTX(a)	0x4 ## a	//Load TX buffer, a = 0-6
#define MCP_RTS0	0x81		//OR these together to select multiple
#define MCP_RTS1	0x82
#define MCP_RTS2	0x84
#define MCP_READSTATUS	0xA0
#define MCP_RXSTATUS	0xB0
#define MCP_BITMOD	0x05

#endif
