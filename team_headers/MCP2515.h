//MCP2515 functions


#ifndef MCP2515_H

#define MCP2515_H

extern int MCP2515SendMessage(unsigned int ID, unsigned long ExtID, unsigned int DataL, unsigned int* Data);
extern int MCP2515GetMessage(unsigned int *dmesg);
extern int MCP2515GetMessageAvailable(void);
extern void MCP2515Mode(unsigned int Mode);
extern void PgmInitMCP2515(unsigned int cnf1, const unsigned int *data);
extern void MCP2515LoadTx(unsigned int n, unsigned int sid, unsigned long eid, unsigned int dl, unsigned int *data);
extern unsigned int MCP2515Read(unsigned int Addr);
extern void MCP2515Write(unsigned int Addr, unsigned int Data);

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
#endif
