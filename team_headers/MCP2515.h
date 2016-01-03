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
extern void MCP2515ReadBlock(unsigned int Addr, unsigned int* buf, unsigned int length);
extern void MCP2515Write(unsigned int Addr, unsigned int Data);
//***********************buffer functions**********************************
extern void Buffer_Clear(buffer_struct* buf);
extern int Buffer_MCPGetMessage(buffer_struct* buf, int rxbn);
extern int Buffer_MCPFillMessage(buffer_struct* buf, int txbn);
extern int Buffer_Read(buffer_struct* buf, unsigned int* data);
extern int Buffer_Write(buffer_struct* buf, unsigned int* data);


#endif
