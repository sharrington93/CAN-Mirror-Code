//MCP2515 functions

#include <stdint.h>

#ifndef MCP2515_H

#define MCP2515_H

extern int MCP2515SendMessage(uint16_t ID, uint32_t ExtID, uint8_t DataL, uint8_t* Data);
extern int MCP2515GetMessage(uint8_t *dmesg);
extern int MCP2515GetMessageAvailable(void);
extern void MCP2515Mode(uint8_t Mode);
extern void RamInitMCP2515(uint8_t cnf1, uint8_t *data);
extern void MCP2515LoadTx(uint8_t n, uint16_t sid, uint32_t eid, uint8_t dl, uint8_t *data);
extern uint8_t MCP2515Read(uint8_t Addr);
extern void MCP2515Write(uint8_t Addr, uint8_t Data);
#endif
