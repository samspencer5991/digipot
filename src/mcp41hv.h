#ifndef MCP41HV_H_
#define MCP41HV_H_

#include "SPI.h"
#include "stdint.h"

#define DIGIPOT_OK		1
#define DIGIPOT_FAULT	0

struct Mcp41hvDigipot
{
	SPIClass *hspi;
};

typedef struct Mcp41hvDigipot Mcp41hvDigipot;

void mcp41hv_Write(Mcp41hvDigipot* digipot, uint8_t data);
uint8_t mcp41hv_Read(Mcp41hvDigipot* digipot);
void mcp41hv_Increment(Mcp41hvDigipot* digipot);
void mcp41hv_Decrement(Mcp41hvDigipot* digipot);
#endif /* MCP41HV_H_ */