#ifndef MCP41HV_H_
#define MCP41HV_H_

#include "Arduino.h"
#include "SPI.h"

typedef struct
{
#if defined(FRAMEWORK_STM32CUBE)
	SPI_HandleTypeDef* hspi;	// handle for the SPI instance the digipot is connected to
#elif defined(FRAMEWORK_ARDUINO)
	SPIClass *hspi;
#endif
} Mcp41hvDigipot;

void mcp41hv_Write(Mcp41hvDigipot* digipot, uint8_t data);
int mcp41hv_Read(Mcp41hvDigipot* digipot);
void mcp41hv_Increment(Mcp41hvDigipot* digipot);
void mcp41hv_Decrement(Mcp41hvDigipot* digipot);

#endif /* MCP41HV_H_ */