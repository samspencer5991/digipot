#ifndef MCP45HV_H_
#define MCP45HV_H_

#include "Wire.h"
#include "stdint.h"

#define DIGIPOT_OK		1
#define DIGIPOT_FAULT	0

typedef struct
{
	// Don't touch
	uint8_t address;		// I2C address
	TwoWire* hi2c;			// Handle for the I2C instance
} Mcp45hvDigipot;

uint8_t mcp45hv_Init(Mcp45hvDigipot* mcp45hv, uint8_t a0Pin, uint8_t a1Pin, TwoWire* hi2c);
void mcp45hv_Write(Mcp45hvDigipot* mcp45hv, uint8_t data);
uint8_t mcp45hv_Read(Mcp45hvDigipot* mcp45hv);

#endif /* MCP45HV_H_ */