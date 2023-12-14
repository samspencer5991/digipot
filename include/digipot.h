#ifndef DIGIPOT_H_
#define DIGIPOT_H_

// I2C variant
#ifdef USE_MCP45HV_DIGIPOT
#include "mcp45hv.h"
#endif

// SPI variant
#ifdef USE_MCP41HV_DIGIPOT
#include "mcp41hv.h"
#endif

#endif /* DIGIPOT_H_ */
