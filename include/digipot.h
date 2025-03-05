#ifndef DIGIPOT_H_
#define DIGIPOT_H_

#define DIGIPOT_TEST_VALUE				69	// Used for verification of digipot wiper setting

// I2C variant
#ifdef USE_MCP45HV_DIGIPOT
#include "mcp45hv.h"
#endif

// SPI variant
#ifdef USE_MCP41HV_DIGIPOT
#include "mcp41hv.h"
#endif

#endif /* DIGIPOT_H_ */
