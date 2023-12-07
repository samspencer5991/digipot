/*
 * digipot.h
 *
 *  Created on: Jan 16, 2020
 *      Author: Sam Work
 */

#ifndef DIGIPOT_H_
#define DIGIPOT_H_

#include "definition_linker.h"
// Default definitions for number of digipots

#ifdef USE_MCP41HV_DIGIPOT
#include "spi.h"
#endif

#ifdef USE_MCP45HV_DIGIPOT
#include "i2c.h"
#endif

typedef enum
{
	DigipotCmdError,			// an unknown or invalid command was transmitted to the digipot
	DigipotHardwareError,	// the digipot did not respond to commands or other hardware error
	DigipotHalError,			// an error was reported from a HAL peripheral function
	DigipotOk							// no error state
} DigipotStatus;

#ifdef USE_MCP41HV_DIGIPOT
typedef struct
{
	GPIO_TypeDef* port;				// GPIO port for chip select pin
	uint16_t pin;							// GPIO pin number for chip select pin
	SPI_HandleTypeDef* hspi;	// handle for the SPI instance the digipot is connected to
	DigipotStatus status;
} Mcp41hvDigipot;
#endif

#ifdef USE_MCP45HV_DIGIPOT
typedef struct
{
	uint8_t address;					// I2C address
	I2C_HandleTypeDef* hi2c;	// Handle for the I2C instance the digipot is connected to
} Mcp45hvDigipot;
#endif


#ifdef USE_MCP41HV_DIGIPOT
DigipotStatus mcp41hv_init(uint16_t index, GPIO_TypeDef* port, uint16_t pin, SPI_HandleTypeDef* hspi);
void mcp41hv_write(uint16_t index, uint8_t data);
uint8_t mcp41hv_read(uint16_t index);
void mcp41hv_increment(uint16_t index);
void mcp41hv_decrement(uint16_t index);
#endif

#ifdef USE_MCP45HV_DIGIPOT
DigipotStatus mcp45hv_init(Mcp45hvDigipot* mcp45hv, uint8_t a0Pin, uint8_t a1Pin);
DigipotStatus mcp45hv_write(Mcp45hvDigipot* mcp45hv, uint8_t data);
DigipotStatus mcp45hv_read(Mcp45hvDigipot* mcp45hv, uint8_t* data);
void digipot_rxHandler();
void digipot_txHandler();
#endif

void digipot_txHandler();
void digipot_rxHandler();


#endif /* DIGIPOT_H_ */
