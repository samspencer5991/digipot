#include "mcp41hv.h"
#include "Arduino.h"

#define MCP41HV_CMD_WRITE 			0x00	// 8-bit command to write to MCPHV41 Write address
#define MCP41HV_CMD_READ 			0x0f	// 8-bit command to write to MCPHV41 Read address
#define MCP41HV_CMD_INCREMENT 	0x04	// 8-bit command to write to MCPHV41 Increment address
#define MCP41HV_CMD_DECREMENT 	0x08	// 8-bit command to write to MCPHV41 Decrement address

#define TEST_VALUE				69	// Used for verification of digipot wiper setting
#define SPI_FREQUENCY			1000000
#define SPI_BUS					SPI1

void mcp41hv_Write(Mcp41hvDigipot* mcp41hv, uint8_t data)
{
	// prepare data buffer
	uint8_t txData[2] = {MCP41HV_CMD_WRITE, data};
	SPI_BUS.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	SPI_BUS.transfer(&txData, 2);
	SPI_BUS.endTransaction();
}

/**
  * @brief 	Writes to the read address and then reads the returned wiper value
  * @param 	data output variable to store read result
  * @retval data read from SDO pin
  */

uint8_t mcp41hv_Read(Mcp41hvDigipot* mcp41hv)
{
	// dummy data in txData to
	uint8_t txData[2] = {MCP41HV_CMD_READ, 0x00};
	uint8_t rxData[2];
	// transmit data
	SPI_BUS.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	rxData[0] = SPI_BUS.transfer(txData[0]);
	rxData[1] = SPI_BUS.transfer(txData[1]);
	SPI_BUS.endTransaction();
	return rxData[1];
}

/**
  * @brief	Writes to the increment address of MCP41HV digipot
  * @param	*digipot Pointer to the digipot type to use
  * @retval none
  */
void mcp41hv_increment(Mcp41hvDigipot* mcp41hv)
{
	uint8_t cmd = MCP41HV_CMD_INCREMENT;

	mcp41hv->hspi->beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	mcp41hv->hspi->transfer(&cmd, 1);
	mcp41hv->hspi->endTransaction();
}

/**
  * @brief	Writes to the decrement address of MCP41HV digipot
  * @param 	*digipot Pointer to the digipot type to use
  * @retval	none
  */
void mcp41hv_decrement(Mcp41hvDigipot* mcp41hv)
{
	uint8_t cmd = MCP41HV_CMD_DECREMENT;

	mcp41hv->hspi->beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	mcp41hv->hspi->transfer(&cmd, 1);
	mcp41hv->hspi->endTransaction();
}
