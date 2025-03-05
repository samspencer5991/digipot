#include "mcp41hv.h"

#ifdef USE_MCP41HV_DIGIPOT
#include "SPI.h"
#include <api/HardwareSPI.h>
#include <Arduino.h>
#define MCP41HV_CMD_WRITE 			0x00	// 8-bit command to write to MCPHV41 Write address
#define MCP41HV_CMD_READ 			0x0f	// 8-bit command to write to MCPHV41 Read address
#define MCP41HV_CMD_INCREMENT 	0x04	// 8-bit command to write to MCPHV41 Increment address
#define MCP41HV_CMD_DECREMENT 	0x08	// 8-bit command to write to MCPHV41 Decrement address
#endif


#define SPI_FREQUENCY			1000000

#ifdef USE_MCP41HV_DIGIPOT


void mcp41hv_Write(Mcp41hvDigipot* mcp41hv, uint8_t data)
{
	// prepare data buffer
	uint8_t txData[2] = {MCP41HV_CMD_WRITE, data};
#if defined(FRAMEWORK_STM32CUBE)
	// transmit decrement
	if(HAL_SPI_Transmit_IT(digipot->hspi, &txData, 2)  != HAL_OK)
	{
		digipot->status = DigipotHalError;
	}
#elif defined(FRAMEWORK_ARDUINO)
	mcp41hv->hspi->beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	mcp41hv->hspi->transfer(&txData, 2);
	mcp41hv->hspi->endTransaction();
#endif
}

/**
  * @brief 	Writes to the read address and then reads the returned wiper value
  * @param 	data output variable to store read result
  * @retval data read from SDO pin
  */

int mcp41hv_Read(Mcp41hvDigipot* mcp41hv)
{
	// dummy data in txData to
	uint8_t txData[2] = {MCP41HV_CMD_READ, 0x00};
	uint8_t rxData[2];
	// transmit data
#if defined(FRAMEWORK_STM32CUBE)
	if(HAL_SPI_TransmitReceive(digipot->hspi, txData, rxData, 2, 10) != HAL_OK)
	{
		digipot->status = DigipotHalError;
	}
#elif defined(FRAMEWORK_ARDUINO)
	mcp41hv->hspi->beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	rxData[0] = mcp41hv->hspi->transfer(txData[0]);
	rxData[1] = mcp41hv->hspi->transfer(txData[1]);
	mcp41hv->hspi->endTransaction();
#endif
	// check data to ensure valid command
	if(rxData[0] != 0xff)
		return -1;
	
	return rxData[1];
}

/**
  * @brief	Writes to the increment address of MCP41HV digipot
  * @param	*digipot Pointer to the digipot type to use
  * @retval none
  */
void mcp41hv_Increment(Mcp41hvDigipot* mcp41hv)
{
	uint8_t cmd = MCP41HV_CMD_INCREMENT;
#if defined(FRAMEWORK_STM32CUBE)
	// transmit decrement
	if(HAL_SPI_Transmit_IT(digipot->hspi, &cmd, 1)  != HAL_OK)
	{
		digipot->status = DigipotHalError;
	}
#elif defined(FRAMEWORK_ARDUINO)
mcp41hv->hspi->beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	mcp41hv->hspi->transfer(&cmd, 1);
	mcp41hv->hspi->endTransaction();
#endif
}

/**
  * @brief	Writes to the decrement address of MCP41HV digipot
  * @param 	*digipot Pointer to the digipot type to use
  * @retval	none
  */
void mcp41hv_Decrement(Mcp41hvDigipot* mcp41hv)
{
	uint8_t cmd = MCP41HV_CMD_DECREMENT;
#if defined(FRAMEWORK_STM32CUBE)
	// transmit decrement
	if(HAL_SPI_Transmit_IT(digipot->hspi, &cmd, 1)  != HAL_OK)
	{
		digipot->status = DigipotHalError;
	}
#elif defined(FRAMEWORK_ARDUINO)
	mcp41hv->hspi->beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
	mcp41hv->hspi->transfer(&cmd, 1);
	mcp41hv->hspi->endTransaction();
#endif
}
#endif