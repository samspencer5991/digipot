#include "mcp41hv.h"

#ifdef USE_MCP41HV_DIGIPOT
#include "SPI.h"
#include <api/HardwareSPI.h>
#include <Arduino.h>
#define MCP41HV_CMD_WRITE 			0x00	// 8-bit command to write to MCPHV41 Write address
#define MCP41HV_CMD_READ 			0x0f	// 8-bit command to write to MCPHV41 Read address
#define MCP41HV_CMD_INCREMENT 	0x04	// 8-bit command to write to MCPHV41 Increment address
#define MCP41HV_CMD_DECREMENT 	0x08	// 8-bit command to write to MCPHV41 Decrement address
void digipot_SetPinState(Mcp41hvDigipot* digipot, uint8_t state);
#endif


/*********** MCP45HV *********/

/**
  * @brief	This function takes a pointer to an mcp41hv type digipot type and performs a write/read check
  * 		The outcome of this check is reflected in the digipot->status value
  * @param 	pointer to the digipot to initialise
  * @retval	none
  */
#ifdef USE_MCP41HV_DIGIPOT
/**
  * @brief 	Writes to the volatile wiper address of MCP41HV digipots
  * @param 	data data to be written
  * @retval data read from SDO pin
  */
void mcp41hv_write(Mcp41hvDigipot* digipot, uint8_t data)
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
	digipot->hspi->beginTransaction(SPISettings(1330000, MSBFIRST, SPI_MODE0));
	digipot->hspi->transfer(&txData, 2);
	digipot->hspi->endTransaction();
#endif
}

/**
  * @brief 	Writes to the read address and then reads the returned wiper value
  * @param 	data output variable to store read result
  * @retval data read from SDO pin
  */

uint8_t mcp41hv_read(Mcp41hvDigipot* digipot)
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
	digipot->hspi->beginTransaction(SPISettings(1330000, MSBFIRST, SPI_MODE0));
	rxData[0] = digipot->hspi->transfer(txData[0]);
	rxData[1] = digipot->hspi->transfer(txData[1]);
	digipot->hspi->endTransaction();
#endif
	// check data to ensure valid command
	if(rxData[0] != 0xff)
	{
		digipot->status = DigipotCmdError;
	}
	return rxData[1];
}

/**
  * @brief	Writes to the increment address of MCP41HV digipot
  * @param	*digipot Pointer to the digipot type to use
  * @retval none
  */
void mcp41hv_increment(Mcp41hvDigipot* digipot)
{
	uint8_t cmd = MCP41HV_CMD_INCREMENT;
#if defined(FRAMEWORK_STM32CUBE)
	// transmit decrement
	if(HAL_SPI_Transmit_IT(digipot->hspi, &cmd, 1)  != HAL_OK)
	{
		digipot->status = DigipotHalError;
	}
#elif defined(FRAMEWORK_ARDUINO)
	digipot->hspi->beginTransaction(SPISettings(1330000, MSBFIRST, SPI_MODE0));
	digipot->hspi->transfer(&cmd, 1);
	digipot->hspi->endTransaction();
#endif
}

/**
  * @brief	Writes to the decrement address of MCP41HV digipot
  * @param 	*digipot Pointer to the digipot type to use
  * @retval	none
  */
void mcp41hv_decrement(Mcp41hvDigipot* digipot)
{
	uint8_t cmd = MCP41HV_CMD_DECREMENT;
#if defined(FRAMEWORK_STM32CUBE)
	// transmit decrement
	if(HAL_SPI_Transmit_IT(digipot->hspi, &cmd, 1)  != HAL_OK)
	{
		digipot->status = DigipotHalError;
	}
#elif defined(FRAMEWORK_ARDUINO)
	digipot->hspi->beginTransaction(SPISettings(1330000, MSBFIRST, SPI_MODE0));
	digipot->hspi->transfer(&cmd, 1);
	digipot->hspi->endTransaction();
#endif
}
#endif