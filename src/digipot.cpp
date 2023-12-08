/*
 * digipot.c
 * This library is used for communicating with the MCP41HV series digipots
 *  Created on: 7Jun.,2019
 *      Author: samspencer
 *
 * Because of the simplicity of most digipots (they only have a handful of accepted commands),
 * return data on the MISO line for write commands is not monitored.
 * Instead, a monitored check is performed during the initialisation process to confirm hardware functionality.
 *
 * MCP41HV CubeMX settings:
 * SPI(0.0)
 * 	Clock Polarity (CPOL) - LOW
 * 	Clock Phase (CPHA) - 1 Edge
 *
 * 	OR
 *
 * 	SPI(1.1)
 * 	Clock Polarity (CPOL) - HIGH
 * 	Clock Phase (CPHA) - 2 Edge
 *
 */

/* Private Includes */

#include "digipot.h"

#define TEST_VALUE	83

/*********** MCP41HV *********/
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
#ifdef USE_MCP45HV_DIGIPOT
#include "i2c.h"
#define MCP45HV_BASE_ADDRESS	0b0111100		// bits 0-1 are configured by hardware pins
#define MCP45HV_WIPER_ADDRESS	0x00
#define MCP45HV_TCON_ADDRESS	0x04
#define MCP41HV_CMD_WRITE 		0b00				// 2-bit command to write to MCPHV45 Write address
#define MCP41HV_CMD_READ 			0b11				// 2-bit command to write to MCPHV45 Read address
#define MCP41HV_CMD_INCREMENT 0b01				// 2-bit command to write to MCPHV45 Increment address
#define MCP41HV_CMD_DECREMENT 0b10				// 2-bit command to write to MCPHV45 Decrement address
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

#ifdef USE_MCP45HV_DIGIPOT
DigipotStatus mcp45hv_init(Mcp45hvDigipot* mcp45hv, uint8_t a0Pin, uint8_t a1Pin)
{
	DigipotStatus status;
	uint8_t data = TEST_VALUE;
	uint8_t rxData;
	mcp45hv->address = MCP45HV_BASE_ADDRESS | (a1Pin<<1) | a0Pin;

	status = mcp45hv_write(mcp45hv, data);
	if(status != DigipotOk)
	{
		return status;
	}
	status = mcp45hv_read(mcp45hv, &rxData);
	if(status != DigipotOk)
	{
		return status;
	}
	if(rxData != TEST_VALUE)
	{
		return DigipotHardwareError;
	}
	return DigipotOk;
}

DigipotStatus mcp45hv_write(Mcp45hvDigipot* mcp45hv, uint8_t data)
{
	uint8_t deviceAddress = mcp45hv->address << 1;
	uint8_t memoryAddress = (MCP45HV_WIPER_ADDRESS << 4) | (MCP41HV_CMD_WRITE << 2);
	uint8_t dataBuf[2] = {memoryAddress, data};
	while(HAL_I2C_GetState(mcp45hv->hi2c) != HAL_I2C_STATE_READY);
	if(HAL_I2C_Master_Transmit(mcp45hv->hi2c, deviceAddress, dataBuf, 2, HAL_MAX_DELAY) != HAL_OK)
	{
		return DigipotHalError;
	}
	return DigipotOk;
}

DigipotStatus mcp45hv_read(Mcp45hvDigipot* mcp45hv, uint8_t *data)
{
	uint8_t rxBuf[2];
	uint8_t deviceAddress = mcp45hv->address << 1;
	uint8_t memoryAddress = (MCP45HV_WIPER_ADDRESS << 4) | (MCP41HV_CMD_READ << 2);
	while(HAL_I2C_GetState(mcp45hv->hi2c) != HAL_I2C_STATE_READY);
	if(HAL_I2C_Mem_Read(mcp45hv->hi2c, deviceAddress, memoryAddress, 1, rxBuf, 2, HAL_MAX_DELAY) != HAL_OK)
	{
		return DigipotHalError;
	}
	data[0] = rxBuf[1];
	return DigipotOk;
}
#endif

void digipot_rxHandler()
{

}

void digipot_txHandler()
{

}

void digipot_SetPinState(Mcp41hvDigipot* digipot, uint8_t state)
{
#if MCU_CORE_RP2040
	gpio_put(digipot->pin, (bool)state);
#elif MCU_CORE_STM32
	HAL_GPIO_WritePin(digipot->port, digipot->pin, state); 
#endif
}
