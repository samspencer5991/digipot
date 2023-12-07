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

#define TEST_VALUE	127

/*********** MCP41HV *********/
#ifdef USE_MCP41HV_DIGIPOT
#define MCP41HV_CMD_WRITE 		0x00	// 8-bit command to write to MCPHV41 Write address
#define MCP41HV_CMD_READ 			0x0f	// 8-bit command to write to MCPHV41 Read address
#define MCP41HV_CMD_INCREMENT 0x04	// 8-bit command to write to MCPHV41 Increment address
#define MCP41HV_CMD_DECREMENT 0x08	// 8-bit command to write to MCPHV41 Decrement address
#define CS_ASSERT 	GPIO_PIN_RESET
#define CS_RELEASE 	GPIO_PIN_SET
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

#ifdef USE_MCP41HV_DIGIPOT
Mcp41hvDigipot mcp41hvDigipots[NUM_MCP41HV];
uint16_t currentMcp41hvDigipot;
#endif

/*********** MCP45HV *********/

/**
  * @brief	This function takes a pointer to an mcp41hv type digipot type and performs a write/read check
  * 		The outcome of this check is reflected in the digipot->status value
  * @param 	pointer to the digipot to initialise
  * @retval	none
  */
#ifdef USE_MCP41HV_DIGIPOT
DigipotStatus mcp41hv_init(uint16_t index, GPIO_TypeDef* port, uint16_t pin, SPI_HandleTypeDef* hspi)
{
	mcp41hvDigipots[index].port = port;
	mcp41hvDigipots[index].pin = pin;
	mcp41hvDigipots[index].hspi = hspi;
	uint8_t data = TEST_VALUE;
	mcp41hvDigipots[index].status = DigipotOk;
	mcp41hv_write(index, data);
	if(mcp41hv_read(index) != TEST_VALUE)
	{
		mcp41hvDigipots[index].status = DigipotHardwareError;
	}
	return mcp41hvDigipots[index].status;
}

/**
  * @brief 	Writes to the volatile wiper address of MCP41HV digipots
  * @param 	data data to be written
  * @retval data read from SDO pin
  */
void mcp41hv_write(uint16_t index, uint8_t data)
{
	// prepare data buffer
	uint8_t txData[2] = {MCP41HV_CMD_WRITE, data};

	currentMcp41hvDigipot = index;
	// assert chip select pin to low
	HAL_GPIO_WritePin(mcp41hvDigipots[index].port, mcp41hvDigipots[index].pin, CS_ASSERT);

	// transmit data
	while(HAL_SPI_GetState(mcp41hvDigipots[index].hspi) != HAL_SPI_STATE_READY);
	if(HAL_SPI_Transmit_IT(mcp41hvDigipots[index].hspi, txData, 2) != HAL_OK)
	{
		mcp41hvDigipots[index].status = DigipotHalError;
	}
}

/**
  * @brief 	Writes to the read address and then reads the returned wiper value
  * @param 	data output variable to store read result
  * @retval data read from SDO pin
  */

uint8_t mcp41hv_read(uint16_t index)
{
	// assert chip select pin to low
	HAL_GPIO_WritePin(mcp41hvDigipots[index].port, mcp41hvDigipots[index].pin, CS_ASSERT);

	// dummy data in txData to
	uint8_t txData[2] = {MCP41HV_CMD_READ, 0x00};
	uint8_t rxData[2];
	// transmit data
	if(HAL_SPI_TransmitReceive(mcp41hvDigipots[index].hspi, txData, rxData, 2, 10) != HAL_OK)
	{
		mcp41hvDigipots[index].status = DigipotHalError;
	}
	// release chip select pin to high
	HAL_GPIO_WritePin(mcp41hvDigipots[index].port, mcp41hvDigipots[index].pin, CS_RELEASE);
	// check data to ensure valid command
	if(rxData[0] != 0xff)
	{
		mcp41hvDigipots[index].status = DigipotCmdError;
	}
	return rxData[1];
}

/**
  * @brief	Writes to the increment address of MCP41HV digipot
  * @param	*digipot Pointer to the digipot type to use
  * @retval none
  */
void mcp41hv_increment(uint16_t index)
{
	uint8_t cmd = MCP41HV_CMD_INCREMENT;
	// assert chip select pin LOW
	HAL_GPIO_WritePin(mcp41hvDigipots[index].port, mcp41hvDigipots[index].pin, CS_ASSERT);
	// transmit increment
	if(HAL_SPI_Transmit_IT(mcp41hvDigipots[index].hspi, &cmd, 1)  != HAL_OK)
	{
		mcp41hvDigipots[index].status = DigipotHalError;
	}
	// release chip select pin HIGH
	HAL_GPIO_WritePin(mcp41hvDigipots[index].port, mcp41hvDigipots[index].pin, CS_RELEASE);
}

/**
  * @brief	Writes to the decrement address of MCP41HV digipot
  * @param 	*digipot Pointer to the digipot type to use
  * @retval	none
  */
void mcp41hv_decrement(uint16_t index)
{
	uint8_t cmd = MCP41HV_CMD_DECREMENT;
	// assert chip select pin LOW
	HAL_GPIO_WritePin(mcp41hvDigipots[index].port, mcp41hvDigipots[index].pin, CS_ASSERT);
	// transmit decrement
	if(HAL_SPI_Transmit_IT(mcp41hvDigipots[index].hspi, &cmd, 1)  != HAL_OK)
	{
		mcp41hvDigipots[index].status = DigipotHalError;
	}
	// release chip select pin HIGH
	HAL_GPIO_WritePin(mcp41hvDigipots[index].port, mcp41hvDigipots[index].pin, CS_RELEASE);
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
#ifdef USE_MCP41HV_DIGIPOT
	HAL_GPIO_WritePin(mcp41hvDigipots[currentMcp41hvDigipot].port, mcp41hvDigipots[currentMcp41hvDigipot].pin, CS_RELEASE);
#endif
}
