#include "mcp45hv.h"
#include "Arduino.h"

#define MCP45HV_BASE_ADDRESS	0b0111100		// bits 0-1 are configured by hardware pins
#define MCP45HV_WIPER_ADDRESS	0x00
#define MCP45HV_TCON_ADDRESS	0x04
#define MCP41HV_CMD_WRITE 		0b00				// 2-bit command to write to MCPHV45 Write address
#define MCP41HV_CMD_READ 		0b11				// 2-bit command to write to MCPHV45 Read address
#define MCP41HV_CMD_INCREMENT 0b01				// 2-bit command to write to MCPHV45 Increment address
#define MCP41HV_CMD_DECREMENT 0b10				// 2-bit command to write to MCPHV45 Decrement address

#define TEST_VALUE				69	// Used for verification of digipot wiper setting

uint8_t mcp45hv_Init(Mcp45hvDigipot* mcp45hv, uint8_t a0Pin, uint8_t a1Pin, TwoWire* hi2c)
{
	uint8_t data = TEST_VALUE;
	uint8_t rxData;
	mcp45hv->address = MCP45HV_BASE_ADDRESS | (a1Pin<<1) | a0Pin;
	mcp45hv->hi2c = hi2c;
	mcp45hv_Write(mcp45hv, data);
	rxData = mcp45hv_Read(mcp45hv);
	if(rxData != TEST_VALUE)
		return DIGIPOT_FAULT;

	return DIGIPOT_OK;
}

void mcp45hv_Write(Mcp45hvDigipot* mcp45hv, uint8_t data)
{
	uint8_t deviceAddress = mcp45hv->address << 1;
	uint8_t memoryAddress = (MCP45HV_WIPER_ADDRESS << 4) | (MCP41HV_CMD_WRITE << 2);
	uint8_t dataBuf[2] = {memoryAddress, data};

	Wire.beginTransmission(mcp45hv->address);
	Wire.write(dataBuf[0]);
	Wire.write(dataBuf[1]);
	Wire.endTransmission();
	//HAL_I2C_Master_Transmit(mcp45hv->hi2c, deviceAddress, dataBuf, 2, HAL_MAX_DELAY);
	
}

uint8_t mcp45hv_Read(Mcp45hvDigipot* mcp45hv)
{
	uint8_t rxBuf[2];
	uint8_t deviceAddress = mcp45hv->address << 1;
	uint8_t memoryAddress = (MCP45HV_WIPER_ADDRESS << 4) | (MCP41HV_CMD_READ << 2);

	mcp45hv->hi2c->beginTransmission(mcp45hv->address);
	mcp45hv->hi2c->write(memoryAddress);
	mcp45hv->hi2c->endTransmission(false);

	mcp45hv->hi2c->requestFrom(mcp45hv->address, 2);
	mcp45hv->hi2c->requestFrom(mcp45hv->address, (size_t)2);
>>>>>>> Stashed changes
	if(mcp45hv->hi2c->available())
	{ 
		rxBuf[0] = mcp45hv->hi2c->read();   // First byte is 0x00
		#if DEBUG
		Serial.print("\nRead Wiper MSB:  ");
		Serial.println(buff);
		#endif
		rxBuf[1] = mcp45hv->hi2c->read();   // Second byte contains the wiper value
		#if DEBUG
		Serial.print("Read Wiper LSB:  ");
		Serial.println(buff);
		#endif
	}

	//HAL_I2C_Mem_Read(mcp45hv->hi2c, deviceAddress, memoryAddress, 1, rxBuf, 2, HAL_MAX_DELAY);

	return rxBuf[1];
}