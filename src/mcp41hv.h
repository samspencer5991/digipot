#ifndef MCP41HV_H_
#define MCP41HV_H_

/*
typedef struct
{
#if defined(FRAMEWORK_STM32CUBE)
	GPIO_TypeDef* port;				// GPIO port for chip select pin
	SPI_HandleTypeDef* hspi;	// handle for the SPI instance the digipot is connected to
#elif defined(FRAMEWORK_ARDUINO)
	SPIClass *hspi;
#endif
	uint16_t pin;							// GPIO pin number for chip select pin
	DigipotStatus status;
} Mcp41hvDigipot;

void mcp41hv_write(Mcp41hvDigipot* digipot, uint8_t data);
uint8_t mcp41hv_read(Mcp41hvDigipot* digipot);
void mcp41hv_increment(Mcp41hvDigipot* digipot);
void mcp41hv_decrement(Mcp41hvDigipot* digipot);
*/
#endif /* MCP41HV_H_ */