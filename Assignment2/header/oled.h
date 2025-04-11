#ifndef OLED_H
#define OLED_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define OLED_ADDR         (0x3C << 1)
#define OLED_WIDTH        128
#define OLED_HEIGHT       64

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_DrawChar(uint8_t x, uint8_t y, char c);
void OLED_DrawText(uint8_t x, uint8_t y, const char* text);
void OLED_WriteCommand(uint8_t cmd);

// OLED graph
void OLED_DrawPixel(uint8_t x, uint8_t y);
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

extern uint8_t OLED_Buffer[1024];
extern I2C_HandleTypeDef hi2c1;


#endif // OLED_H
