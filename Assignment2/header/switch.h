// switch.h
#ifndef SWITCH_H
#define SWITCH_H

#define SWITCH_I2C_ADDR 0x03 << 1

#include <stdint.h>

// --- 5-way switch handling ---
typedef enum {
    TOP,
    LEFT,
    RIGHT,
    BOTTOM,
    CENTER,
    NONE
} SwitchDirection;

void Switch_Init(void);
void Check_Grove_Switch(void);
SwitchDirection Switch_ReadDirection(void);
const char* Switch_GetDirectionString(SwitchDirection dir);

#endif // SWITCH_H
