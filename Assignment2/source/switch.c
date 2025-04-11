// switch.c
#include "switch.h"
#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

SwitchDirection currentDirection = NONE;

void Switch_Init(void) {
    // 此處預留初始化，如需開啟 GPIO 或中斷設定，可加入
}

SwitchDirection Switch_GetDirection(void) {
    uint8_t buf[9];
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
        &hi2c1,
        SWITCH_I2C_ADDR,
        0x01,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        9,
        HAL_MAX_DELAY
    );

    if (result != HAL_OK) {
        return NONE;
    }

    if ((buf[4] & 0x01) == 0) return TOP;
    if ((buf[5] & 0x01) == 0) return LEFT;
    if ((buf[6] & 0x01) == 0) return BOTTOM;
    if ((buf[7] & 0x01) == 0) return RIGHT;
    if ((buf[8] & 0x01) == 0) return CENTER;

    return NONE;
}

const char* Switch_DirectionString(SwitchDirection dir) {
    switch (dir) {
        case TOP: return "TOP";
        case LEFT: return "LEFT";
        case RIGHT: return "RIGHT";
        case BOTTOM: return "BOTTOM";
        case CENTER: return "CENTER";
        default: return "NONE";
    }
}

SwitchDirection Switch_ReadDirection(void) {
	printf("Switch dir: %s", Switch_DirectionString(currentDirection));
    return Switch_GetDirection();
}

void Check_Grove_Switch(void)
{
    uint8_t buf[9];
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
        &hi2c1,
        0x03 << 1,
        0x01,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        9,
        HAL_MAX_DELAY
    );

    if (result != HAL_OK)
    {
        return;
    }

    if ((buf[4] & 0x01) == 0) currentDirection = TOP;
    else if ((buf[5] & 0x01) == 0) currentDirection = LEFT;
    else if ((buf[6] & 0x01) == 0) currentDirection = BOTTOM;
    else if ((buf[7] & 0x01) == 0) currentDirection = RIGHT;
    else if ((buf[8] & 0x01) == 0) currentDirection = CENTER;
    else currentDirection = NONE;
}
