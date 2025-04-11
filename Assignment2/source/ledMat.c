#include "ledmat.h"
#include "main.h"
#include "stm32l4xx_hal.h"

#define LED_MATRIX_ADDR  (0x70 << 1)

static uint8_t LED_Buffer[8] = { 0 };
extern I2C_HandleTypeDef hi2c1;


void LEDMAT_Init(void) {
    uint8_t cmd;

    cmd = 0x21;  // enable oscillator
    HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_ADDR, &cmd, 1, HAL_MAX_DELAY);

    cmd = 0x81;  // display on, blinking off
    HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_ADDR, &cmd, 1, HAL_MAX_DELAY);

    cmd = 0xE0 | 0x0F;  // brightness max
    HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_ADDR, &cmd, 1, HAL_MAX_DELAY);

    LEDMAT_Clear();
}

void LEDMAT_Update(void) {
    uint8_t data[17] = {0};
    data[0] = 0x00;
    for (int i = 0; i < 8; i++) {
        data[1 + i * 2] = LED_Buffer[i];
        data[2 + i * 2] = 0x00;
    }
    HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_ADDR, data, 17, HAL_MAX_DELAY);
}

void LEDMAT_Clear(void) {
    for (int i = 0; i < 8; i++) {
        LED_Buffer[i] = 0x00;
    }
    LEDMAT_Update();
}

void LEDMAT_SetPixel(uint8_t row, uint8_t col, uint8_t state) {
    if (row > 7 || col > 7) return;

    if (state) {
        LED_Buffer[row] |= (1 << col);
    } else {
        LED_Buffer[row] &= ~(1 << col);
    }

    LEDMAT_Update();
}

// 簡單表情圖案
static const uint8_t face_happy[8] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10100101,
    0b10011001,
    0b01000010,
    0b00111100
};

static const uint8_t face_sad[8] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10011001,
    0b10100101,
    0b01000010,
    0b00111100
};

static const uint8_t face_neutral[8] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10111101,
    0b10000001,
    0b01000010,
    0b00111100
};

void LEDMAT_ShowFace(LEDFaceType face) {
    const uint8_t* pattern = face_happy;

    switch (face) {
        case FACE_HAPPY:
            pattern = face_happy;
            break;
        case FACE_SAD:
            pattern = face_sad;
            break;
        case FACE_NEUTRAL:
            pattern = face_neutral;
            break;
    }

    for (int i = 0; i < 8; i++) {
        LED_Buffer[i] = pattern[i];
    }

    LEDMAT_Update();
}
