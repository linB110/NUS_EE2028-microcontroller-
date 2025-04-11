#ifndef LEDMAT_H
#define LEDMAT_H

#include <stdint.h>

void LEDMAT_Init(void);
void LEDMAT_Clear(void);
void LEDMAT_SetPixel(uint8_t row, uint8_t col, uint8_t state);
void LEDMAT_Update(void);

// 表情功能
typedef enum {
    FACE_HAPPY,
    FACE_SAD,
    FACE_NEUTRAL
} LEDFaceType;

void LEDMAT_ShowFace(LEDFaceType face);

#endif // LEDMAT_H
