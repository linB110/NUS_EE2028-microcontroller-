#ifndef MODE_H
#define MODE_H

#include <stdint.h>

#define COMFORT_TEMP 35.0f
#define CHILL_TEMP 31.0f
#define COMFORT_HUMID 82.0f
#define LOW_HUMID 81.0f

typedef enum {
    MODE_STANDARD,
    MODE_VIEW_TEMP,
    MODE_VIEW_HUMID,
    MODE_COUNT
} SystemMode;

void Mode_Init(void);
void Mode_Next(void);
void Mode_Prev(void);
SystemMode Mode_GetCurrent(void);
const char* Mode_GetName(SystemMode mode);
void Mode_Handle(SystemMode mode);

#endif // MODE_H
