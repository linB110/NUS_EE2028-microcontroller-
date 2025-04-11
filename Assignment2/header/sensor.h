// sensor.h
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"

extern float accel_data[3];
extern int16_t accel_data_i16[3];

void Sensor_Init(void);

// temperature
float read_temperature(void);
const float* get_Temperature_History(void);
float get_AvgTemperature(void);
void update_Temperature(float new_val);
uint8_t get_Temperature_HistoryCount(void);

// acceleration

// humidity
float read_humidity(void);
const float* get_Humidity_Temperature_History(void);
float get_AvgHumidity(void);
void update_Humidity(float new_val);
uint8_t get_Humidity_HistoryCount(void);

// sound sensor
uint16_t read_ADC(void);

#endif // SENSOR_H
