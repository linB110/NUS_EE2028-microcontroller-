// sensor.c
#include "sensor.h"
#include "main.h"
#include <stdio.h>

#define MAX_HISTORY 10

static float temp_history[MAX_HISTORY] = {0};
static uint8_t temp_index = 0;
static uint8_t temp_valid_count = 0;

static float humid_history[MAX_HISTORY] = { 0 };
static uint8_t humid_index = 0;
static uint8_t humid_valid_count = 0;


// sensor data
float accel_data[3] = { 0 };
int16_t accel_data_i16[3] = { 0 };

void Sensor_Init(void) {
	//BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
    // ADC and GPIO already initialized in main
    for (int i = 0; i < MAX_HISTORY; i++) {
        temp_history[i] = 0;
//        humid_history[i] = 0;
    }
    temp_index = 0;
//    humid_index = 0;
}

float read_temperature(void){
	float temp_data;
    temp_data = BSP_TSENSOR_ReadTemp();
    return temp_data;
}

float get_AvgTemperature(void) {
    const float* history = get_Temperature_History();
    float sum = 0.0f;

    if (temp_valid_count == 0) return 0;

    for (int i = 0; i < temp_valid_count; ++i) {
        sum += history[i];
    }

    return sum / temp_valid_count;
}

void update_Temperature(float new_val) {
    temp_history[temp_index] = new_val;
    temp_index = (temp_index + 1) % MAX_HISTORY;

    if (temp_valid_count < MAX_HISTORY)
        temp_valid_count++;
}


const float* get_Temperature_History(void) {
    static float ordered[MAX_HISTORY];

    for (int i = 0; i < temp_valid_count; ++i) {
        ordered[i] = temp_history[(temp_index + MAX_HISTORY - temp_valid_count + i) % MAX_HISTORY];
    }

    return ordered;
}

uint8_t get_Temperature_HistoryCount(void){
	return temp_valid_count;
}


float read_humidity(void){
	float humidity;
	humidity = BSP_HSENSOR_ReadHumidity();
	return humidity;
}

void update_Humidity(float new_val) {
    humid_history[humid_index] = new_val;
    humid_index = (humid_index + 1) % MAX_HISTORY;

    if (humid_valid_count < MAX_HISTORY)
        humid_valid_count++;
}

const float* get_Humidity_History(void) {
    static float ordered[MAX_HISTORY];

    for (int i = 0; i < humid_valid_count; ++i) {
        ordered[i] = humid_history[(humid_index + MAX_HISTORY - humid_valid_count + i) % MAX_HISTORY];
    }

    return ordered;
}

float get_AvgHumidity(void) {
    const float* history = get_Humidity_History();
    float sum = 0.0f;

    if (humid_valid_count == 0) return 0;

    for (int i = 0; i < humid_valid_count; ++i) {
        sum += history[i];
    }

    return sum / humid_valid_count;
}

uint8_t get_Humidity_HistoryCount(void) {
    return humid_valid_count;
}


uint16_t read_ADC(void) {
    if (ADC1->CR & ADC_CR_ADSTART) return 0;

    ADC1->CR |= ADC_CR_ADSTART;

    uint32_t timeout = 3000;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0 && --timeout);

    if (timeout == 0) return 0xFFFF;
    return (uint16_t)ADC1->DR;
}

