#include "mode.h"
#include "oled.h"
#include "ledMat.h"
#include "sensor.h"
#include "switch.h"

// 當前模式
static SystemMode currentMode = MODE_STANDARD;

void Mode_Init(void) {
    currentMode = MODE_STANDARD;
}

void Mode_Next(void) {
    currentMode = (currentMode + 1) % MODE_COUNT;
}

void Mode_Prev(void) {
    currentMode = (currentMode + MODE_COUNT - 1) % MODE_COUNT;
}

SystemMode Mode_GetCurrent(void) {
    return currentMode;
}

const char* Mode_GetName(SystemMode mode) {
    switch (mode) {
        case MODE_STANDARD: return "STANDARD";
        case MODE_VIEW_TEMP: return "TEMP";
        case MODE_VIEW_HUMID: return "HUMID";
        default: return "UNKNOWN";
    }
}


void Mode_Handle(SystemMode mode) {
    // 根據模式更新 OLED 顯示
    OLED_Clear();

    switch (mode) {
        case MODE_STANDARD:
            OLED_DrawText(0, 0, "MODE: STANDARD");
            break;

        case MODE_VIEW_TEMP: {
            OLED_Clear();

            float temp = read_temperature();
            update_Temperature(temp);

            const float* raw_history = get_Temperature_History();
            uint8_t count = get_Temperature_HistoryCount();

            OLED_DrawText(0, 0, "TEMP HISTORY");

            float sum = 0;
            for (int i = 0; i < count; i++) sum += raw_history[i];
            float avg = (count > 0) ? (sum / count) : 0;

            char buf[20];
            snprintf(buf, sizeof(buf), "AVG TEMP: %.2fC", avg);
            OLED_DrawText(0, 2, buf);

            // 畫圖參數
            const int baseX = 20;
            const int baseY = 63;
            const int graphHeight = 40;
            const float maxVal = 32.0f;
            const float minVal = 30.0f;
            const int pointGap = 10;

            // baseline 設為中間值
            float midVal = (maxVal + minVal) / 2.0f;
            int y_base = baseY - (int)((midVal - minVal) * graphHeight / (maxVal - minVal));
            for (int x = baseX; x <= baseX + pointGap * (count - 1); x += 2) {
                OLED_DrawPixel(x, y_base);
            }
            OLED_DrawText(100, y_base - 5, "MID");

            // 繪製折線圖（從最舊到最新）
            for (int i = 0; i < count - 1; i++) {
                int x0 = baseX + i * pointGap;
                int x1 = baseX + (i + 1) * pointGap;

                float val0 = raw_history[i];
                float val1 = raw_history[i + 1];

                int y0 = baseY - (int)((val0 - minVal) * graphHeight / (maxVal - minVal));
                int y1 = baseY - (int)((val1 - minVal) * graphHeight / (maxVal - minVal));

                OLED_DrawLine(x0, y0, x1, y1);
            }

            OLED_Update();
            break;
        }


        case MODE_VIEW_HUMID: {
            OLED_Clear();

            float h = read_humidity();
            update_Humidity(h);

            const float* history = get_Humidity_History();
            uint8_t count = get_Humidity_HistoryCount();

            OLED_DrawText(0, 0, "HUMID HISTORY");

            float sum = 0;
            for (int i = 0; i < count; i++) sum += history[i];
            float avg = (count > 0) ? (sum / count) : 0;

            char buf[24];
            snprintf(buf, sizeof(buf), "AVG HUMID: %.2f%%", avg);
            OLED_DrawText(0, 2, buf);

            const int baseX = 20;
            const int baseY = 63;
            const int graphHeight = 40;
            const float maxHum = 100.0f;
            const float minHum = 30.0f;
            const int pointGap = 10;

            int y_base = baseY - (int)((COMFORT_HUMID - minHum) * graphHeight / (maxHum - minHum));
            for (int x = baseX; x < 128; x += 2) {
                OLED_DrawPixel(x, y_base);
            }
            OLED_DrawText(100, y_base - 5, "82%");

            for (int i = 0; i < count - 1; i++) {
                int x0 = baseX + pointGap * i;
                int x1 = baseX + pointGap * (i + 1);

                int y0 = baseY - (int)((history[i] - minHum) * graphHeight / (maxHum - minHum));
                int y1 = baseY - (int)((history[i + 1] - minHum) * graphHeight / (maxHum - minHum));

                OLED_DrawLine(x0, y0, x1, y1);
            }

            OLED_Update();
            break;
        }




        default:
            OLED_DrawText(0, 0, "DEFAULT MODE");
            break;
    }

    OLED_Update();
}
