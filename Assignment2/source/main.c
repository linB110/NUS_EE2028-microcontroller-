#include "main.h"
#include "stm32l4xx_hal.h"

#include <stdio.h>
#include <stdbool.h>

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"

#include "sensor.h"
#include "oled.h"
#include "ledMat.h"
#include "mode.h"
#include "switch.h"

#define GYRO_SENSTIVITY  0.07f

//extern void initialise_monitor_handles(void);

void SystemClock_Config(void);
static void UART1_Init(void);
static void GPIO_Init(void);
static void ADC_Init(void);
void MX_I2C1_Init(void);
static void MX_GPIO_InitLED(void);

static void accelInt(void);
static void enableFree(void);

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

// mode ticks
uint32_t modeTick = 0;

//For button Press
uint32_t lastButtonTick = 0;
bool doublePress = false;
bool singlePress = false;
uint32_t pressButtonInterval = 1000;

//Boolean values for which mode is the watch in currently
bool standardMode = true;
bool emergencyMode = false;

//For free-fall interrupt
bool fallDet = false;

static SwitchDirection lastDir = NONE;
static bool isModeSelected = false;

extern uint8_t OLED_Buffer[1024];

//External Interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//For double clicking the button.
	if(GPIO_Pin == BUTTON_EXTI13_Pin) {
		uint32_t currentTick = HAL_GetTick();
		if ( (currentTick - lastButtonTick) <= pressButtonInterval){
			doublePress = true;
			singlePress = false;
			emergencyMode = true;
			standardMode = false;
		} else {
			doublePress = false;
			singlePress = true;
			emergencyMode = false;
			standardMode = true;
		}
		lastButtonTick = currentTick;

	}
	//For accelerometer interrupt
	if(emergencyMode == true){
		if(GPIO_Pin == GPIO_PIN_11) {
			fallDet = true;
		}
	}


}

int main(void)
{
    initialise_monitor_handles();

    /* HAL, Sytstem, communication initializations */
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    ADC_Init();
    MX_I2C1_Init();
    UART1_Init();

    /* GPIO initialization for LED and Buzzer*/
   	MX_GPIO_InitLED();

   	accelInt();
   	enableFree();

    /* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_LED_Init(LED2);
	BSP_MAGNETO_Init();
	BSP_GYRO_Init();
	BSP_PSENSOR_Init();

	/* external peripherals initializations */
    OLED_Init();
    LEDMAT_Init();
    Sensor_Init();
    Switch_Init();
    Mode_Init();

	BSP_PB_Init(BUTTON_EXTI13_Pin, BUTTON_MODE_EXTI); //For setting up button

    OLED_Clear();
    OLED_DrawText(10, 2, "SYSTEM START");
    OLED_Update();

    int seconds_count = 0;

	//Different tick counts for each device monitoring
	int ticks = HAL_GetTick();
	int ticksTemp = HAL_GetTick();
	int ticksFall = HAL_GetTick();
	int ticksMov = HAL_GetTick();
	int ticksPost = HAL_GetTick();
	int ticksLED2 = HAL_GetTick();
	int ticksLED1 = HAL_GetTick();

	//Boolean values for emergency mode detection
	bool feverDet = false;

	bool suddenMov = false;
	bool postBad = false;

	//Boolean values to send the message once
	bool standardModeMessage = true;
	bool emergencyModeMessage = true;

	//Variables for the data
	float temp_data;
	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };
	float gyro_data[3];
	int16_t gyro_data_i16[3] = { 0 };
	float magnet_data[3];
	int16_t magnet_data_i16[3] = { 0 };
	float humidity_data;
	float baro_data;

	//Thresholds
	float TEMP_THRESHOLD = BSP_TSENSOR_ReadTemp() + 1.0; //Calibrates the temp threshold
	float ACC_THRESHOLD = 19.50;
	float GYRO_THRESHOLD = 100000;
	float GYRO_NORMAL = 2660;
	float MAG_THRESHOLD = 900.0;

	float MAG_NORMAL[3] = { 0 };


//    SwitchDirection lastDir = NONE;
//    uint32_t tickStart = HAL_GetTick();
//    uint32_t sensorTick = HAL_GetTick();
//    uint32_t updateInterval = 300;
//    uint32_t sensorInterval = 2000;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 5-way switch unit test */
//        SwitchDirection dir = Switch_GetDirection();
//        printf("Direction: %s\n", Switch_DirectionString(dir));

        /* OLED unit test */
//      OLED_Clear();
//		OLED_DrawText(0, 0, "OLED TEST");
//		OLED_DrawText(0, 2, "123 ABC");
//		OLED_DrawText(0, 4, "HELLO :)");
//		OLED_Update();


        /* LED unit test */
//        LEDMAT_Clear();
//        LEDMAT_ShowFace(FACE_HAPPY);
//        LEDMAT_SetPixel(2, 2, 1);
//        LEDMAT_SetPixel(2, 5, 1);
//        LEDMAT_SetPixel(5, 2, 1);
//        LEDMAT_SetPixel(5, 3, 1);
//        LEDMAT_SetPixel(5, 4, 1);
//        LEDMAT_SetPixel(5, 5, 1);


        /* sound sensor unit test */
//        uint16_t adc_val = read_ADC();
//        printf("ADC value: %u\n", adc_val);


        /* mode unit test */
//        SwitchDirection dir = Switch_GetDirection();
//        static SwitchDirection lastDir = NONE;
//
//        if (dir != NONE && dir != lastDir) {
//            lastDir = dir;
//
//            if (dir == LEFT) {
//                Mode_Prev();
//            } else if (dir == RIGHT) {
//                Mode_Next();
//            } else if (dir == CENTER) {
//                Mode_Handle(Mode_GetCurrent());
//            }
//
//            // 更新顯示
//            Mode_Handle(Mode_GetCurrent());
//        }
//
//        if (dir == NONE) {
//            lastDir = NONE;
//        }
//
        /* read temperauture unit test */
//        float tempData = read_temperature();
//		update_Temperature(tempData);
//		float* tempHistory = get_Temperature_History();
//		printf("temp history is %.2f\n", tempData);
//		printf("temp history is %.2f\n", *tempHistory);

        /* read humidity unit test */
//        float humidity = read_humidity();
//        printf("humidity is %.2f\n", humidity);
//        HAL_Delay(500);

        /* temperature + OLED  + LED matrix unit test */
        SwitchDirection dir = Switch_GetDirection();

        if (!isModeSelected) {
            OLED_Clear();
            OLED_DrawText(0, 0, Mode_GetName(Mode_GetCurrent()));
            OLED_DrawText(0, 2, "PRESS TO SELECT");
            OLED_Update();
        }

        if (dir != NONE && dir != lastDir) {
            lastDir = dir;

            if (dir == RIGHT) {
                Mode_Next();
                isModeSelected = false;
            } else if (dir == LEFT) {
                Mode_Prev();
                isModeSelected = false;
            } else if (dir == CENTER) {
                isModeSelected = true;
            }

            OLED_Clear();
            OLED_DrawText(0, 0, Mode_GetName(Mode_GetCurrent()));
            if (!isModeSelected) {
                OLED_DrawText(0, 2, "PRESS TO SELECT");
            }
            OLED_Update();
        }

        if (dir == NONE) lastDir = NONE;

        if (emergencyMode == true){

		   //Prints out the message once
		   if(emergencyModeMessage == true){
			   char message_printEmergency[32];
			   sprintf(message_printEmergency, "Entering Emergency Mode.\r\n");
			   HAL_UART_Transmit(&huart1, (uint8_t*)message_printEmergency, strlen(message_printEmergency),0xFFFF);
			   emergencyModeMessage = false;
			   standardModeMessage = true;

			   isModeSelected = false;
		   }
		   //Playing the buzzer and blinking the LED2
		   if (HAL_GetTick() - ticksLED2 >= 500){
			   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
			   ticksLED2 = HAL_GetTick();
		   }

		   //Sampling data for emergency mode
		   temp_data = BSP_TSENSOR_ReadTemp();
		   BSP_ACCELERO_AccGetXYZ(accel_data_i16);
		   accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
		   accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
		   accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
		   BSP_GYRO_GetXYZ(gyro_data_i16);
		   gyro_data[0] = gyro_data_i16[0] * GYRO_SENSTIVITY;
		   gyro_data[1] = gyro_data_i16[1] * GYRO_SENSTIVITY;
		   gyro_data[2] = gyro_data_i16[2] * GYRO_SENSTIVITY;
		   BSP_MAGNETO_GetXYZ(magnet_data_i16);
		   magnet_data[0] = magnet_data_i16[0];
		   magnet_data[1] = magnet_data_i16[1];
		   magnet_data[2] = magnet_data_i16[2];
		   humidity_data = BSP_HSENSOR_ReadHumidity();
		   baro_data = BSP_PSENSOR_ReadPressure();

		   //Fever Detector
		   if (HAL_GetTick() - ticksTemp >= 5000){
			   if (temp_data >= TEMP_THRESHOLD) {
				   feverDet = true;
				   char message_printTemp[32];
				   sprintf(message_printTemp, "Fever is detected!\r\n");
				   HAL_UART_Transmit(&huart1, (uint8_t*)message_printTemp, strlen(message_printTemp),0xFFFF);
				   ticksTemp = HAL_GetTick();
			   } else {
				   feverDet = false;
			   }
		   }


		   //Fall Detector (Using interrupt)
		   if (fallDet == true){
			   char message_printFall[32];
			   sprintf(message_printFall, "Falling is detected!\r\n");
			   HAL_UART_Transmit(&huart1, (uint8_t*)message_printFall, strlen(message_printFall),0xFFFF);
			   fallDet = false;
		   }

		   //Sudden Movement Detector
		   if (HAL_GetTick() - ticksMov >= 5000){

			   if ( fabs(GYRO_NORMAL - fabs(gyro_data[0])) >= GYRO_THRESHOLD) {
				suddenMov = true;
				char message_printMov[32];
				sprintf(message_printMov, "Abnormal movement detected!\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_printMov, strlen(message_printMov),0xFFFF);
				ticksMov = HAL_GetTick();
			   } else {
				   suddenMov = false;
			   }
		   }

		   //Posture Monitor
		   if (HAL_GetTick() - ticksPost >= 5000){

			   if ( fabs( fabs(MAG_NORMAL[0]) - fabs(magnet_data[0])) >= MAG_THRESHOLD || fabs( fabs(MAG_NORMAL[1]) - fabs(magnet_data[1])) >= MAG_THRESHOLD || fabs( fabs(MAG_NORMAL[2]) - fabs(magnet_data[2])) >= MAG_THRESHOLD ){
				   postBad = true;
				   char message_printPost[32];
				   sprintf(message_printPost, "Incorrect Posture Detected!\r\n");
				   HAL_UART_Transmit(&huart1, (uint8_t*)message_printPost, strlen(message_printPost),0xFFFF);
				   ticksPost = HAL_GetTick();

			   } else {
				   postBad = false;
			   }

		   }

		   //LED blinking for emergency mode
		   if (HAL_GetTick() - ticksLED1 >= 166 && feverDet == true){
			   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			   ticksLED1 = HAL_GetTick();
		   }

		   if (HAL_GetTick() - ticksLED1 >= 250 && (fallDet == true || postBad == true || suddenMov == true)){
			   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			   ticksLED1 = HAL_GetTick();
		   }

		   continue;

	   } //end of emergency mode

        if (HAL_GetTick() - ticks >= 1000 && standardMode){
		//Sampling Data Every Second
		   temp_data = BSP_TSENSOR_ReadTemp();
		   BSP_ACCELERO_AccGetXYZ(accel_data_i16);
		   accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
		   accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
		   accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
		   BSP_GYRO_GetXYZ(gyro_data_i16);
		   gyro_data[0] = gyro_data_i16[0] * GYRO_SENSTIVITY;
		   gyro_data[1] = gyro_data_i16[1] * GYRO_SENSTIVITY;
		   gyro_data[2] = gyro_data_i16[2] * GYRO_SENSTIVITY;
		   BSP_MAGNETO_GetXYZ(magnet_data_i16);
		   magnet_data[0] = magnet_data_i16[0];
		   magnet_data[1] = magnet_data_i16[1];
		   magnet_data[2] = magnet_data_i16[2];
		   humidity_data = BSP_HSENSOR_ReadHumidity();
		   baro_data = BSP_PSENSOR_ReadPressure();

		   //Turns LED's off and sends the message once standard mode is entered
		   if(standardModeMessage == true){
			  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			  char message_printStandard[32];
			  sprintf(message_printStandard, "Entering Standard Mode.\r\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*)message_printStandard, strlen(message_printStandard),0xFFFF);
			  standardModeMessage = false;
			  emergencyModeMessage = true;
		   }

		   seconds_count++;
		   // Be careful about the buffer size used. Here, we assume that seconds_count does not exceed 6 decimal digits
		   char message_print[32];        // UART transmit buffer. See the comment in the line above.
		   sprintf(message_print, "%03d TEMP %0.2f ACC %0.2f %0.2f %0.2f \r\n", seconds_count,temp_data, accel_data[0], accel_data[1], accel_data[2]);
		   HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode

		   char message_printB[32];
		   sprintf(message_printB, "%03d GYRO_X %0.2f GYRO_Y %0.2f GYRO_Z %0.2f MAGNETO %0.2f %0.2f %0.2f \r\n", seconds_count, gyro_data[0], gyro_data[1], gyro_data[2],
				   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   magnet_data[0], magnet_data[1], magnet_data[2]);
		   HAL_UART_Transmit(&huart1, (uint8_t*)message_printB, strlen(message_printB),0xFFFF);

		   char message_printC[32];
		   sprintf(message_printC, "%03d HUMIDITY %0.2f and BARO %0.2f \r\n", seconds_count, humidity_data, baro_data);
		   HAL_UART_Transmit(&huart1, (uint8_t*)message_printC, strlen(message_printC),0xFFFF);

		   ticks = HAL_GetTick();// End of Standard Mode
		}

        if (HAL_GetTick() - modeTick >= 1000 && isModeSelected) {
            SystemMode mode = Mode_GetCurrent();

           if (mode == MODE_VIEW_TEMP) {
                float temp = read_temperature();
                update_Temperature(temp);
                float t_avg = get_AvgTemperature();
                //printf("Tavg = %.2f\n", t_avg);

                if (t_avg > COMFORT_TEMP) {
                    LEDMAT_ShowFace(FACE_SAD);
                } else if (t_avg <= CHILL_TEMP){
                    LEDMAT_ShowFace(FACE_HAPPY);
                } else{
                	LEDMAT_ShowFace(FACE_NEUTRAL);
                }
            }else if (mode == MODE_VIEW_HUMID){
            	float humid = read_humidity();
            	update_Humidity(humid);
            	float h_avg = get_AvgHumidity();
            	//printf("Havg = %.2f\n", h_avg);

            	if (h_avg > COMFORT_HUMID){
            		LEDMAT_ShowFace(FACE_SAD);
            	} else if (h_avg <= LOW_HUMID){
            		LEDMAT_ShowFace(FACE_HAPPY);
            	} else {
            		LEDMAT_ShowFace(FACE_NEUTRAL);
            	}
            }

            if (mode == MODE_VIEW_TEMP || mode == MODE_VIEW_HUMID) {
                Mode_Handle(mode);
            }
        }

    }
}
static void UART1_Init()
{
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically
		*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while(1);
	}
}

static void GPIO_Init(void) {
    /* Enable GPIOA and GPIOB peripheral clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    /* PA0 as analog mode (ADC input) */
    GPIOA->MODER &= ~(3U << (0 * 2));   // clear mode bits for PA0
    GPIOA->MODER |=  (3U << (0 * 2));   // set mode to analog (11)
    GPIOA->PUPDR &= ~(3U << (0 * 2));   // no pull-up, no pull-down
    /* PB14 as general-purpose output (LED) */
    GPIOB->MODER &= ~(3U << (14 * 2));  // clear mode bits for PB14
    GPIOB->MODER |=  (1U << (14 * 2));  // set mode to output (01)
    GPIOB->OTYPER &= ~(1U << 14);       // push-pull output
    GPIOB->OSPEEDR &= ~(3U << (14 * 2)); // low speed output
    GPIOB->PUPDR &= ~(3U << (14 * 2));   // no pull-up, no pull-down
    /* Ensure LED is off initially */
    GPIOB->ODR &= ~(1U << 14);
}

/* Initialize ADC1 on PA0 (ADC channel 5) in interrupt mode */
static void ADC_Init(void) {
	/* Enable ADC clock */
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	/* Select system clock as ADC clock source (ADCSEL bits = 11) */
	RCC->CCIPR &= ~RCC_CCIPR_ADCSEL;
	RCC->CCIPR |= (3U << RCC_CCIPR_ADCSEL_Pos);
	/* ADC1 configuration */
	if (ADC1->CR & ADC_CR_ADEN) {
		ADC1->CR |= ADC_CR_ADDIS;            // Disable ADC if already enabled
	}
	ADC1->CR &= ~ADC_CR_DEEPPWD;             // Exit deep-power-down mode (enable ADC clock domain)
	ADC1->CR |= ADC_CR_ADVREGEN;             // Enable ADC internal voltage regulator
	/* Small delay to stabilize the regulator */
	for (volatile uint32_t i = 0; i < 1000; i++) { __NOP(); }
	/* Calibrate ADC (single-ended mode) */
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL) { /* wait for calibration to complete */ }
	/* Configure ADC regular channel sequence length = 1 */
	ADC1->SQR1 &= ~ADC_SQR1_L;               // L[3:0] = 0 (sequence length = 1)
	/* Select ADC channel 5 (PA0) for first conversion in sequence */
	ADC1->SQR1 &= ~(0x1F << 6);              // clear SQ1 bits [10:6]
	ADC1->SQR1 |= (5U << 6);                 // set SQ1 = 5 (ADC channel 5)
	/* Set ADC sampling time for channel 5 */
	ADC1->SMPR1 &= ~(0x7 << (5 * 3));        // clear SMP5 bits
	ADC1->SMPR1 |=  (0x7 << (5 * 3));        // set SMP5 = 0b111 (maximum sampling time)
	/* Configure ADC in continuous conversion mode */
	//ADC1->CFGR |= ADC_CFGR_CONT;
	/* Enable end-of-conversion interrupt */
	//ADC1->IER |= ADC_IER_EOCIE;
	/* Enable ADC interrupt in NVIC */
	NVIC_EnableIRQ(ADC1_2_IRQn);
	/* Enable the ADC */
	ADC1->ISR |= ADC_ISR_ADRDY;             // clear ADRDY flag
	ADC1->CR |= ADC_CR_ADEN;                // enable ADC
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) { /* wait until ADC is ready */ }
}

static void MX_GPIO_InitLED(void) {
		__HAL_RCC_GPIOA_CLK_ENABLE();	// Enable AHB2 Bus for GPIOA

		GPIO_InitTypeDef GPIO_InitStruct1 = {0};

		//For LED1
		HAL_GPIO_WritePin(GPIOA, ARD_D13_Pin, GPIO_PIN_RESET);
		GPIO_InitStruct1.Pin = ARD_D13_Pin;
		GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct1.Pull = GPIO_NOPULL;
		GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);


		//For Buzzer
		GPIO_InitStruct1.Pin = GPIO_PIN_1;
		GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct1.Pull = GPIO_NOPULL;
		GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);
}


void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

void MX_I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        //printf("I2C init failed\n");
        while (1);
    }

    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

static void accelInt(void) {

	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructFall = {0};

	GPIO_InitStructFall.Pin = GPIO_PIN_11;
	GPIO_InitStructFall.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructFall.Pull = GPIO_NOPULL;
	GPIO_InitStructFall.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructFall);


	HAL_NVIC_SetPriority((IRQn_Type)(GPIO_PIN_11), 0x00, 0x00); //Sets it up for high priority
	HAL_NVIC_EnableIRQ((IRQn_Type)(GPIO_PIN_11));

}

//Writes the the accelerometer to configure the free fall detection
static void enableFree(void){
	SENSOR_IO_Write(0xD4, 0x10, 0x20); //turns on the accelerometer
	SENSOR_IO_Write(0xD4, 0x58, 0x80); //enables interrupts
	SENSOR_IO_Write(0xD4, 0x5C, 0x00); //Sets the duration of the free fall event
	SENSOR_IO_Write(0xD4, 0x5D, 0x57); //Controls the sensitivity of free fall
	SENSOR_IO_Write(0xD4, 0x5E, 0x10); //enables free fall in the register
}

