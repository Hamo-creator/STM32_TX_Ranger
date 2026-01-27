/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include "crsf.h"
#include "config.h"
#include "ST7735.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern UART_HandleTypeDef huart1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAP_SCALE_NUM   (CRSF_DIGITAL_CHANNEL_MAX - CRSF_DIGITAL_CHANNEL_MIN)
#define MAP_SCALE_DEN   (ADC_MAX - ADC_MIN)
#define MAP_OFFSET      (CRSF_DIGITAL_CHANNEL_MIN)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Global CRSF serial instance
CrsfSerial_HandleTypeDef hcrsf;

#define UART_RX_BUFFER_SIZE 128	// 64
uint8_t  uartRxBuf[UART_RX_BUFFER_SIZE];
uint16_t oldPos = 0;
// static uint16_t oldPos = 0;
uint16_t ADC_BUF[6];

#define ADC_MAX_VALUE      4095.0f

// Battery voltage
#define R1 33400.0f   // ohms
#define R2 20000.0f   // ohms
#define VREF               3.3f
float batteryVoltage;
const float warningVoltage = 7.8;
const float dangerVoltage =  7.4;

// CRSF
int Aileron_value = 0; // values read from the pot
int Elevator_value = 0;
int Flabs_value = 0;
int Rudder_value = 0;
int Throttle_value = 0;
int previous_throttle = 191;

// switch values read from the digital pin
int AUX1_Arm = 0;		//PB5
int AUX2_value = 0;		//PB4

#define SMA_WINDOW_SIZE 8  // You can choose 4, 8, 16, 32...

uint16_t Aileron_samples[SMA_WINDOW_SIZE] = {0};
uint16_t Elevator_samples[SMA_WINDOW_SIZE] = {0};
uint16_t Flabs_samples[SMA_WINDOW_SIZE] = {0};
uint16_t Rudder_samples[SMA_WINDOW_SIZE] = {0};
uint16_t Throttle_samples[SMA_WINDOW_SIZE] = {0};

uint8_t sample_index = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
uint16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;
uint32_t overFlowTime = 0x2000000;

uint8_t sendStatus_1;
uint8_t sendStatus_2;

int loopCount = 0;
int currentPktRate = 0;
int currentPower = 0;
int currentDynamic = 0;
int currentSetting = 0;
int stickMoved = 0;
int stickInt = 0;

uint32_t stickMovedMillis = 0;
uint32_t currentMillis = 0;
uint32_t loopStarttime = 0;
uint32_t loopEndtime = 0;

uint32_t BatShowCount = 0;
uint8_t receive_status = 6;
bool int_entered = false;

bool calStatus=false;


CRSF_State currentState = STATE_STARTUP;
uint8_t stateRepeatCount = 0;

GPIO_PinState AUX_1 = GPIO_PIN_RESET;	//PB5
GPIO_PinState AUX_2 = GPIO_PIN_RESET;	//PB4

// --- CRSF Scheduler Timing ---
uint32_t next_rc_packet_time_us = 0;
uint32_t packet_counter = 0;
const uint32_t RC_PACKETS_PER_POLL = 2; // Send 1 poll for every 8 RC packets

// Struct instance to read the struct element
crsf_link_statistics_t LinkStats;
int16_t display_rssi = 0;
uint8_t display_lq = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void selectSetting() {
    // startup stick commands (rate/power selection / initiate bind / turn on tx module wifi)
    // Right stick:
    // Up Left - Rate/Power setting 1 (250hz / 100mw / Dynamic)
    // Up Right - Rate/Power setting 2 (50hz / 100mw)
    // Down Left - Start TX bind (for 3.4.2 it is now possible to bind to RX easily). Power cycle after binding
    // Down Right - Start TX module wifi (for firmware update etc)

    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron left
        currentPktRate = SETTING_1_PktRate;
        currentPower = SETTING_1_Power;
        currentDynamic = SETTING_1_Dynamic;
        currentSetting = 1;
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron right
        currentPktRate = SETTING_2_PktRate;
        currentPower = SETTING_2_Power;
        currentDynamic = SETTING_2_Dynamic;
        currentSetting = 2;
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron left
        currentSetting = 3;  // Bind
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron right
        currentSetting = 4;  // TX Wifi
    } else {
        currentSetting = 0;
    }
}

bool checkStickMove(){
    // check if stick moved, warring after 10 minutes
    if(abs(previous_throttle - rcChannels[THROTTLE]) < 30){
        stickMoved = 0;
    }else{
        previous_throttle = rcChannels[THROTTLE];
        stickMovedMillis = HAL_GetTick();
        stickMoved = 1;
    }

    if (HAL_GetTick() - stickMovedMillis > STICK_ALARM_TIME){
        return true;
    }else{
        return false;
    }
}

static inline uint16_t fastMap4095(uint16_t x, uint16_t out_min, uint16_t out_max)
{
    return (uint16_t)(((uint32_t)x * (out_max - out_min)) / 4095 + out_min);
}

void DWT_Init(void) {
    // Enable the trace unit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Unlock the DWT (necessary on some F4/F7 series)
    DWT->LAR = 0xC5ACCE55; 
    // Reset the cycle counter
    DWT->CYCCNT = 0;
    // Start the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void) {
    // CYCCNT increments every CPU cycle. 
    // For 100MHz, SystemCoreClock / 1000000 = 100.
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

/*
uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}
*/

float Get_BatteryVoltage(void)
{
	uint16_t adc_value = ADC_BUF[5];
    // Step 1: Convert ADC reading to measured divider voltage
    float vout = (adc_value * VREF) / ADC_MAX_VALUE;

    // Step 2: Calculate battery voltage
    float vbat = vout * ((R1 + R2) / R2);

    return vbat;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Init(); // Initialize the cycle counter early

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

	/*
  // Enable DWT Cycle Counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to DWT
  DWT->CYCCNT = 0;                                // Reset the counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable the cycle counter
  */
	
  if (huart1.Instance == NULL) {
      // Error: UART not initialized
      Error_Handler();
  }

  // Strating ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_BUF, 6);

  // inialize rc data
  for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
      rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
  }

  // Initialize ST7735 display
  ST7735_Init(1);

  // Clear screen
  ST7735_FillScreen(BLACK);

  // Display welcome message
  ST7735_WriteString(35, 30, "Welcome", Font_11x18, WHITE, BLACK);
  ST7735_WriteString(60, 50, "to", Font_11x18, CYAN, BLACK);
  ST7735_WriteString(38, 70, "DIY RC", Font_11x18, YELLOW, BLACK);

  HAL_Delay(1000);
  // Clear screen
  ST7735_FillScreen(BLACK);
	
  // Start CRSF UART reception (for telemetry from FC)
  // For half-duplex, ensure the pin is in RX mode initially
  CRSF_SetRxMode();
  // Enable IDLE line interrupt for packet detection
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  // HAL_UART_Receive_DMA(hcrsf.huart, rx_dma_buffer, rx_dma_buffer_size);
  HAL_UART_Receive_DMA(&huart1, uartRxBuf, UART_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartRxBuf, UART_RX_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // ====================================================================
    //  PHASE 1: READ AND PROCESS ALL INPUTS
    // ====================================================================
	uint32_t currentMicros = micros();
//	loopStarttime = micros();

	// Handle micros() overflow
	if (currentMicros < crsfTime - CRSF_TIME_BETWEEN_FRAMES_US) {
	    // We've had an overflow, reset the timing
	    crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
	}

	//CRSF_SetRxMode();

	// Read Switches
	AUX_1 = HAL_GPIO_ReadPin(GPIOB, SW_AUX1_Pin);
	AUX_2 = HAL_GPIO_ReadPin(GPIOB, SW_AUX2_Pin);

	// Read and average analog sticks (your existing code is good)
	Aileron_samples[sample_index]  = ADC_BUF[1];
	Elevator_samples[sample_index] = ADC_BUF[0];
	Flabs_samples[sample_index]    = ADC_BUF[3];
	Rudder_samples[sample_index]   = ADC_BUF[4];
	Throttle_samples[sample_index] = ADC_BUF[2];

	sample_index = (sample_index + 1) % SMA_WINDOW_SIZE;

	// Now, calculate the average for each stick
	uint32_t aileron_sum = 0, elevator_sum = 0, flabs_sum = 0, rudder_sum = 0, throttle_sum = 0;

	for (uint8_t i = 0; i < SMA_WINDOW_SIZE; i++) {
		aileron_sum  += Aileron_samples[i];
		elevator_sum += Elevator_samples[i];
		flabs_sum += Flabs_samples[i];
		rudder_sum   += Rudder_samples[i];
		throttle_sum += Throttle_samples[i];
	}

	Aileron_value  = aileron_sum / SMA_WINDOW_SIZE;
	Elevator_value = elevator_sum / SMA_WINDOW_SIZE;
	Flabs_value 	 = flabs_sum / SMA_WINDOW_SIZE;
	Rudder_value   = rudder_sum / SMA_WINDOW_SIZE;
	Throttle_value = throttle_sum / SMA_WINDOW_SIZE;

	// After that, continue mapping like before (0-4095 to your range)
	Aileron_value  = (int32_t)Aileron_value * (ADC_MAX - ADC_MIN) / 4095 + ADC_MIN;
	Elevator_value = (int32_t)Elevator_value * (ADC_MAX - ADC_MIN) / 4095 + ADC_MIN;
	Flabs_value	 = (int32_t)Flabs_value * (ADC_MAX - ADC_MIN) / 4095 + ADC_MIN;
	Rudder_value   = (int32_t)Rudder_value * (ADC_MAX - ADC_MIN) / 4095 + ADC_MIN;
	Throttle_value = (int32_t)Throttle_value * (ADC_MAX - ADC_MIN) / 4095 + ADC_MIN;

	rcChannels[AILERON]   = fastMap4095(Aileron_value,  CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
	rcChannels[ELEVATOR]  = fastMap4095(Elevator_value,  CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
	rcChannels[FLABS] 	= fastMap4095(Flabs_value,  CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
	rcChannels[RUDDER]    = fastMap4095(Rudder_value,  CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
	rcChannels[THROTTLE]  = fastMap4095(Throttle_value,  CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);

	// Map the GPIO state to CRSF channel values
	rcChannels[AUX1] = (AUX_1 == GPIO_PIN_SET) ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;
	rcChannels[AUX2] = (AUX_2 == GPIO_PIN_SET) ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;

	// Handle LCD display updates (this can be slow, so it's good to limit its frequency)
	if (BatShowCount++ > 400000) {
		batteryVoltage = Get_BatteryVoltage();
		ST7735_WriteString(70, 10, "RC BAT:", Font_7x10, BLUE, BLACK);
		if (batteryVoltage > warningVoltage) {
			ST7735_WriteFloat(120, 10, batteryVoltage, 2,Font_7x10, GREEN, BLACK);
		} else if (batteryVoltage <= warningVoltage && batteryVoltage > dangerVoltage) {
			ST7735_WriteFloat(120, 10, batteryVoltage, 2,Font_7x10, YELLOW, BLACK);
		} else if (batteryVoltage < dangerVoltage) {
			ST7735_WriteFloat(120, 10, batteryVoltage, 2,Font_7x10, RED, BLACK);
		}
		BatShowCount = 0;
	}


	  if ((uint32_t)currentMicros >= crsfTime){
		  //CRSF_SetTxMode();
		  if(loopCount <= 500){	//500
			  CRSF_PrepareDataPacket(crsfPacket, rcChannels);

			  sendStatus_1 = CRSF_WritePacket(crsfPacket, CRSF_PACKET_SIZE);
			  loopCount++;
		  }

		  else if (loopCount > 500 && loopCount <= 505) { // repeat 5 packets to avoid bad packet, change rate setting
			  // Build commond packet
			  if (currentSetting == 1 || currentSetting == 2) {
				  CRSF_PrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);
				  CRSF_WritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
			  } else if (currentSetting == 3) {
				  CRSF_PrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
				  CRSF_WritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
			  } else if (currentSetting == 4) {
				  CRSF_PrepareCmdPacket(crsfCmdPacket, ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
				  CRSF_WritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
			  }
			  loopCount++;
		  }
//		  else if (loopCount > 505 && loopCount <= 510) { // repeat 5 packets to avoid bad packet, change TX power level
//			  if (currentSetting == 1 || currentSetting == 2) {
//				  //CRSF_PrepareChannelsPacket_Correct(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
//				  CRSF_PrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
//				  //HAL_GPIO_WritePin(TX_RX_EN_GPIO_Port, TX_RX_EN_Pin, GPIO_PIN_RESET);
//				  CRSF_WritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
//				  //HAL_GPIO_WritePin(TX_RX_EN_GPIO_Port, TX_RX_EN_Pin, GPIO_PIN_SET);
//			  }
//			  loopCount++;
//		  } else if (loopCount > 510 && loopCount <= 515) { // repeat 5 packets to avoid bad packet, change TX dynamic power setting
//			  if (currentSetting == 1 || currentSetting == 2) {
//				  //CRSF_PrepareChannelsPacket_Correct(crsfCmdPacket, ELRS_DYNAMIC_POWER_COMMAND, currentDynamic);
//				  CRSF_PrepareCmdPacket(crsfCmdPacket, ELRS_DYNAMIC_POWER_COMMAND, currentDynamic);
//				  //HAL_GPIO_WritePin(TX_RX_EN_GPIO_Port, TX_RX_EN_Pin, GPIO_PIN_RESET);
//				  CRSF_WritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
//				  //HAL_GPIO_WritePin(TX_RX_EN_GPIO_Port, TX_RX_EN_Pin, GPIO_PIN_SET);
//			  }
//			  loopCount++;
//		  }
//		  else if (packet_counter >= RC_PACKETS_PER_POLL) {
// 			  CRSF_SetRxMode();
////			  if (Crsf_SendTelemetryPoll(&hcrsf) == HAL_OK) {
//				  packet_counter = 0; // Reset the counter
////				  //next_rc_packet_time_us = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
////			  }
//		  }

		  /*
		  else {
			  if ((micros() - currentMicros) >= CRSF_TIME_BETWEEN_FRAMES_US) {
				  loopStarttime = micros();
				  //CRSF_SetTxMode();
				  CRSF_PrepareDataPacket(crsfPacket, rcChannels);
				  sendStatus_2 = CRSF_WritePacket(crsfPacket, CRSF_PACKET_SIZE);
				  if (sendStatus_2 == HAL_OK) {
					  packet_counter++; // Increment the counter towards the next poll
				  }
				  loopEndtime = (micros() - loopStarttime);
			  }
		  }
		  */

		  else {
			  loopStarttime = micros();
			  //CRSF_SetTxMode();
			  CRSF_PrepareDataPacket(crsfPacket, rcChannels);
			  sendStatus_2 = CRSF_WritePacket(crsfPacket, CRSF_PACKET_SIZE);
			  if (sendStatus_2 == HAL_OK) {
				  packet_counter++; // Increment the counter towards the next poll
			  }
			  loopEndtime = (micros() - loopStarttime);
		  }
		  
		  /* Recovery Logic: If UART crashed due to noise/collision */
		  if (hcrsf.uart_error_occurred) {
			  hcrsf.uart_error_occurred = false;
			  
			  // Stop and restart DMA to clear the internal HAL state
			  HAL_UART_DMAStop(&huart1);
			  CRSF_SetRXMode();
			  HAL_UART_ReceiveDMA(&huart1, uartRxBuf, UART_RX_BUFFER_SIZE);
		  }
		  
		  // Handle Telemetry Reception out of the ISR
		  if (hcrsf.idlecallback) {
			  hcrsf.idelcallback = false;
			  CrsfSerial_UART_IdleCallback(&hcrsf);
		  }

	      crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
	  }
//	  loopEndtime = (micros() - loopStarttime);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

