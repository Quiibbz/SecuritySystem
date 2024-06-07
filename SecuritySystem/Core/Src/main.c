/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint8_t button1Pressed = 0;
volatile uint8_t button2Pressed = 0;
volatile uint32_t lastButtonPressTime = 0;
volatile uint16_t rawValue;
volatile char value;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
	IDLE_State,
	ARMED_State,
	TRIGGERED_State

} eSystemState;

/* Prototype Event Handlers */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t currentTime = HAL_GetTick();
	if(GPIO_Pin == BUTTON1_Pin && currentTime - lastButtonPressTime > 50) {
		button1Pressed = 1;
		lastButtonPressTime = HAL_GetTick();
	} else if(GPIO_Pin == BUTTON2_Pin && currentTime - lastButtonPressTime > 50) {
		button2Pressed = 1;
		lastButtonPressTime = HAL_GetTick();
	}
}

uint8_t PassCode_Handler() {
	uint32_t TIMEOUT_TIMER = 5000;
	uint8_t PASSWORD_LENGTH = 4;
	int password[4] = {1,1,1,1};
	volatile uint8_t passwordIndex = 0;
	button1Pressed = 0;
	button2Pressed = 0;
	lastButtonPressTime = HAL_GetTick();

	while(1) {
		//Correct Password Entered
		if(passwordIndex == PASSWORD_LENGTH) {
			HAL_GPIO_WritePin(GPIOA, ARMED_GREEN_LED_Pin, GPIO_PIN_SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOA, ARMED_GREEN_LED_Pin, GPIO_PIN_RESET);
			value = 'P';
			UART_Handler(value);
			button1Pressed = 0;
			button2Pressed = 0;
			return 1;
		}
		//If 5 seconds elapses since last button input, leaves the passcode handler
		if(HAL_GetTick() - lastButtonPressTime >= TIMEOUT_TIMER) {
			button1Pressed = 0;
			button2Pressed = 0;
			return 2;
		}
		//Checks if button1 was the correct choice
		if(button1Pressed) {
			if(passwordIndex < PASSWORD_LENGTH && password[passwordIndex] == 1) {
				passwordIndex++;
				button1Pressed = 0;
				//Incorrect
			} else {
				HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_RESET);
				HAL_Delay(500);
				value = 'p';
				UART_Handler(value);
				button1Pressed = 0;
				return 0;
			}
			//Checks if button2 was the correct choice
		}else if(button2Pressed) {
			if(passwordIndex < PASSWORD_LENGTH && password[passwordIndex] == 0) {
				passwordIndex++;
				button2Pressed = 0;
				//Incorrect
			} else {
				HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_RESET);
				HAL_Delay(500);
				value = 'p';
				UART_Handler(value);
				button2Pressed = 0;
				return 0;
			}
		}
	}
}

void UART_Handler(char value) {
	char data[128];
	uint8_t time = HAL_GetTick()/1000;
	switch(value) {
	case 'p': {
		sprintf(data, "%d : Password Entered Incorrectly\r\n", time);
		HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		break;
	}
	case 'P': {
		sprintf(data, "%d : Password Entered Correctly\r\n", time);
		HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		break;
	}
	case 'I': {
		sprintf(data, "%d : Security System Unarmed\r\n", time);
		HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		break;
	}
	case 'A': {
		sprintf(data, "%d : Security System Armed\r\n", time);
		HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		break;
	}
	case 'M': {
		sprintf(data, "%d : Motion Detected!\r\n", time);
		HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		break;
	}
	case 'L': {
		sprintf(data, "%d : Light Change Detected!\r\n", time);
		HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		break;
	}
	default: break;
	}
	return;
}

uint8_t Sensor_Handler() {
    HAL_ADC_Init(&hadc1);
    HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	rawValue = HAL_ADC_GetValue(&hadc1);

	while(1) {
		//If rawValue = 2482 (Voltage = 2V) then trigger Security Alarm
		if(rawValue >= 100) {
			value = 'L';
			return 1;
		}

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)) {
			value = 'M';
			return 1;
		}

		else {
			return 0;
		}
	}
	return 0;
}

eSystemState IDLE_Handler(void)
{

	HAL_GPIO_WritePin(GPIOA, ARMED_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IDLE_YELLOW_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

	value = 'I';
	UART_Handler(value);

	while(1) {
		if(PassCode_Handler() == 1) {
			return ARMED_State;
		}
	}
}

eSystemState ARMED_Handler(void) {
	//So owner has time to leave room before alarm is set
	int i;
	for(i = 0; i < 3; i= i + 1) {
		HAL_GPIO_WritePin(GPIOA, ARMED_GREEN_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, ARMED_GREEN_LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(500);
	}
	HAL_GPIO_WritePin(GPIOB, IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ARMED_GREEN_LED_Pin, GPIO_PIN_SET);

	value = 'A';
	UART_Handler(value);

	while(1) {
		if(Sensor_Handler() == 1) {
			return TRIGGERED_State;
		}
	}
}

eSystemState TRIGGERED_Handler(void)
{
	//Visually depicts current state
	HAL_GPIO_WritePin(GPIOB, IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ARMED_GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_SET);
	//Alarm Buzzer Triggers
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

	UART_Handler(value);

	while(1) {
		HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin, GPIO_PIN_SET);
		//Input Password to disarm
		if(PassCode_Handler() == 1)
		{
			return IDLE_State;
		}
	}
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
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* Declare eNextState and initialize it to IDLE_State */
    eSystemState eNextState = IDLE_State;

    HAL_ADC_Init(&hadc1);
    HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	switch(eNextState)
    	{
    		case IDLE_State:
    			eNextState = IDLE_Handler();
    	  	break;

    	  	case ARMED_State:
    	  		eNextState = ARMED_Handler();
    	  	break;

    	  	case TRIGGERED_State:
    	  		eNextState = TRIGGERED_Handler();
    	  	break;

    	  	default:
    	  		eNextState = IDLE_Handler();
    	  	break;
    	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZER_OUPUT_Pin|ARMED_GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIGGERED_RED_LED_Pin|IDLE_YELLOW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_OUPUT_Pin ARMED_GREEN_LED_Pin */
  GPIO_InitStruct.Pin = BUZZER_OUPUT_Pin|ARMED_GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON1_Pin BUTTON2_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin|BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTION_OUTPUT_Pin */
  GPIO_InitStruct.Pin = MOTION_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTION_OUTPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIGGERED_RED_LED_Pin IDLE_YELLOW_LED_Pin */
  GPIO_InitStruct.Pin = TRIGGERED_RED_LED_Pin|IDLE_YELLOW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
