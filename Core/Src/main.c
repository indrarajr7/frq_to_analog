/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "disp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_CONV_FACTOR		((float)4096/20000)
#define CLK_FREQ			(48000000UL)
#define DEBOUNCE_TIME		(200)
#define LONG_PRESS_TIME		(3000)

#define MODE_DISP			0
#define MODE_PROG			1
#define MODE_SET			2
#define SP_COUNT			3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SW1_IN()			(HAL_GPIO_ReadPin(SW_1_IN_GPIO_Port, SW_1_IN_Pin))
#define SW2_IN()			(HAL_GPIO_ReadPin(SW_2_IN_GPIO_Port, SW_2_IN_Pin))
#define SW3_IN()			(HAL_GPIO_ReadPin(SW_3_IN_GPIO_Port, SW_3_IN_Pin))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t sw1_flag, sw2_flag, sw3_flag;
uint16_t sw1_timer, sw2_timer, sw3_timer;
void sw_routine() {
	sw1_timer = sw1_flag ? sw1_timer + 1 : 0;
	sw2_timer = sw2_flag ? sw2_timer + 1 : 0;
	sw3_timer = sw3_flag ? sw3_timer + 1 : 0;
}
uint8_t sw1_read(uint8_t is_long) {
	if(SW1_IN()) {
		sw1_flag = 1;
		if(sw1_timer > \
				(is_long ? LONG_PRESS_TIME : DEBOUNCE_TIME)) {
			return 1;
		}
	}
	else sw1_flag = 0;
	return 0;
}
uint8_t sw2_read(uint8_t is_long) {
	if(SW2_IN()) {
		sw2_flag = 1;
		if(sw2_timer > \
				(is_long ? LONG_PRESS_TIME : DEBOUNCE_TIME)) {
			return 1;
		}
	}
	else sw2_flag = 0;
	return 0;
}
uint8_t sw3_read(uint8_t is_long) {
	if(SW3_IN()) {
		sw3_flag = 1;
		if(sw3_timer > \
				(is_long ? LONG_PRESS_TIME : DEBOUNCE_TIME)) {
			return 1;
		}
	}
	else sw3_flag = 0;
	return 0;
}

uint8_t mode; /** 0: display, 1: prog, 2: set */
uint8_t prg_idx; /** index for programming params */

typedef enum {
	PULSES_PER_REVOLUTION,
	RANGE,
	REFRESH_RATE
} sp_id;

typedef struct {
	uint8_t idx;
	char name[10];
	uint16_t def;
	uint8_t step;
	uint16_t min;
	uint16_t max;
	uint16_t val;
} sp;

sp sp_arr[SP_COUNT] = {{
		.idx = PULSES_PER_REVOLUTION,
		.def = 1,
		.step = 1,
		.min = 1,
		.max = 5,
		.val = 1
	}, {
		.idx = RANGE,
		.def = 10000,
		.step = 10,
		.min = 10,
		.max = 20000,
		.val = 10000
	}, {
		.idx = REFRESH_RATE,
		.def = 5,
		.step = 1,
		.min = 1,
		.max = 20000,
		.val = 5
	}
};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t pps, rpm;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	static uint16_t c1 = 0, c2 = 0, intvl = 0;
	static uint8_t c1_flag;
	if(htim->Instance == TIM1) {
		if(c1_flag) {
			c2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); /* Rising edge 2 */
			/* Calculate time between consecutive pulses */
			if(c2 >= c1) {
				intvl = c2 - c1;
			}
			else {
				intvl = c2 + (65536 - c1);
			}
			pps = (1 / (float)(intvl * (htim->Instance->PSC / (float)CLK_FREQ)));
			rpm = pps * 60;
			c1_flag = 0;
		}
		else {
			c1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); /* Rising edge 1 */
			c1_flag = 1;
		}
	}
}
/* ms timer */
uint32_t sec;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	static uint8_t disp_idx = 0;
	if(htim->Instance == TIM16) {
		if(sec != 0 && sec % 4 == 0) { /* every 4 ms */
		// Do stuff
			display_value(disp_idx + 1, char_idxs[disp_idx]);
			disp_idx = (disp_idx == 5) ? 1 - 1 : disp_idx + 1;
		}
	}
	/* TODO ppm, rps clear somewhere else */
	if(sec % 100 == 0) pps = 0, rpm = 0; /* change in other func */
	sec = (sec >= 1000) ? 0 : sec + 1;
	sw_routine();

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
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  mode = 0;
  uint8_t sp_idx = 0;
  sp *curr;
  curr = &sp_arr[sp_idx]; /* TODO load from eeprom here */

  uint8_t sp_idx_chgflag = 1, sp_val_chgflag = 1;
  while (1)
  {
	  switch(mode) {
	  case MODE_DISP:
		  if(sw1_read(1))
			 mode = MODE_PROG;
		  /* TODO dipslay rpm */
		  /* TODO update rpm based on refresh rate */
		  break;
	  case MODE_PROG:
		  /* update curr from eeprom */
		  if(sp_idx_chgflag) {
			  curr = &sp_arr[sp_idx];
//			  disp_text(&curr->name, strlen(curr->name)); /* TODO fn to be created */
			  disp_no(curr->idx + 10);
			  sp_idx_chgflag = 0;
		  }
		  if(sw1_read(0))
			 mode = MODE_SET;
		  if(sw2_read(0)) { /* up button */
			 sp_idx = (sp_idx == SP_COUNT - 1) ? 0 : sp_idx + 1;
			 sp_idx_chgflag = 1;
		  }
		  if(sw3_read(0)) {/* down button */
			 sp_idx = (sp_idx == 0) ? SP_COUNT - 1 : sp_idx - 1;
			 sp_idx_chgflag = 1;
		  }
		  break;
	  case MODE_SET:
		  /* TODO display value */
		  if(sp_val_chgflag) {
			  disp_no(curr->val);
			  sp_val_chgflag = 0;
		  }
		  if(sw1_read(0)) {
			  /* on press, write val to eeprom */
//			  sp_save(sp_idx, curr->val); /* TODO implement sp_save */
			  sp_idx_chgflag = 1;
			  mode = MODE_PROG;
		  }
		  if(sw2_read(0)) { /* up button */
			 curr->val = curr->val >= curr->max ? \
					 curr->min : curr->val + curr->step;
			 sp_val_chgflag = 1;
		  }
		  if(sw3_read(0)) { /* down button */
			 curr->val = curr->val <= curr->min ? \
					 curr->max : curr->val - curr->step;
			 sp_val_chgflag = 1;
		  }
		  break;
	  default:
		  mode = MODE_DISP;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, DISP_A_Pin|DISP_B_Pin|DISP_C_Pin|DISP_D_Pin
                          |DISP_E_Pin|DISP_F_Pin|DISP_G_Pin|DISP_DP_Pin
                          |DISP_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISP_5_Pin|DISP_4_Pin|DISP_3_Pin|DISP_2_Pin
                          |DISP_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISP_A_Pin DISP_B_Pin DISP_C_Pin DISP_D_Pin
                           DISP_E_Pin DISP_F_Pin DISP_G_Pin DISP_DP_Pin
                           DISP_6_Pin */
  GPIO_InitStruct.Pin = DISP_A_Pin|DISP_B_Pin|DISP_C_Pin|DISP_D_Pin
                          |DISP_E_Pin|DISP_F_Pin|DISP_G_Pin|DISP_DP_Pin
                          |DISP_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_2_IN_Pin SW_1_IN_Pin */
  GPIO_InitStruct.Pin = SW_2_IN_Pin|SW_1_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_3_IN_Pin */
  GPIO_InitStruct.Pin = SW_3_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_3_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_5_Pin DISP_4_Pin DISP_3_Pin DISP_2_Pin
                           DISP_1_Pin */
  GPIO_InitStruct.Pin = DISP_5_Pin|DISP_4_Pin|DISP_3_Pin|DISP_2_Pin
                          |DISP_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
