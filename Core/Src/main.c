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
#include "stm32f429i_discovery_lcd.h"

/* Private variables ---------------------------------------------------------*/
typedef struct {
    char name[20];      // Nazwa programu
    uint16_t time;      // Czas trwania
    uint16_t temperature; // Temperatura
} WashingProgram;

WashingProgram programs[] = {
    {"Czyszczenie", 30, 60},
    {"Plukanie", 10, 30},
    {"Szybkie pranie", 15, 40},
    {"Delikatne", 40, 30},
    {"Bawelna", 60, 60}
};
#define NUM_PROGRAMS (sizeof(programs) / sizeof(programs[0]))


int current_program = 0;  // Indeks aktualnego programu

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
void displayProgram(void); // Function to display the current program

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();

  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_Clear(LCD_COLOR_RED);
  display_program(programs[current_program]);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
      if (HAL_GPIO_ReadPin(GPIOA, NextBut_Pin) == GPIO_PIN_SET) {
          // Przejdź do następnego programu
          current_program = (current_program + 1) % NUM_PROGRAMS;
          display_program(programs[current_program]);
          HAL_Delay(200);
      }

      if (HAL_GPIO_ReadPin(GPIOA, LastBut_Pin) == GPIO_PIN_SET) {
          // Przejdź do poprzedniego programu
          current_program = (current_program - 1 + NUM_PROGRAMS) % NUM_PROGRAMS;
          display_program(programs[current_program]);
          HAL_Delay(200);
      }

      HAL_Delay(100);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* Function to display the current selected program */
void display_program(WashingProgram program) {
      char buffer[40];
      sprintf(buffer, "Prog: %s", program.name);
      BSP_LCD_DisplayStringAtLine(0, (uint8_t*)buffer);

      sprintf(buffer, "Czas: %d min", program.time);
      BSP_LCD_DisplayStringAtLine(1, (uint8_t*)buffer);

      sprintf(buffer, "Temp: %d C", program.temperature);
      BSP_LCD_DisplayStringAtLine(2, (uint8_t*)buffer);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Other necessary initialization functions */

static void MX_DMA2D_Init(void) { /* Code for DMA2D initialization */ }
static void MX_I2C3_Init(void) { /* Code for I2C3 initialization */ }
static void MX_LTDC_Init(void) { /* Code for LTDC initialization */ }
static void MX_SPI5_Init(void) { /* Code for SPI5 initialization */ }
static void MX_FMC_Init(void) { /* Code for FMC initialization */ }
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : NextBut_Pin LastBut_Pin */
  GPIO_InitStruct.Pin = NextBut_Pin | LastBut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */
