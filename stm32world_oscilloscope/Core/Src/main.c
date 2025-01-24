/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 Lars Boegild Thomsen <lth@stm32world.com>
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
#include <stdio.h>

#include "ssd1306.h"
#include "image.h"
//#include "ssd1306_tests.h"
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

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern unsigned char garfield_128x64[];
extern unsigned char github_logo_64x64[];
extern uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send printf to uart1
int _write(int fd, char *ptr, int len) {
    HAL_StatusTypeDef hstatus;

    if (fd == 1 || fd == 2) {
        hstatus = HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
        if (hstatus == HAL_OK)
            return len;
        else
            return -1;
    }
    return -1;
}

// Shift whole screen left 1 pixel
void ssd1306_ShiftLeft() {
    uint8_t pages = SSD1306_HEIGHT / 8;

    for (uint8_t page = 0; page < pages; ++page) {
        for (uint8_t column = 0; column < SSD1306_WIDTH - 1; ++column) {
            SSD1306_Buffer[page * SSD1306_WIDTH + column] = SSD1306_Buffer[page * SSD1306_WIDTH + column + 1];
        }
        SSD1306_Buffer[page * SSD1306_WIDTH + SSD1306_WIDTH - 1] = 0;
    }

}

void drawLines()
{
    for (int16_t i = 0; i < ssd1306_GetWidth(); i += 4)
            {
        ssd1306_DrawLine(0, 0, i, ssd1306_GetHeight() - 1);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    for (int16_t i = 0; i < ssd1306_GetHeight(); i += 4)
            {
        ssd1306_DrawLine(0, 0, ssd1306_GetWidth() - 1, i);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    HAL_Delay(250);

    ssd1306_Clear();
    for (int16_t i = 0; i < ssd1306_GetWidth(); i += 4)
            {
        ssd1306_DrawLine(0, ssd1306_GetHeight() - 1, i, 0);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    for (int16_t i = ssd1306_GetHeight() - 1; i >= 0; i -= 4)
            {
        ssd1306_DrawLine(0, ssd1306_GetHeight() - 1, ssd1306_GetWidth() - 1, i);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    HAL_Delay(500);
    ssd1306_Clear();
    for (int16_t i = ssd1306_GetWidth() - 1; i >= 0; i -= 4)
            {
        ssd1306_DrawLine(ssd1306_GetWidth() - 1, ssd1306_GetHeight() - 1, i, 0);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    for (int16_t i = ssd1306_GetHeight() - 1; i >= 0; i -= 4)
            {
        ssd1306_DrawLine(ssd1306_GetWidth() - 1, ssd1306_GetHeight() - 1, 0, i);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    HAL_Delay(500);
    ssd1306_Clear();
    for (int16_t i = 0; i < ssd1306_GetHeight(); i += 4)
            {
        ssd1306_DrawLine(ssd1306_GetWidth() - 1, 0, 0, i);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    for (int16_t i = 0; i < ssd1306_GetWidth(); i += 4)
            {
        ssd1306_DrawLine(ssd1306_GetWidth() - 1, 0, i, ssd1306_GetHeight() - 1);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    HAL_Delay(500);
}

// Adapted from Adafruit_SSD1306
void drawRect(void)
{
    for (int16_t i = 0; i < ssd1306_GetHeight() / 2; i += 2)
            {
        ssd1306_DrawRect(i, i, ssd1306_GetWidth() - 2 * i, ssd1306_GetHeight() - 2 * i);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
}

// Adapted from Adafruit_SSD1306
void fillRect(void) {
    uint8_t color = 1;
    for (int16_t i = 0; i < ssd1306_GetHeight() / 2; i += 3)
            {
        ssd1306_SetColor((color % 2 == 0) ? Black : White); // alternate colors
        ssd1306_FillRect(i, i, ssd1306_GetWidth() - i * 2, ssd1306_GetHeight() - i * 2);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
        color++;
    }
    // Reset back to WHITE
    ssd1306_SetColor(White);
}

// Adapted from Adafruit_SSD1306
void drawCircle(void)
{
    for (int16_t i = 0; i < ssd1306_GetHeight(); i += 4)
            {
        ssd1306_DrawCircle(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2, i);
        ssd1306_UpdateScreen();
        HAL_Delay(10);
    }
    HAL_Delay(1000);
    ssd1306_Clear();

    // This will draw the part of the circel in quadrant 1
    // Quadrants are numberd like this:
    //   0010 | 0001
    //  ------|-----
    //   0100 | 1000
    //
    ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2, ssd1306_GetHeight() / 4, 0x01 /*0b00000001*/);
    ssd1306_UpdateScreen();
    HAL_Delay(200);
    ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2, ssd1306_GetHeight() / 4, 0x03 /*0b00000011*/);
    ssd1306_UpdateScreen();
    HAL_Delay(200);
    ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2, ssd1306_GetHeight() / 4, 0x07 /*0b00000111*/);
    ssd1306_UpdateScreen();
    HAL_Delay(200);
    ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2, ssd1306_GetHeight() / 4, 0x0F /*0b00001111*/);
    ssd1306_UpdateScreen();
}

void drawProgressBarDemo(int counter)
{
    char str[128];
    // draw the progress bar
    ssd1306_DrawProgressBar(0, 32, 120, 10, counter);

    // draw the percentage as String
    ssd1306_SetCursor(55, 15);
    sprintf(str, "%i%%", counter);
    ssd1306_WriteString(str, Font_7x10);
    ssd1306_UpdateScreen();
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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

    printf("\n\n\n--------\nStarting\n");

    printf("Scan i2c1\n");
    // Go through all possible i2c addresses
    for (uint8_t i = 0; i < 128; i++) {

        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 3, 5) == HAL_OK) {
            // We got an ack
            printf("%2x ", i);
        } else {
            printf("-- ");
        }

        if (i > 0 && (i + 1) % 16 == 0)
            printf("\n");

    }

    printf("\n");

    ssd1306_Init();
    ssd1306_FlipScreenVertically();
    ssd1306_Clear();
    ssd1306_SetColor(White);
    //ssd1306_SetContrast(0x01);

//      ssd1306_TestAll();
//    ssd1306_Fill(Black);
//    ssd1306_DrawBitmap(32, 0, github_logo_64x64, 64, 64, White);
//    ssd1306_UpdateScreen();
//
//    for (int i = 0; i < 128; ++i) {
//        uint32_t start = uwTick;
//        ssd1306_ShiftLeft();
//        ssd1306_UpdateScreen();
//        printf("LShift t = %lu\n", uwTick - start);
//        HAL_Delay(25);
//    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

        drawLines();
        HAL_Delay(1000);
        ssd1306_Clear();

        drawRect();
        HAL_Delay(1000);
        ssd1306_Clear();

        fillRect();
        HAL_Delay(1000);
        ssd1306_Clear();

        drawCircle();
        HAL_Delay(1000);
        ssd1306_Clear();

        for(int i = 0; i < 100; i++)
        {
          drawProgressBarDemo(i);
          HAL_Delay(25);
          while(!ssd1306_UpdateScreenCompleted());
          ssd1306_Clear();
        }

        ssd1306_DrawRect(0, 0, ssd1306_GetWidth(), ssd1306_GetHeight());
        ssd1306_SetCursor(8, 20);
        ssd1306_WriteString("SSD1306", Font_16x26);
        ssd1306_UpdateScreen();
        HAL_Delay(2000);
        ssd1306_Clear();
        ssd1306_DrawBitmap(0, 0, 128, 64, stm32fan);
        ssd1306_UpdateScreen();
        HAL_Delay(2000);
        ssd1306_InvertDisplay();
        HAL_Delay(2000);
        ssd1306_NormalDisplay();
        ssd1306_Clear();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
