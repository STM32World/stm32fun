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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TAU 6.28318530717958647692
#define I2S_DMA_BUFFER_SAMPLES 1024
#define I2S_DMA_BUFFER_SIZE 2 * 2 * I2S_DMA_BUFFER_SAMPLES // 2 full buffers L+R samples
#define SAMPLE_FREQ 96000
#define OUTPUT_MID 32768

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

const uint32_t sample_frequencies[] = {
I2S_AUDIOFREQ_96K,
I2S_AUDIOFREQ_48K,
I2S_AUDIOFREQ_44K
};

uint8_t current_freq = 0;

float freq[2] = {
        220,
        440
};

float angle[2] = {
        0,
        0
};

float angle_change[2] = { // Will be calculated from freq
        0,
                0
        };

float amplification[2] = {
        0.8,
        0.8
};

int16_t i2s_dma_buffer[I2S_DMA_BUFFER_SIZE];

//int16_t *dma_buffer_to_fill;

uint8_t dma_buffer_processing = 0;
uint32_t dma_overflow = 0;

uint32_t i2c_cb, i2c_hcb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2S2_Init(void);
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

void process_buffer(int16_t *buffer) {

    dma_buffer_processing = 1;

    for (int i = 0; i < 2 * I2S_DMA_BUFFER_SAMPLES; i += 2) { // Two samples left/right per step
        buffer[i] = OUTPUT_MID * amplification[0] * arm_cos_f32(angle[0]);      // Left
        buffer[i + 1] = OUTPUT_MID * amplification[1] * arm_cos_f32(angle[1]);  // Right
        angle[0] += angle_change[0];
        angle[1] += angle_change[1];
        if (angle[0] > TAU)
            angle[0] -= TAU;
        if (angle[1] > TAU)
            angle[1] -= TAU;
    }

    dma_buffer_processing = 0;

}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (dma_buffer_processing) {
        ++dma_overflow; // Still processing previous buffer
    } else {
        process_buffer(&i2s_dma_buffer[2 * I2S_DMA_BUFFER_SAMPLES]); // Second half
        ++i2c_cb;
    }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (dma_buffer_processing) {
        ++dma_overflow; // Still processing previous buffer
    } else {
        process_buffer(&i2s_dma_buffer[0]);
        ++i2c_hcb;
    }

}

void set_angle_changes() {
    angle_change[0] = freq[0] * (TAU / sample_frequencies[current_freq]); // left
    angle_change[1] = freq[1] * (TAU / sample_frequencies[current_freq]);  // right
}

void set_i2s_freq(uint32_t freq) {

    // Pause the DMA transfers
    HAL_I2S_DMAStop(&hi2s2);

    // Deinit
    HAL_I2S_DeInit(&hi2s2);

    hi2s2.Init.AudioFreq = freq;

    HAL_I2S_Init(&hi2s2);

    set_angle_changes();

    // Restart DMA
    //HAL_I2S_DMAResume(&hi2s2);
    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) &i2s_dma_buffer, I2S_DMA_BUFFER_SIZE);

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
    MX_USART1_UART_Init();
    MX_I2S2_Init();
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */

    printf("\n\n\n---------------------\nStarting music player\n");

    set_angle_changes();

    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) &i2s_dma_buffer, I2S_DMA_BUFFER_SIZE);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t now = 0, next_blink = 500, next_tick = 1000, next_freq = 10000, loop_cnt = 0;

    while (1) {

        now = uwTick;

        if (now >= next_blink) {

            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            next_blink = now + 500;
        }

        if (now >= next_tick) {

            printf("Tick %lu (loop=%lu hcb=%lu cb=%lu do=%lu)\n", now / 1000, loop_cnt, i2c_hcb, i2c_cb, dma_overflow);

            loop_cnt = 0;
            next_tick = now + 1000;

        }

        if (now >= next_freq) {

            // Increase index
            ++current_freq;

            // Loop around if too big
            if (current_freq >= sizeof(sample_frequencies) / sizeof(sample_frequencies[0])) {
                current_freq = 0;
            }

            printf("Setting sample freq to: %lu\n", sample_frequencies[current_freq]);

            // Set the freq
            set_i2s_freq(sample_frequencies[current_freq]);

            next_freq = now + 10000;
        }

        ++loop_cnt;

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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
            {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
            {
        Error_Handler();
    }
}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void)
{

    /* USER CODE BEGIN I2S2_Init 0 */

    /* USER CODE END I2S2_Init 0 */

    /* USER CODE BEGIN I2S2_Init 1 */

    /* USER CODE END I2S2_Init 1 */
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN I2S2_Init 2 */

    /* USER CODE END I2S2_Init 2 */

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
    /* DMA1_Stream4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BTN_Pin */
    GPIO_InitStruct.Pin = BTN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SD_DET_Pin */
    GPIO_InitStruct.Pin = SD_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

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
