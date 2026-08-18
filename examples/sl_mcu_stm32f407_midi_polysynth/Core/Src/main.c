/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STM32World <lth@stm32world.com>
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
#include <math.h>
#include <arm_math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum wave_t {
    SINE_WAVE = 0,
    SAW_RIGHT_WAVE = 1,
    SAW_LEFT_WAVE = 2,
    TRIANGLE_WAVE = 3,
    SQUARE_WAVE = 4
};

typedef struct {
    uint16_t *buffer;
    enum wave_t wave_type;
    float angle;
    float angle_change;
    float attenuation;
} channel_queue_t;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DMA_BUFFER_SIZE 16
#define SAMPLE_FREQ 20000
#define OUTPUT_MID 2048
#define OUTPUT_MAX 4096

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static const float MIDI_NOTE_TO_FREQ[128] = {
    8.1758f,    8.6620f,    9.1770f,    9.7227f,    10.3009f,   10.9134f,   11.5623f,   12.2499f,   // 0 - 7
    12.9783f,   13.7500f,   14.5676f,   15.4339f,   16.3516f,   17.3239f,   18.3540f,   19.4454f,   // 8 - 15
    20.6017f,   21.8268f,   23.1247f,   24.4997f,   25.9565f,   27.5000f,   29.1352f,   30.8677f,   // 16 - 23
    32.7032f,   34.6478f,   36.7081f,   38.8909f,   41.2034f,   43.6535f,   46.2493f,   48.9994f,   // 24 - 31
    51.9131f,   55.0000f,   58.2705f,   61.7354f,   65.4064f,   69.2957f,   73.4162f,   77.7817f,   // 32 - 39
    82.4069f,   87.3071f,   92.4986f,   97.9989f,   103.8262f,  110.0000f,  116.5409f,  123.4708f,  // 40 - 47
    130.8128f,  138.5913f,  146.8324f,  155.5635f,  164.8138f,  174.6141f,  185.0000f,  195.9977f,  // 48 - 55
    207.6523f,  220.0000f,  233.0819f,  246.9417f,  261.6256f,  277.1826f,  293.6648f,  311.1270f,  // 56 - 63
    329.6276f,  349.2282f,  369.9944f,  391.9954f,  415.3047f,  440.0000f,  466.1638f,  493.8833f,  // 64 - 71
    523.2511f,  554.3653f,  587.3295f,  622.2540f,  659.2551f,  698.4565f,  739.9888f,  783.9909f,  // 72 - 79
    830.6094f,  880.0000f,  932.3275f,  987.7666f,  1046.5023f, 1108.7305f, 1174.6591f, 1244.5079f, // 80 - 87
    1318.5102f, 1396.9129f, 1479.9777f, 1567.9817f, 1661.2188f, 1760.0000f, 1864.6550f, 1975.5332f, // 88 - 95
    2093.0045f, 2217.4610f, 2349.3181f, 2489.0159f, 2637.0205f, 2793.8259f, 2959.9554f, 3135.9635f, // 96 - 103
    3322.4376f, 3520.0000f, 3729.3101f, 3951.0664f, 4186.0090f, 4434.9221f, 4698.6363f, 4978.0317f, // 104 - 111
    5274.0410f, 5587.6518f, 5919.9108f, 6271.9270f, 6644.8752f, 7040.0000f, 7458.6202f, 7902.1328f, // 112 - 119
    8372.0181f, 8869.8443f, 9397.2726f, 9956.0635f, 10548.0820f,11175.3037f,11839.8216f,12543.8540f // 120 - 127
};


DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

uint16_t dma_buffer_1[2 * DMA_BUFFER_SIZE];
//uint16_t dma_buffer_2[2 * DMA_BUFFER_SIZE];

// Handler for both channels
channel_queue_t dacs[2] = {
        {
                &dma_buffer_1[0],
                SINE_WAVE,
                0,
                1000 * (2 * M_PI / SAMPLE_FREQ),
                0.99
        },
        {
                &dma_buffer_2[0],
                SINE_WAVE,
                0,
                1000 * (2 * M_PI / SAMPLE_FREQ),
                0.99
        }
};

uint8_t change_wave = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send printf to uart1
int __io_putchar(int ch) {
    if (ch == '\n') {
        HAL_UART_Transmit(&huart1, (uint8_t*) "\r", 1, HAL_MAX_DELAY);
    }
    if (HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    return ch;
}

void process_buffer(channel_queue_t *channel) {
    // Pre-calculate inverse constants to replace slow division with multiplication
    const float inv_two_pi = 1.0f / (float)M_TWOPI;

    // Pre-calculate invariant scaling limits based on the channel's attenuation
    float peak_to_peak = (OUTPUT_MAX - 1) * channel->attenuation;
    float minimum_val  = OUTPUT_MID - (peak_to_peak / 2.0f);

    // Run through the buffer using Loop Unswitching for maximum performance
    switch (channel->wave_type) {

    case SINE_WAVE: {
        // Pre-calculate the absolute scaling factor for the sine loop
        float sine_amp = channel->attenuation * OUTPUT_MID;

        for (int i = 0; i < DMA_BUFFER_SIZE; ++i) {
            // arm_cos_f32 uses the FPU excellently, no division required
            channel->buffer[i] = (uint16_t)(OUTPUT_MID - (sine_amp * arm_cos_f32(channel->angle)));

            channel->angle += channel->angle_change;
            if (channel->angle >= (float)M_TWOPI) {
                channel->angle -= (float)M_TWOPI;
            }
        }
        break;
    }

    case SAW_RIGHT_WAVE:
        for (int i = 0; i < DMA_BUFFER_SIZE; ++i) {
            float normalized_phase = channel->angle * inv_two_pi;

            channel->buffer[i] = (uint16_t)(minimum_val + (normalized_phase * peak_to_peak));

            channel->angle += channel->angle_change;
            if (channel->angle >= (float)M_TWOPI) {
                channel->angle -= (float)M_TWOPI;
            }
        }
        break;

    case SAW_LEFT_WAVE:
        for (int i = 0; i < DMA_BUFFER_SIZE; ++i) {
            float normalized_phase = channel->angle * inv_two_pi;

            // Invert the phase step to make it ramp downwards
            channel->buffer[i] = (uint16_t)(minimum_val + ((1.0f - normalized_phase) * peak_to_peak));

            channel->angle += channel->angle_change;
            if (channel->angle >= (float)M_TWOPI) {
                channel->angle -= (float)M_TWOPI;
            }
        }
        break;

    case TRIANGLE_WAVE:
        for (int i = 0; i < DMA_BUFFER_SIZE; ++i) {
            float normalized_phase = channel->angle * inv_two_pi;
            float tri_value;

            if (normalized_phase < 0.5f) {
                // Rise phase: scale [0.0 to 0.5] -> [0.0 to 1.0]
                tri_value = normalized_phase * 2.0f;
            } else {
                // Fall phase: scale [0.5 to 1.0] -> [1.0 to 0.0]
                tri_value = 2.0f - (normalized_phase * 2.0f);
            }

            channel->buffer[i] = (uint16_t)(minimum_val + (tri_value * peak_to_peak));

            channel->angle += channel->angle_change;
            if (channel->angle >= (float)M_TWOPI) {
                channel->angle -= (float)M_TWOPI;
            }
        }
        break;

    case SQUARE_WAVE: {
        // Pre-calculate hard high and low values outside the loop
        uint16_t high_level = (uint16_t)(OUTPUT_MID + (peak_to_peak / 2.0f));
        uint16_t low_level  = (uint16_t)(OUTPUT_MID - (peak_to_peak / 2.0f));

        for (int i = 0; i < DMA_BUFFER_SIZE; ++i) {
            float normalized_phase = channel->angle * inv_two_pi;

            if (normalized_phase < 0.5f) {
                channel->buffer[i] = high_level;
            } else {
                channel->buffer[i] = low_level;
            }

            channel->angle += channel->angle_change;
            if (channel->angle >= (float)M_TWOPI) {
                channel->angle -= (float)M_TWOPI;
            }
        }
        break;
    }

    default:
        // Graceful fallback: output a flat mid-level DC signal if an invalid wave type is passed
        for (int i = 0; i < DMA_BUFFER_SIZE; ++i) {
            channel->buffer[i] = OUTPUT_MID;
        }
        break;
    }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    //++conv_half_ch1;
    dacs[0].buffer = &dma_buffer_1[0];
    process_buffer(&dacs[0]);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    //++conv_ch1;
    dacs[0].buffer = &dma_buffer_1[DMA_BUFFER_SIZE];
    process_buffer(&dacs[0]);
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    //++conv_half_ch2;
    dacs[1].buffer = &dma_buffer_2[0];
    process_buffer(&dacs[1]);
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    //++conv_ch2;
    dacs[1].buffer = &dma_buffer_2[DMA_BUFFER_SIZE];
    process_buffer(&dacs[1]);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BTN_Pin) { // If the button
        change_wave = 1;
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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

    printf("\n\n\nStarting Function Generator\n");

    HAL_TIM_Base_Start_IT(&htim6);

    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) &dma_buffer_1, 2 * DMA_BUFFER_SIZE, DAC_ALIGN_12B_R);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*) &dma_buffer_2, 2 * DMA_BUFFER_SIZE, DAC_ALIGN_12B_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    uint32_t now, loop_cnt = 0, next_blink = 500, next_tick = 1000;

    while (1) {

        now = uwTick;

        if (change_wave) {
            printf("Change wave\n");

            ++dacs[1].wave_type; // a tad ugly - if interrupt happens while this is one too big it will be handled by switch default

            if (dacs[1].wave_type > 4) dacs[1].wave_type = 0;

            change_wave = 0;
        }

        if (now >= next_blink) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            next_blink = now + 500;
        }

        if (now >= next_tick) {
            printf("Tick %lu (loop=%lu)\n", now / 1000, loop_cnt);
            loop_cnt = 0;
            next_tick = now + 1000;
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50- 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 921600;
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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
#ifdef USE_FULL_ASSERT
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
