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
#include "tusb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER_CLOCK_FREQ 1000000UL // 1 MHz (Timer frequency after Prescaler)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

// Lookup table converting MIDI Note numbers (0-127) to Frequency (Hz)
// Rounded to nearest integer. 0 = Note off / out of range.
static const uint16_t midi_note_freq[128] = {
        8, 9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 15,   // 0-11
        16, 17, 18, 19, 21, 22, 23, 24, 26, 28, 29, 31,   // 12-23
        33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62,   // 24-35
        65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123,  // 36-47
        131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247,  // 48-59
        262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,  // 60-71 (60 = C4/Middle C, 69 = A4/440Hz)
        523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,  // 72-83
        1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, // 84-95
        2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, // 96-107
        4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902, // 108-119
        8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544                         // 120-127
        };

// Tracks the last active note so Note-Off only kills the sound if it matches
static uint8_t current_note = 0xFF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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

/**
 * @brief Sets PWM frequency (20 Hz - 8000 Hz) at a fixed 50% duty cycle.
 * @param htim Pointer to the TIM_HandleTypeDef structure
 * @param Channel TIM Channels to be configured (e.g., TIM_CHANNEL_1)
 * @param frequency Target frequency in Hz (20 to 8000)
 */
void set_frequency(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t frequency)
{
    // Clamp frequency range
    if (frequency < 20)
        frequency = 20;
    if (frequency > 8000)
        frequency = 8000;

    // Frequency = Timer_Clock / (ARR + 1)  ==>  ARR = (Timer_Clock / Frequency) - 1
    uint32_t new_arr = (TIMER_CLOCK_FREQ / frequency) - 1;

    // 2. Calculate CCR for 50% duty cycle
    uint32_t new_ccr = (new_arr + 1) / 2;

    // 3. Update Timer Registers
    __HAL_TIM_SET_AUTORELOAD(htim, new_arr);
    __HAL_TIM_SET_COMPARE(htim, Channel, new_ccr);

}

/**
 * Helper function to play or stop sound on the buzzer
 */
void buzzer_note_on(uint8_t note) {
    if (note > 127)
        return;

    uint16_t freq = midi_note_freq[note];
    current_note = note;

    // Set PWM frequency and enable PWM Output Channel
    set_frequency(&htim1, TIM_CHANNEL_1, freq);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void buzzer_note_off(uint8_t note) {
    // Only stop playing if the note being released is the currently active note
    if (note == current_note || note == 0xFF) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        current_note = 0xFF;
    }
}

// TinyUSB callback when receiving midi message
void tud_midi_rx_cb(uint8_t itf)
{
    (void) itf;
    uint8_t packet[4];

    while (tud_midi_packet_read(packet))
    {
        uint8_t cable = packet[0] >> 4;
        uint8_t cin = packet[0] & 0x0F;
        uint8_t status = packet[1];
        uint8_t data1 = packet[2]; // MIDI Note
        uint8_t data2 = packet[3]; // Velocity

        uint8_t channel = (status & 0x0F) + 1;
        uint8_t msg_type = status & 0xF0;

        printf("RAW Packet: [%02X %02X %02X %02X] | Cable: %d | CIN: 0x%X\r\n",
                packet[0], packet[1], packet[2], packet[3], cable, cin);

        switch (msg_type) {
        case 0x90: // Note On
            if (data2 > 0) {
                printf("  -> [Ch %d] Note ON  | Note: %d, Velocity: %d\r\n", channel, data1, data2);
                buzzer_note_on(data1);  // <--- PLAY NOTE
            } else {
                printf("  -> [Ch %d] Note OFF | Note: %d (Vel 0)\r\n", channel, data1);
                buzzer_note_off(data1); // <--- STOP NOTE
            }
            break;

        case 0x80: // Note Off
            printf("  -> [Ch %d] Note OFF | Note: %d, Velocity: %d\r\n", channel, data1, data2);
            buzzer_note_off(data1);     // <--- STOP NOTE
            break;

        case 0xB0: // Control Change (e.g. All Notes Off / Controller 123)
            printf("  -> [Ch %d] CC       | Controller: %d, Value: %d\r\n", channel, data1, data2);
            if (data1 == 123 || data1 == 120) {
                buzzer_note_off(0xFF); // Kill sound on All Notes Off CC
            }
            break;

        default:
            break;
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
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

    printf("\n\n\n--------\nStarting\n");

    // Initialize the TinyUSB Device stack
    tusb_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    register uint32_t now = 0, loop_cnt = 0, next_blink = 500, next_tick = 1000;

    while (1) {

        now = uwTick;

        if (now >= next_blink) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            next_blink += 500;
        }

        if (now >= next_tick) {

            printf("Tick %lu (loop = %lu)\n", now / 1000, loop_cnt);

            loop_cnt = 0;
            next_tick = now + 1000;
        }

        tud_task();

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
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
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
