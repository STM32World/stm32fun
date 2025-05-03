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
#include "fatfs.h"
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

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

const uint32_t sample_frequencies[] = {
I2S_AUDIOFREQ_96K,
I2S_AUDIOFREQ_48K,
I2S_AUDIOFREQ_44K
};

uint8_t current_freq = 0;

float freq[2] = {
        8000,
        8001
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

const char total_uptime_filename[] = "uptime.dat";

int16_t i2s_dma_buffer[I2S_DMA_BUFFER_SIZE];
int16_t *do_buffer;

uint32_t buffers_done;
uint32_t total_uptime;

uint8_t open_next_file = 1;

FRESULT res;
DIR dir;
FIL music_file;
FILINFO music_file_info;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BTN_Pin) {

        printf("BTN ");
        GPIO_PinState pin_state = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);

        if (pin_state == GPIO_PIN_RESET) {
            printf("released\n");
            open_next_file = 1;
            //new_btn_state = 1;
        } else {
            printf("pressed\n");
            //new_btn_state = 0;
        }

    }
}

void ls() {

    char *path;

    path = ""; // where you want to list

    res = f_opendir(&dir, path);

#ifdef DEBUG
    if (res != FR_OK)
        printf("res = %d f_opendir\n", res);
#endif

    if (res == FR_OK) {
        while (1) {

            FILINFO fno;

            res = f_readdir(&dir, &fno);

#ifdef DEBUG
            if (res != FR_OK)
                printf("res = %d f_readdir\n", res);
#endif

            if ((res != FR_OK) || (fno.fname[0] == 0))
                break;

            printf("%c%c%c%c %10d %s/%s\n",
                    ((fno.fattrib & AM_DIR) ? 'D' : '-'),
                    ((fno.fattrib & AM_RDO) ? 'R' : '-'),
                    ((fno.fattrib & AM_SYS) ? 'S' : '-'),
                    ((fno.fattrib & AM_HID) ? 'H' : '-'),
                    (int) fno.fsize, path, fno.fname);
        }

    }

    uint32_t free_clusters;
    FATFS *fs_ptr;

    res = f_getfree("", &free_clusters, &fs_ptr);
    if (res == FR_OK) {
        uint32_t totalBlocks = (fs_ptr->n_fatent - 2) * fs_ptr->csize;
        uint32_t freeBlocks = free_clusters * fs_ptr->csize;

        printf("Total blocks: %lu (%lu Mb)\n", totalBlocks, totalBlocks / 2000);
        printf("Free blocks : %lu (%lu Mb)\n", freeBlocks, freeBlocks / 2000);
    } else {
        printf("Unable to get free space\n");
    }

}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    do_buffer = &i2s_dma_buffer[2 * I2S_DMA_BUFFER_SAMPLES]; // Second half
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    do_buffer = &i2s_dma_buffer[0]; // Second half
}

uint8_t BSP_SD_IsDetected(void)
{
    __IO uint8_t status = SD_PRESENT;

//    if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET) {
//        status = SD_NOT_PRESENT;
//    }

    return status;
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
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    /* USER CODE BEGIN 2 */

    printf("\n\n\n---------------------\nStarting music player\n");

    printf("SD Card Information:\n");
    printf("Block size  : %lu\n", hsd.SdCard.BlockSize);
    printf("Block nmbr  : %lu\n", hsd.SdCard.BlockNbr);
    printf("Card size   : %lu\n", (hsd.SdCard.BlockSize * hsd.SdCard.BlockNbr) / 1000);
    printf("Card version: %lu\n", hsd.SdCard.CardVersion);

    uint32_t wbytes, rbytes;

    if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
        printf("Unable to mount disk\n");
        Error_Handler();
    }

    if (f_open(&SDFile, total_uptime_filename, FA_OPEN_EXISTING | FA_READ) == FR_OK) {
        if (f_read(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &rbytes) == FR_OK) {
            printf("Total uptime = %lu\n", total_uptime);
            f_close(&SDFile);
        } else {
            printf("Unable to read\n");
            Error_Handler();
        }
    } else {
        // File did not exist - let's create it
        if (f_open(&SDFile, total_uptime_filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
            if (f_write(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &wbytes) == FR_OK) {
                printf("File %s created\n", total_uptime_filename);
                f_close(&SDFile);
            } else {
                printf("Unable to write\n");
                Error_Handler();
            }
        } else {
            printf("Unable to create\n");
            Error_Handler();
        }
    }

    res = f_findfirst(&dir, &music_file_info, "", "*.wav");

    printf("Found: %s\n", music_file_info.fname);

//
//    // Create tick file always
//    if (f_open(&SDFile, tick_filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
//        f_close(&SDFile);
//    }

    ls();

    printf("Total uptime reported = %lu\n", total_uptime);

    set_angle_changes();

    //HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) &i2s_dma_buffer, 2 * I2S_DMA_BUFFER_SAMPLES);
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

            printf("Tick %lu (loop=%lu bdo=%lu)\n", now / 1000, loop_cnt, buffers_done);

            ++total_uptime;

            // Update the total uptime file
            if (f_open(&SDFile, total_uptime_filename, FA_OPEN_EXISTING | FA_WRITE) == FR_OK) {
                if (f_write(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &wbytes) != FR_OK) {
                    printf("Unable to write\n");
                }
                f_close(&SDFile);
            } else {
                printf("Unable to open file\n");
            }

            open_next_file = 0;

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

        if (open_next_file) {

            char buf[32];
            unsigned int n;

            f_close(&music_file);

            // Experimental advance to next
            res = f_findnext(&dir, &music_file_info);

            if (res != FR_OK || music_file_info.fname[0] == '\0') { // If we're out of files start again
                res = f_findfirst(&dir, &music_file_info, "", "*.wav");
            }

            printf("Next file: %s\n", music_file_info.fname);

            if (f_open(&music_file, music_file_info.fname, FA_READ) != FR_OK) {
                printf("Unable to open %s\n", music_file_info.fname);
            }

            // Try to read first four byte
            if (f_read(&music_file, &buf, 4, &n) != FR_OK && n != 4) {
                printf("Could not read 4 byte\n");
            }

            printf("File header: %s\n", buf);

            open_next_file = 0;

        }

        if (do_buffer) {

            for (int i = 0; i < 2 * I2S_DMA_BUFFER_SAMPLES; i += 2) { // Two samples left/right per step
                do_buffer[i] = OUTPUT_MID * amplification[0] * arm_cos_f32(angle[0]);      // Left
                do_buffer[i + 1] = OUTPUT_MID * amplification[1] * arm_cos_f32(angle[1]);  // Right
                angle[0] += angle_change[0];
                angle[1] += angle_change[1];
                if (angle[0] > TAU)
                    angle[0] -= TAU;
                if (angle[1] > TAU)
                    angle[1] -= TAU;
            }

            ++buffers_done;
            do_buffer = 0;
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
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{

    /* USER CODE BEGIN SDIO_Init 0 */

    /* USER CODE END SDIO_Init 0 */

    /* USER CODE BEGIN SDIO_Init 1 */

    /* USER CODE END SDIO_Init 1 */
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 4;
    /* USER CODE BEGIN SDIO_Init 2 */

    // First init with 1B bus - SD card will not initialize with 4 bits
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    if (HAL_SD_Init(&hsd) != HAL_OK) {
        Error_Handler();
    }

    // Now we can switch to 4 bit mode
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE END SDIO_Init 2 */

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
    __HAL_RCC_GPIOD_CLK_ENABLE();

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
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SD_DET_Pin */
    GPIO_InitStruct.Pin = SD_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
