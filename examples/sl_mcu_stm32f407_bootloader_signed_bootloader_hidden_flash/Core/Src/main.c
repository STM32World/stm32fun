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
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint32_t magic;          // 4 bytes: Magic identifier (e.g. 0x4150504B / "KAPP")
    uint32_t payload_len;    // 4 bytes: Size of binary firmware in bytes
    uint8_t signature[64];  // 64 bytes: ECDSA raw signature (32-byte R + 32-byte S)
    uint8_t reserved[56];   // 56 bytes: Zero padding to align header to 128 bytes
} Header128_t;

typedef void (*pFunction)(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #define printf(...)      // Nuke printf to save space
#define DFU_BOOT_FLAG 0xDEADBEEF
#define APP_ADDRESS 0x08008000
#define FIRMWARE_MAGIC 0x4150504B
#define STAGING_ADDRESS 0x08080000              // Sector 8 - start of the hidden upper 512 kB
#define STAGING_SECTOR FLASH_SECTOR_8
#define APP_MAX_SIZE (480 * 1024)               // Sectors 2-7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern int _bflag;
uint32_t *dfu_boot_flag;
pFunction JumpToApplication;
uint32_t JumpAddress;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static bool flash_stm32_app(uint32_t src_addr, uint32_t dst_addr, uint32_t length);
static bool erase_staging(void);

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
 * @brief Erases the application sectors (2-7) and copies the payload from the hidden staging area
 */
static bool flash_stm32_app(uint32_t src_addr, uint32_t dst_addr, uint32_t length) {

    HAL_FLASH_Unlock();

    // Erase application sectors only (2-7) - sectors 8-11 hold the staged image
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_2; // Sector 2 starts at 0x08008000
    EraseInitStruct.NbSectors = 6;           // Sectors 2-7, up to 0x0807FFFF

    printf("[BOOT] Erasing STM32 internal Flash sectors 2-7...\n");
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        printf("[BOOT] Error erasing internal flash sector %lu!\n", SectorError);
        HAL_FLASH_Lock();
        return false;
    }

    // Both source and destination are memory mapped, so copy word by word.
    // Rounding up to a whole word is harmless - the staging sector is erased beyond the payload.
    printf("[BOOT] Copying firmware 0x%08lX -> 0x%08lX (%lu bytes)...\n", src_addr, dst_addr, length);
    for (uint32_t offset = 0; offset < length; offset += 4) {
        uint32_t word = *(__IO uint32_t*) (src_addr + offset);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dst_addr + offset, word) != HAL_OK) {
            printf("[BOOT] Programming failed at 0x%08lX!\n", dst_addr + offset);
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    printf("[BOOT] Internal Flash programming complete!\n");
    return true;
}

/**
 * @brief Erases the staging sector so the image is not flashed again on next boot
 */
static bool erase_staging(void) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = STAGING_SECTOR;
    EraseInitStruct.NbSectors = 1;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    HAL_FLASH_Lock();

    return status == HAL_OK;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    // We handle all this before the HAL Libraries have been initialized.
    dfu_boot_flag = (uint32_t*) (&_bflag); // set in linker script

    if (*dfu_boot_flag != DFU_BOOT_FLAG) {

        /* Test if user code is programmed starting from address 0x08008000 */
        if (((*(__IO uint32_t*) APP_ADDRESS) & 0x2FFC0000) == 0x20000000) {

            /* Jump to user application */
            JumpAddress = *(__IO uint32_t*) (APP_ADDRESS + 4);
            JumpToApplication = (pFunction) JumpAddress;

            /* Initialize user application's Stack Pointer */
            __set_MSP(*(__IO uint32_t*) APP_ADDRESS);

            JumpToApplication();
        }

    }

    *dfu_boot_flag = 0; // So next boot won't be affected

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
    /* USER CODE BEGIN 2 */

    printf("\n\n\n\n--------\nBootloader\n");

    // The staged image header sits at the start of the hidden flash
    const Header128_t *header = (const Header128_t*) STAGING_ADDRESS;

    if (header->magic == FIRMWARE_MAGIC && header->payload_len > 0 && header->payload_len <= APP_MAX_SIZE) {
        printf("[BOOT] New firmware found in hidden flash (%lu bytes).\n", header->payload_len);

        if (flash_stm32_app(STAGING_ADDRESS + sizeof(Header128_t), APP_ADDRESS, header->payload_len)) {

            printf("[BOOT] Erasing staging sector...\n");
            if (!erase_staging()) {
                printf("[BOOT] ERROR: Failed to erase staging sector!\n");
            }

            printf("[BOOT] Firmware update complete. Rebooting MCU...\n");
            HAL_NVIC_SystemReset();

        } else {
            printf("[BOOT] ERROR: Failed to write internal flash!\n");
        }
    } else {
        printf("[BOOT] No valid firmware found in hidden flash.\n");
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    register uint32_t now = 0, loop_cnt = 0, next_blink = 100, next_tick = 1000;

    while (1) {

        now = uwTick;

        if (now >= next_blink) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            next_blink += 100;
        }

        if (now >= next_tick) {

            printf("Tick %lu (loop = %lu)\n", now / 1000, loop_cnt);

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
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
