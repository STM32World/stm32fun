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
#include "w25qxx.h"

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
#define DFU_BOOT_FLAG 0xDEADBEEF
#define APP_ADDRESS 0x08008000
#define CHUNK_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
W25QXX_HandleTypeDef w25qxx;

extern int _bflag;
uint32_t *dfu_boot_flag;
pFunction JumpToApplication;
uint32_t JumpAddress;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

static bool flash_stm32_app(uint32_t src_flash_addr, uint32_t dst_stm32_addr, uint32_t length);

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
 * @brief Erases STM32 internal flash sectors and programs payload from external W25Qxx SPI flash
 */
static bool flash_stm32_app(uint32_t src_flash_addr, uint32_t dst_stm32_addr, uint32_t length) {
    uint8_t buffer[CHUNK_SIZE];
    uint32_t bytes_written = 0;

    HAL_FLASH_Unlock();

    // Erase STM32 internal Flash sectors for Application space (Sector 2 through 11 for STM32F407 1MB)
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_2; // Sector 2 starts at 0x08008000
    EraseInitStruct.NbSectors = 10;            // Erase all upper sectors allocated for app

    printf("[BOOT] Erasing STM32 internal Flash starting at Sector 2...\n");
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        printf("[BOOT] Error erasing internal flash sector %lu!\n", SectorError);
        HAL_FLASH_Lock();
        return false;
    }
    printf("[BOOT] STM32 internal Flash erased successfully.\n");

    // Copy payload from SPI Flash to STM32 Internal Flash
    printf("[BOOT] Copying firmware to internal Flash (Address 0x%08lX, %lu bytes)...\n", dst_stm32_addr, length);
    while (bytes_written < length) {
        uint32_t chunk_len = (length - bytes_written < CHUNK_SIZE) ? (length - bytes_written) : CHUNK_SIZE;

        w25qxx_read(&w25qxx, src_flash_addr + bytes_written, buffer, chunk_len);

        for (uint32_t i = 0; i < chunk_len; i++) {
            uint32_t target_addr = dst_stm32_addr + bytes_written + i;
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, target_addr, buffer[i]) != HAL_OK) {
                printf("[BOOT] Programming failed at 0x%08lX!\n", target_addr);
                HAL_FLASH_Lock();
                return false;
            }
        }

        bytes_written += chunk_len;
    }

    HAL_FLASH_Lock();
    printf("[BOOT] Internal Flash programming complete!\n");
    return true;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

    printf("\n\n\n\n--------\nBootloader\n");

    if (w25qxx_init(&w25qxx, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin) != W25QXX_Ok) {
        printf("W25QXX Error");
        Error_Handler();
    }

    printf("W25QXX successfully initialized\n");
    printf("Manufacturer       = 0x%2x\n", w25qxx.manufacturer_id);
    printf("Device             = 0x%4x\n", w25qxx.device_id);
    printf("Block size         = 0x%04lx (%lu)\n", w25qxx.block_size, w25qxx.block_size);
    printf("Block count        = 0x%04lx (%lu)\n", w25qxx.block_count, w25qxx.block_count);
    printf("Sector size        = 0x%04lx (%lu)\n", w25qxx.sector_size, w25qxx.sector_size);
    printf("Sectors per block  = 0x%04lx (%lu)\n", w25qxx.sectors_in_block, w25qxx.sectors_in_block);
    printf("Page size          = 0x%04lx (%lu)\n", w25qxx.page_size, w25qxx.page_size);
    printf("Pages per sector   = 0x%04lx (%lu)\n", w25qxx.pages_in_sector, w25qxx.pages_in_sector);
    printf("Total size (in kB) = 0x%04lx (%lu)\n", (w25qxx.block_count * w25qxx.block_size) / 1024, (w25qxx.block_count * w25qxx.block_size) / 1024);

    // Read 128-byte header from external SPI Flash
    Header128_t header;
    w25qxx_read(&w25qxx, 0x00000000, (uint8_t*) &header, sizeof(Header128_t));

    // Check if valid firmware exists in external SPI flash
    if (header.magic == 0x4150504B && header.payload_len > 0) {
        printf("[BOOT] New valid firmware found in SPI flash (%lu bytes).\n", header.payload_len);

        // Flash firmware to STM32 internal Flash starting at offset 128 (payload area)
        if (flash_stm32_app(sizeof(Header128_t), APP_ADDRESS, header.payload_len)) {

            printf("[BOOT] Nuking magic marker...\n");

            // Invalidate magic in SPI flash header so we don't re-flash on next reboot
            uint32_t invalid_magic = 0xFFFFFFFF;

            w25qxx_erase(&w25qxx, 0x00000000, sizeof(invalid_magic));
            w25qxx_write(&w25qxx, 0x00000000, (uint8_t*) &invalid_magic, sizeof(invalid_magic));

            printf("[BOOT] Firmware update complete. Rebooting MCU...\n");
            HAL_NVIC_SystemReset();
        } else {
            printf("[BOOT] ERROR: Failed to write internal flash!\n");
        }
    } else {
        printf("[BOOT] No valid firmware magic found in SPI flash.\n");
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

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
