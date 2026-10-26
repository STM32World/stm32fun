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
#include "mbedtls.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "w25qxx.h"
#include "tusb.h"

#include "mbedtls/md.h"
#include "mbedtls/pk.h"
#include "mbedtls/bignum.h"
#include "mbedtls/asn1write.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;          // 4 bytes: Magic identifier (e.g. 0x4150504B / "KAPP")
    uint32_t payload_len;    // 4 bytes: Size of binary firmware in bytes
    uint8_t  signature[64];  // 64 bytes: ECDSA raw signature (32-byte R + 32-byte S)
    uint8_t  reserved[56];   // 56 bytes: Zero padding to align header to 128 bytes
} Header128_t;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHUNK_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
W25QXX_HandleTypeDef w25qxx;

static uint32_t dfu_bytes_received = 0;
static uint32_t last_erased_sector_addr = 0xFFFFFFFF;

static const char *PUBLIC_KEY_PEM =
    "-----BEGIN PUBLIC KEY-----\n"
    "MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAE/TvaaBFk8lA2UK6vEFBSNEIluTsx\n"
    "vgLjSyEFtuQmGuPXHu4Qaa7d4DkQgQ1cXTG1jOtfXUym4S4ddeaSyJtTGg==\n"
    "-----END PUBLIC KEY-----\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
bool verify_flash_firmware_stream(uint32_t flash_payload_addr, size_t payload_len, const uint8_t *raw_sig);
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
 * @brief Convert raw 64-byte (32-byte R + 32-byte S) ECDSA signature to ASN.1 DER format
 */
static size_t raw_sig_to_der(const uint8_t *raw_sig, uint8_t *der_buf, size_t der_buf_size) {
    int ret = 0;
    unsigned char *p = der_buf + der_buf_size;
    size_t len = 0;

    mbedtls_mpi r, s;
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    if (mbedtls_mpi_read_binary(&r, raw_sig, 32) == 0 &&
        mbedtls_mpi_read_binary(&s, raw_sig + 32, 32) == 0) {

        MBEDTLS_ASN1_CHK_ADD(len, mbedtls_asn1_write_mpi(&p, der_buf, &s));
        MBEDTLS_ASN1_CHK_ADD(len, mbedtls_asn1_write_mpi(&p, der_buf, &r));
        MBEDTLS_ASN1_CHK_ADD(len, mbedtls_asn1_write_len(&p, der_buf, len));
        MBEDTLS_ASN1_CHK_ADD(len, mbedtls_asn1_write_tag(&p, der_buf, MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE));

        memmove(der_buf, p, len);
    } else {
        ret = -1;
    }

    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);

    return (ret < 0) ? 0 : len;
}

/**
 * @brief Streams firmware payload from flash through SHA-256 and verifies ECDSA signature
 */
bool verify_flash_firmware_stream(uint32_t flash_payload_addr, size_t payload_len, const uint8_t *raw_sig) {
    uint8_t ram_buffer[CHUNK_SIZE];
    uint8_t hash[32];
    uint8_t der_sig[80];

    size_t der_len = raw_sig_to_der(raw_sig, der_sig, sizeof(der_sig));
    if (der_len == 0) {
        printf("[VERIFY] Failed to format signature to ASN.1 DER\n");
        return false;
    }

    mbedtls_md_context_t md_ctx;
    mbedtls_pk_context pk_ctx;

    mbedtls_md_init(&md_ctx);
    mbedtls_pk_init(&pk_ctx);

    int ret = -1;

    do {
        const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        if ((ret = mbedtls_md_setup(&md_ctx, md_info, 0)) != 0) break;
        if ((ret = mbedtls_md_starts(&md_ctx)) != 0) break;

        size_t bytes_remaining = payload_len;
        uint32_t current_flash_addr = flash_payload_addr;

        while (bytes_remaining > 0) {
            size_t bytes_to_read = (bytes_remaining < CHUNK_SIZE) ? bytes_remaining : CHUNK_SIZE;

            w25qxx_read(&w25qxx, current_flash_addr, ram_buffer, bytes_to_read);

            if ((ret = mbedtls_md_update(&md_ctx, ram_buffer, bytes_to_read)) != 0) break;

            current_flash_addr += bytes_to_read;
            bytes_remaining -= bytes_to_read;
        }
        if (ret != 0) break;

        if ((ret = mbedtls_md_finish(&md_ctx, hash)) != 0) break;

        ret = mbedtls_pk_parse_public_key(&pk_ctx, (const uint8_t *)PUBLIC_KEY_PEM, strlen(PUBLIC_KEY_PEM) + 1);
        if (ret != 0) {
            printf("[VERIFY] Public key parse failed: -0x%04X\n", -ret);
            break;
        }

        ret = mbedtls_pk_verify(&pk_ctx, MBEDTLS_MD_SHA256, hash, sizeof(hash), der_sig, der_len);
    } while (0);

    mbedtls_md_free(&md_ctx);
    mbedtls_pk_free(&pk_ctx);

    if (ret == 0) {
        return true;
    } else {
        printf("[VERIFY] Verification failed! Error code: -0x%04X\n", -ret);
        return false;
    }
}

//--------------------------------------------------------------------+
// DFU Callbacks
//--------------------------------------------------------------------+

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const *data, uint16_t length) {
    (void) alt;

    uint32_t flash_addr = block_num * CFG_TUD_DFU_XFER_BUFSIZE;

    if (block_num == 0) {
        dfu_bytes_received = 0;
        last_erased_sector_addr = 0xFFFFFFFF;
        printf("\n[DFU] Download started...\n");
    }

    if (length > 0) {
        uint32_t current_sector = flash_addr & ~(w25qxx.sector_size - 1);

        if (current_sector != last_erased_sector_addr) {
            w25qxx_erase(&w25qxx, current_sector, w25qxx.sector_size);
            last_erased_sector_addr = current_sector;
        }

        w25qxx_write(&w25qxx, flash_addr, (uint8_t*) data, length);
        dfu_bytes_received += length;
        printf("[DFU] Block %u written (%u bytes) at 0x%08lX\n", block_num, length, flash_addr);
    }

    tud_dfu_finish_flashing(DFU_STATUS_OK);
}

void tud_dfu_get_status_cb(uint8_t alt, uint8_t *status) {
    (void) alt;
    status[0] = DFU_STATUS_OK;
    status[1] = 0;
    status[2] = 0;
    status[3] = 0;
    status[4] = DFU_DNLOAD_IDLE;
    status[5] = 0;
}

uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state) {
    (void) alt;
    if (state == DFU_DNBUSY)
        return 10;
    return 0;
}

void tud_dfu_manifest_cb(uint8_t alt) {
    (void) alt;
    printf("[DFU] Download complete! Total received: %lu bytes.\n", dfu_bytes_received);

    Header128_t header;

    w25qxx_read(&w25qxx, 0x00000000, (uint8_t*) &header, sizeof(Header128_t));

    char magic_str[5] = { 0 };
    memcpy(magic_str, &header.magic, 4);

    printf("\n================ Header Info ================\n");
    printf("Magic Raw          : 0x%08lX\n", header.magic);
    printf("Magic ASCII        : %s\n", magic_str);
    printf("Payload Size       : %lu bytes\n", header.payload_len);

    printf("Signature (R)      : ");
    for (int i = 0; i < 32; i++) {
        printf("%02X", header.signature[i]);
    }
    printf("\n");

    printf("Signature (S)      : ");
    for (int i = 0; i < 32; i++) {
        printf("%02X", header.signature[32 + i]);
    }
    printf("\n");
    printf("=============================================\n");

    if (header.magic != 0x4150504B) {
        printf("[DFU] ERROR: Invalid Header Magic Identifier!\n\n");
        tud_dfu_finish_flashing(DFU_STATUS_ERR_FILE);
        return;
    }

    printf("[DFU] Verifying ECDSA SECP256R1 signature...\n");
    if (verify_flash_firmware_stream(sizeof(Header128_t), header.payload_len, header.signature)) {
        printf("[DFU] SUCCESS: Firmware signature VALID!\n\n");
        tud_dfu_finish_flashing(DFU_STATUS_OK);
    } else {
        printf("[DFU] ERROR: Firmware signature INVALID!\n\n");
        tud_dfu_finish_flashing(DFU_STATUS_ERR_VERIFY);
    }
}

void tud_dfu_runtime_reboot_to_dfu_cb(void) {

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

    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_SPI1_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_MBEDTLS_Init();

    /* USER CODE BEGIN 2 */

    printf("\n\n\n\n--------\nApplication\n");

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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{
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
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
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
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{
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
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI1_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
