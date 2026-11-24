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
#include "tusb.h"

#include "mbedtls/md.h"
#include "mbedtls/pk.h"
#include "mbedtls/bignum.h"
#include "mbedtls/asn1write.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint32_t magic;          // 4 bytes: Magic identifier (e.g. 0x4150504B / "KAPP")
    uint32_t payload_len;    // 4 bytes: Size of binary firmware in bytes
    uint8_t signature[64];  // 64 bytes: ECDSA raw signature (32-byte R + 32-byte S)
    uint8_t reserved[56];   // 56 bytes: Zero padding to align header to 128 bytes
} Header128_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DFU_BOOT_FLAG 0xDEADBEEF
#define FIRMWARE_MAGIC 0x4150504B
#define STAGING_ADDRESS 0x08080000              // Hidden flash, sectors 8-11
#define STAGING_SIZE (512 * 1024)
#define STAGING_SECTOR_SIZE (128 * 1024)
#define APP_MAX_SIZE (480 * 1024)               // Sectors 2-7
#define ERASE_TIMEOUT_MS 2000                   // Max 128 kB sector erase time (RM0090)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

extern int _bflag;
volatile static uint32_t *dfu_boot_flag;

static uint32_t dfu_bytes_received = 0;
static uint32_t last_erased_sector = 0xFFFFFFFF;

static uint8_t do_boot = 0;

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
        if ((ret = mbedtls_md_setup(&md_ctx, md_info, 0)) != 0)
            break;
        if ((ret = mbedtls_md_starts(&md_ctx)) != 0)
            break;

        // Staging area is memory mapped - hash it in place
        if ((ret = mbedtls_md_update(&md_ctx, (const uint8_t*) flash_payload_addr, payload_len)) != 0)
            break;

        if ((ret = mbedtls_md_finish(&md_ctx, hash)) != 0)
            break;

        ret = mbedtls_pk_parse_public_key(&pk_ctx, (const uint8_t*) PUBLIC_KEY_PEM, strlen(PUBLIC_KEY_PEM) + 1);
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

/**
 * @brief Erase one 128 kB staging sector (8-11). Stalls the CPU for 1-2 s.
 */
static bool erase_staging_sector(uint32_t sector) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = sector;
    EraseInitStruct.NbSectors = 1;

    return HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK;
}

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const *data, uint16_t length) {
    (void) alt;

    uint32_t offset = block_num * CFG_TUD_DFU_XFER_BUFSIZE;

    if (block_num == 0) {
        dfu_bytes_received = 0;
        last_erased_sector = 0xFFFFFFFF;
        printf("\n[DFU] Download started...\n");
    }

    if (length == 0) {
        tud_dfu_finish_flashing(DFU_STATUS_OK);
        return;
    }

    if (offset + length > STAGING_SIZE) {
        printf("[DFU] Image too big for staging area!\n");
        tud_dfu_finish_flashing(DFU_STATUS_ERR_ADDRESS);
        return;
    }

    HAL_FLASH_Unlock();

    // Erase the staging sector when the write first enters it
    uint32_t sector = FLASH_SECTOR_8 + offset / STAGING_SECTOR_SIZE;
    if (sector != last_erased_sector) {
        printf("[DFU] Erasing sector %lu...\n", sector);
        if (!erase_staging_sector(sector)) {
            HAL_FLASH_Lock();
            tud_dfu_finish_flashing(DFU_STATUS_ERR_ERASE);
            return;
        }
        last_erased_sector = sector;
    }

    // Program word by word, padding a trailing partial word with 0xFF
    uint32_t addr = STAGING_ADDRESS + offset;
    for (uint32_t i = 0; i < length; i += 4) {
        uint32_t word = 0xFFFFFFFF;
        memcpy(&word, data + i, (length - i < 4) ? (length - i) : 4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, word) != HAL_OK) {
            printf("[DFU] Programming failed at 0x%08lX!\n", addr + i);
            HAL_FLASH_Lock();
            tud_dfu_finish_flashing(DFU_STATUS_ERR_PROG);
            return;
        }
    }

    HAL_FLASH_Lock();

    dfu_bytes_received += length;
    printf("[DFU] Block %u written (%u bytes) at 0x%08lX\n", block_num, length, addr);

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
    if (state == DFU_DNBUSY) {
        // Next block starts a new staging sector - give the host time for the erase
        return (dfu_bytes_received % STAGING_SECTOR_SIZE == 0) ? ERASE_TIMEOUT_MS : 10;
    }
    return 0;
}

void tud_dfu_manifest_cb(uint8_t alt) {
    (void) alt;
    printf("[DFU] Download complete! Total received: %lu bytes.\n", dfu_bytes_received);

    const Header128_t *header = (const Header128_t*) STAGING_ADDRESS;

    char magic_str[5] = { 0 };
    memcpy(magic_str, &header->magic, 4);

    printf("\n================ Header Info ================\n");
    printf("Magic Raw          : 0x%08lX\n", header->magic);
    printf("Magic ASCII        : %s\n", magic_str);
    printf("Payload Size       : %lu bytes\n", header->payload_len);

    printf("Signature (R)      : ");
    for (int i = 0; i < 32; i++) {
        printf("%02X", header->signature[i]);
    }
    printf("\n");

    printf("Signature (S)      : ");
    for (int i = 0; i < 32; i++) {
        printf("%02X", header->signature[32 + i]);
    }
    printf("\n");
    printf("=============================================\n");

    if (header->magic != FIRMWARE_MAGIC) {
        printf("[DFU] ERROR: Invalid Header Magic Identifier!\n\n");
        tud_dfu_finish_flashing(DFU_STATUS_ERR_FILE);
        return;
    }

    if (header->payload_len == 0 || header->payload_len > APP_MAX_SIZE
            || sizeof(Header128_t) + header->payload_len > dfu_bytes_received) {
        printf("[DFU] ERROR: Invalid payload length!\n\n");
        tud_dfu_finish_flashing(DFU_STATUS_ERR_FILE);
        return;
    }

    printf("[DFU] Verifying ECDSA SECP256R1 signature...\n");
    if (verify_flash_firmware_stream(STAGING_ADDRESS + sizeof(Header128_t), header->payload_len, header->signature)) {
        printf("[DFU] SUCCESS: Firmware signature VALID!\n\n");
        tud_dfu_finish_flashing(DFU_STATUS_OK);

        // Reboot into bootloader
        do_boot = 1;

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
    dfu_boot_flag = (uint32_t*) (&_bflag); // set in linker script
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
    MX_USB_OTG_FS_PCD_Init();
    MX_MBEDTLS_Init();

    /* USER CODE BEGIN 2 */

    printf("\n\n\n\n--------\nApplication test 2\n");

    tusb_init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    register uint32_t now = 0, loop_cnt = 0, next_blink = 500, next_tick = 1000;

    while (1) {

        now = uwTick;

        if (do_boot) {
            // Wait a bit until the dfu usb is finished
            uint32_t wait_end = uwTick + 1500;
            while (uwTick <= wait_end) {
                tud_task();
            }

            // Boot into the bootloader
            *dfu_boot_flag = DFU_BOOT_FLAG;
            HAL_NVIC_SystemReset();
        }

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

    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
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
