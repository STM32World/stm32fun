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

#include "lwip/apps/fs.h"
#include "lwip/apps/httpd.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"

#include "dhcps.h"

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
UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* TinyUSB MAC address definition */
uint8_t tud_network_mac_address[6] = { 0x02, 0x00, 0x00, 0x12, 0x34, 0x56 };

static struct netif netif_data;

// Buffer to store generated HTML response
static char http_response_buf[2048];

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

uint32_t sys_now(void) {
    return uwTick;
}

/* Callback: Output frame from lwIP down to TinyUSB */
static err_t linkoutput_fn(struct netif *netif, struct pbuf *p) {
    (void) netif;

    // Check if TinyUSB can accept a network packet
    if (!tud_network_can_xmit(p->tot_len)) {
        return ERR_MEM;
    }

    // Allocate buffer and copy payload
    tud_network_xmit(p, 0); // TinyUSB provides zero-copy or callback handling
    return ERR_OK;
}

/* Callback: Called by TinyUSB when lwIP sends packet */
uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    struct pbuf *p = (struct pbuf*) ref;
    return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

/* Init low-level netif hardware interface */
static err_t ip_init_cb(struct netif *netif) {
    netif->linkoutput = linkoutput_fn;
    netif->output = etharp_output;
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    memcpy(netif->hwaddr, tud_network_mac_address, 6);
    netif->hwaddr_len = 6;
    return ERR_OK;
}

/* Callback: TinyUSB received a packet from Host */
bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    if (size == 0)
        return true;

    // Allocate an lwIP pbuf for incoming packet
    struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
    if (p) {
        pbuf_take(p, src, size);
        // Pass packet into lwIP stack
        if (netif_data.input(p, &netif_data) != ERR_OK) {
            pbuf_free(p);
        }
    }
    tud_network_recv_renew(); // Signal TinyUSB to receive next packet
    return true;
}

void tud_network_init_cb(void) {
    // Called when USB Network interface is initialized
}

/* Reads the 96-bit STM32F407 Unique ID register and formats it as a hex string */
static void get_stm32f407_uid_str(char *buf, size_t max_len) {
    uint32_t *uid = (uint32_t*) UID_BASE;
    snprintf(buf, max_len, "%08X%08X%08X", (unsigned int) uid[0], (unsigned int) uid[1], (unsigned int) uid[2]);
}

/* Custom fs_open_custom handler for HTML shell and JSON APIs */
int fs_open_custom(struct fs_file *file, const char *name) {

    // --- LED TOGGLE ENDPOINT: /api/led/toggle ---
    if (strcmp(name, "/api/led/toggle") == 0) {

        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

        GPIO_PinState pin_state = HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin);

        int len = snprintf(http_response_buf, sizeof(http_response_buf),
                "{\"status\":\"ok\",\"led\":%d}",
                pin_state == GPIO_PIN_SET ? 1 : 0);

        memset(file, 0, sizeof(struct fs_file));
        file->data = http_response_buf;
        file->len = len;
        file->index = len;
        file->flags = FS_FILE_FLAGS_CUSTOM;
        return 1;
    }

    // --- JSON API ENDPOINT: /api/stats ---
    if (strcmp(name, "/api/stats") == 0) {
        char id_str[25];
        get_stm32f407_uid_str(id_str, sizeof(id_str));

        uint32_t sys_clk_mhz = SystemCoreClock / 1000000;
        uint32_t uptime_sec = HAL_GetTick() / 1000;

        int len = snprintf(http_response_buf, sizeof(http_response_buf), "{"
                "\"architecture\":\"ARM Cortex-M4F\","
                "\"clock_mhz\":%lu,"
                "\"board_id\":\"%s\","
                "\"uptime_seconds\":%lu"
                "}",
                (unsigned long) sys_clk_mhz,
                id_str,
                (unsigned long) uptime_sec);

        memset(file, 0, sizeof(struct fs_file));
        file->data = http_response_buf;
        file->len = len;
        file->index = len;
        file->flags = FS_FILE_FLAGS_CUSTOM;
        return 1;
    }

    // --- HTML WEB PAGE: Dynamic Shell with LED Control Button ---
    if (strcmp(name, "/index.html") == 0 || strcmp(name, "/") == 0) {
        int len = snprintf(http_response_buf, sizeof(http_response_buf),
                "<!DOCTYPE html><html><head><title>STM32 Info</title>"
                "<style>"
                "body{font-family:sans-serif;margin:40px;background:#1a1a1a;color:#eee}"
                "h1{color:#e6005c}.card{background:#2a2a2a;padding:20px;border-radius:8px;"
                "max-width:500px;box-shadow:0 4px 10px rgba(0,0,0,0.5)}"
                "code{background:#333;padding:2px 6px;border-radius:4px;color:#00ffcc}"
                "button{background:#e6005c;color:#fff;border:none;padding:10px 18px;"
                "font-size:14px;border-radius:4px;cursor:pointer;margin-top:10px;font-weight:bold}"
                "button:hover{background:#ff1a75}"
                "</style>"
                "<script>"
                "async function updateStats(){"
                "try{"
                "let r = await fetch('/api/stats');"
                "let d = await r.json();"
                "document.getElementById('arch').innerText = d.architecture;"
                "document.getElementById('clk').innerText = d.clock_mhz + ' MHz';"
                "document.getElementById('id').innerText = d.board_id;"
                "document.getElementById('up').innerText = d.uptime_seconds + ' s';"
                "}catch(e){console.error(e);}"
                "}"
                "async function toggleLed(){"
                "try{"
                "await fetch('/api/led/toggle');"
                "}catch(e){console.error(e);}"
                "}"
                "window.onload = () => {"
                "updateStats();"
                "setInterval(updateStats, 1000);"
                "};"
                "</script></head><body>"
                "<div class='card'><h1>STM32 System Info</h1>"
                "<p><b>Architecture:</b> <span id='arch'>Loading...</span></p>"
                "<p><b>System Clock:</b> <span id='clk'>Loading...</span></p>"
                "<p><b>Unique Board ID:</b> <code id='id'>Loading...</code></p>"
                "<p><b>System Uptime:</b> <span id='up'>Loading...</span></p>"
                "<button onclick='toggleLed()'>Toggle LED</button>"
                "</div></body></html>");

        memset(file, 0, sizeof(struct fs_file));
        file->data = http_response_buf;
        file->len = len;
        file->index = len;
        file->flags = FS_FILE_FLAGS_CUSTOM;
        return 1;
    }

    return 0; // Fallback for unhandled paths
}

void fs_close_custom(struct fs_file *file) {
    (void) file;
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

    printf("\n\n\n--------\nStarting USB Networking Example\n");

    // Initialize the TinyUSB Device stack
    tusb_init();

    lwip_init();

    // Setup network interface IP for RP2350 (192.168.7.1)
    ip4_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr, 192, 168, 10, 1);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 10, 1);

    netif_add(&netif_data, &ipaddr, &netmask, &gw, NULL, ip_init_cb, netif_input);
    netif_set_default(&netif_data);
    netif_set_up(&netif_data);

    // Initialize DHCP Server!
    dhcps_init(&ipaddr, &netmask);

    // Initialize HTTP Server
    httpd_init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    register uint32_t now = 0, loop_cnt = 0, next_blink = 500, next_tick = 1000;

    while (1) {

        now = uwTick;

//        if (now >= next_blink) {
//            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//            next_blink += 500;
//        }

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
