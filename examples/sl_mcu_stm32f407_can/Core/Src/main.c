/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for STM32H723 FDCAN Loopback Test
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Our Message IDs
#define CAN_ID_UPT     0x5FC   // 0b10111111100 (1532)
#define CAN_ID_RND     0x5FD   // 0b10111111101 (1533)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t msg_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retarget printf to UART1
int __io_putchar(int ch) {
    if (ch == '\n') {
        HAL_UART_Transmit(&huart1, (uint8_t*) "\r", 1, HAL_MAX_DELAY);
    }
    if (HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    return ch;
}

/* FDCAN Callbacks */

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
    printf("HAL_FDCAN_TxBufferCompleteCallback (Buffer: 0x%08LX)\n", BufferIndexes);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
            printf("Error receiving on FDCAN1\n");
            Error_Handler();
        }

        if (hfdcan->Instance == FDCAN1) {
            // Case 1: Received a REMOTE request frame
            if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME) {
                if (RxHeader.Identifier == CAN_ID_UPT) {
                    printf("FDCAN1 received Remote Request for Uptime\n");

                    TxHeader.Identifier = CAN_ID_UPT;
                    TxHeader.IdType = FDCAN_STANDARD_ID;
                    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
                    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
                    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
                    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
                    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
                    TxHeader.MessageMarker = 0;

                    uint32_t upt = uwTick / 1000;
                    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t*) &upt) != HAL_OK) {
                        Error_Handler();
                    }
                } else if (RxHeader.Identifier == CAN_ID_RND) {
                    uint32_t rnd = HAL_RNG_GetRandomNumber(&hrng);
                    printf("FDCAN1 received Remote Request for Random: %lu\n", rnd);

                    TxHeader.Identifier = CAN_ID_RND;
                    TxHeader.IdType = FDCAN_STANDARD_ID;
                    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
                    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
                    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
                    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
                    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
                    TxHeader.MessageMarker = 0;

                    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t*) &rnd) != HAL_OK) {
                        Error_Handler();
                    }
                } else {
                    printf("Unknown remote request ID: 0x%03LX\n", RxHeader.Identifier);
                }
            }
            // Case 2: Received a DATA response frame
            else if (RxHeader.RxFrameType == FDCAN_DATA_FRAME) {
                uint32_t *val = (uint32_t*) &RxData[0];
                if (RxHeader.Identifier == CAN_ID_UPT) {
                    printf("FDCAN1 loopback received Uptime: %lu s\n", *val);
                } else if (RxHeader.Identifier == CAN_ID_RND) {
                    printf("FDCAN1 loopback received Random: %lu\n", *val);
                } else {
                    printf("Unknown data frame ID: 0x%03LX\n", RxHeader.Identifier);
                }
            }
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
    printf("HAL_FDCAN_ErrorCallback\n");
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
    MX_FDCAN1_Init();
    MX_RNG_Init();

    /* USER CODE BEGIN 2 */

    printf("\n\n\n\n--------\r\nStarting STM32H723 FDCAN Internal Loopback Test\n");

    // Configure FDCAN Global Filter to accept non-matching frames and standard remote frames
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    // Configure Filter for CAN_ID_UPT and CAN_ID_RND
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_ID_UPT;
    sFilterConfig.FilterID2 = CAN_ID_RND;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Activate interrupts for RX FIFO0 and Transmit completion
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                    FDCAN_IT_TX_COMPLETE |
                    FDCAN_IT_ERROR_PASSIVE |
                    FDCAN_IT_BUS_OFF,
            0) != HAL_OK) {
        Error_Handler();
    }

    // Start FDCAN peripheral
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t now = 0, loop_cnt = 0, next_blink = 500, next_tick = 1000, next_tx0 = 5000, next_tx1 = 10000;

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

        // Request Uptime via Remote Frame every 10s
        if (now >= next_tx0) {
            printf("FDCAN1 Requesting uptime via Remote Frame...\n");

            TxHeader.Identifier = CAN_ID_UPT;
            TxHeader.IdType = FDCAN_STANDARD_ID;
            TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
            TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
            TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            TxHeader.MessageMarker = 0;

            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, NULL) != HAL_OK) {
                Error_Handler();
            }

            next_tx0 = now + 10000;
        }

        // Request Random Number via Remote Frame every 10s
        if (now >= next_tx1) {
            printf("FDCAN1 Requesting random via Remote Frame...\n");

            TxHeader.Identifier = CAN_ID_RND;
            TxHeader.IdType = FDCAN_STANDARD_ID;
            TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
            TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
            TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            TxHeader.MessageMarker = 0;

            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, NULL) != HAL_OK) {
                Error_Handler();
            }

            next_tx1 = now + 10000;
        }

        ++loop_cnt;

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* Peripheral Init Functions generated by CubeMX */

static void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 1;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 11;
    hfdcan1.Init.NominalTimeSeg2 = 2;
    hfdcan1.Init.MessageRAMOffset = 0;
    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = 10;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr = 0;
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 10;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
            {
        Error_Handler();
    }
}

static void MX_RNG_Init(void)
{
    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) != HAL_OK)
            {
        Error_Handler();
    }
}

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

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
