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
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Example filter
//#define CAN_RX_ID    0b10111111100
//#define CAN_RX_MASK  0b11111111100
// Our Message IDs
#define CAN_ID_UPT     0b10111111100   // 0x5fc - 1532
#define CAN_ID_RND     0b10111111101   // 0x5fd - 1533

#define CAN_FILTER_ID_REMOTE 0b00010000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t msg_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_RNG_Init(void);
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

// CAN Callbacks

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_TxMailbox0CompleteCallback\n");
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_TxMailbox1CompleteCallback\n");
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_TxMailbox2CompleteCallback\n");
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_TxMailbox0AbortCallback\n");
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_TxMailbox1AbortCallback\n");
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_TxMailbox2AbortCallback\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_RxFifo0MsgPendingCallback\n");
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        printf("Error receiving\n");
        Error_Handler();
        //printf("Got message %lu - id=0x%04lx type=%lx len=0x%lx, data=%02x%02x%02x%02x%02x%02x%02x%02x\n", msg_count + 1, RxHeader.StdId, RxHeader.RTR, RxHeader.DLC, RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
    }

    if (hcan->Instance == CAN1) {
        printf("Got message on CAN1\n");
        if (RxHeader.RTR == CAN_RTR_REMOTE) {
            if (RxHeader.StdId == CAN_ID_UPT) {

                printf("CAN1 transmitting Uptime Message\n");

                TxHeader.DLC = 4;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.StdId = CAN_ID_UPT;

                uint32_t upt = uwTick / 1000;
                uint32_t mb0;
                if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*) &upt, &mb0) != HAL_OK) {
                    Error_Handler();
                }
            } else if (RxHeader.StdId == CAN_ID_RND) {

                printf("CAN1 transmitting Random Message\n");

                TxHeader.DLC = 4;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.StdId = CAN_ID_RND;

                uint32_t rnd = HAL_RNG_GetRandomNumber(&hrng);

                uint32_t mb0;
                if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*) &rnd, &mb0) != HAL_OK) {
                    Error_Handler();
                }
            } else {
                printf("Unknown remote request on can 1\n");
            }
        } else {
            printf("Got data message on can1 - should not happen\n");
        }
    }

}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_RxFifo0FullCallback\n");
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_RxFifo1MsgPendingCallback\n");
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK) {
        printf("Error receiving\n");
        Error_Handler();
    }

    uint32_t *i = (uint32_t*) &RxData[0];

    if (hcan->Instance == CAN2) {
        printf("Got message on CAN2\n");
        if (RxHeader.RTR == CAN_RTR_DATA) {
            if (RxHeader.StdId == CAN_ID_UPT) {
                printf("CAN2 received uptime: %lu\n", *i);
            } else if (RxHeader.StdId == CAN_ID_RND) {
                printf("CAN2 received random: %lu\n", *i);
            } else {
                printf("Unknown remote request on can 1\n");
            }
        } else {
            printf("Got remote message on CAN2 - should not happen\n");
        }
    } else {
        printf("Got remote message on CAN1 - should not happen\n");
    }
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_RxFifo1FullCallback\n");
}

void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_SleepCallback\n");
}

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_WakeUpFromRxMsgCallback\n");
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    printf("HAL_CAN_ErrorCallback\n");
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
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_RNG_Init();
    /* USER CODE BEGIN 2 */

    printf("\n\n\n--------\nStarting\n");

    CAN_FilterTypeDef canfilterconfig;

    // Create filter for CAN1 - only REMOTE messages will be received
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0;  // anything between 0 to SlaveStartFilterBank
    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterIdLow = (CAN_ID_UPT << 5) | CAN_FILTER_ID_REMOTE;
    canfilterconfig.FilterIdHigh = (CAN_ID_RND << 5) | CAN_FILTER_ID_REMOTE;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

    // Create filter for CAN2 - only DATA messages will be received
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 13;  // anything between 0 to SlaveStartFilterBank
    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    canfilterconfig.FilterIdLow = CAN_ID_UPT << 5;
    canfilterconfig.FilterIdHigh = CAN_ID_RND << 5;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;

    HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_RX_FIFO0_FULL |
            CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_RX_FIFO1_MSG_PENDING |
            CAN_IT_RX_FIFO1_FULL |
            CAN_IT_RX_FIFO1_OVERRUN |
            CAN_IT_WAKEUP |
            CAN_IT_SLEEP_ACK |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR
            );

    HAL_CAN_ActivateNotification(
            &hcan2,
            CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_RX_FIFO0_FULL |
            CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_RX_FIFO1_MSG_PENDING |
            CAN_IT_RX_FIFO1_FULL |
            CAN_IT_RX_FIFO1_OVERRUN |
            CAN_IT_WAKEUP |
            CAN_IT_SLEEP_ACK |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR
            );

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t now = 0, loop_cnt = 0, next_blink = 500, next_tick = 1000, next_tx0 = 5000, next_tx1 = 10000;

    while (1) {

        now = uwTick;

        if (now >= next_blink) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            next_blink = now + 500;
        }

        if (now >= next_tick) {
            printf("Tick %lu (loop=%lu)\n", now / 1000, loop_cnt);
            loop_cnt = 0;
            next_tick = now + 1000;
        }

        if (now >= next_tx0) {

            printf("CAN2 Requesting uptime\n");

            TxHeader.DLC = 4;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.RTR = CAN_RTR_REMOTE;
            TxHeader.StdId = CAN_ID_UPT;

            uint32_t mb0;
            if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, NULL, &mb0) != HAL_OK) {
                Error_Handler();
            }

            next_tx0 = now + 10000;
        }

        if (now >= next_tx1) {

            printf("CAN2 Requesting random\n");

            TxHeader.DLC = 4;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.RTR = CAN_RTR_REMOTE;
            TxHeader.StdId = CAN_ID_RND;

            uint32_t mb0;
            if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, NULL, &mb0) != HAL_OK) {
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
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    /* USER CODE END CAN1_Init 2 */

}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

    /* USER CODE BEGIN CAN2_Init 0 */

    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 3;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = DISABLE;
    hcan2.Init.AutoWakeUp = DISABLE;
    hcan2.Init.AutoRetransmission = DISABLE;
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */

    /* USER CODE END CAN2_Init 2 */

}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void)
{

    /* USER CODE BEGIN RNG_Init 0 */

    /* USER CODE END RNG_Init 0 */

    /* USER CODE BEGIN RNG_Init 1 */

    /* USER CODE END RNG_Init 1 */
    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN RNG_Init 2 */

    /* USER CODE END RNG_Init 2 */

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
