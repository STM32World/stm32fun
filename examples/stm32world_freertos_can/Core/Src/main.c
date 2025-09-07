/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lth@stm32world.com>
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "freertos_extras.h"
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
        .name = "defaultTask",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
        .name = "ledTask",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tickTask */
osThreadId_t tickTaskHandle;
const osThreadAttr_t tickTask_attributes = {
        .name = "tickTask",
        .stack_size = 196 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for statsTask */
osThreadId_t statsTaskHandle;
const osThreadAttr_t statsTask_attributes = {
        .name = "statsTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canHandler1 */
osThreadId_t canHandler1Handle;
const osThreadAttr_t canHandler1_attributes = {
        .name = "canHandler1",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canHandler2 */
osThreadId_t canHandler2Handle;
const osThreadAttr_t canHandler2_attributes = {
        .name = "canHandler2",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canClean */
osThreadId_t canCleanHandle;
const osThreadAttr_t canClean_attributes = {
        .name = "canClean",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tickQueue */
osMessageQueueId_t tickQueueHandle;
const osMessageQueueAttr_t tickQueue_attributes = {
        .name = "tickQueue"
};
/* Definitions for canMessage */
osMessageQueueId_t canMessageHandle;
const osMessageQueueAttr_t canMessage_attributes = {
        .name = "canMessage"
};
/* Definitions for ledMutex */
osMutexId_t ledMutexHandle;
const osMutexAttr_t ledMutex_attributes = {
        .name = "ledMutex"
};
/* Definitions for printMutex */
osMutexId_t printMutexHandle;
const osMutexAttr_t printMutex_attributes = {
        .name = "printMutex"
};
/* Definitions for ledSemaphore */
osSemaphoreId_t ledSemaphoreHandle;
const osSemaphoreAttr_t ledSemaphore_attributes = {
        .name = "ledSemaphore"
};
/* Definitions for canHandlerEvent */
osEventFlagsId_t canHandlerEventHandle;
const osEventFlagsAttr_t canHandlerEvent_attributes = {
        .name = "canHandlerEvent"
};
/* USER CODE BEGIN PV */

volatile unsigned long ulHighFrequencyTimerTicks;

// Pre-allocate all the heap for FreeRTOS.  FreeRTOS will do this by itself if this define is
// not set.
#ifdef configAPPLICATION_ALLOCATED_HEAP
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram"))); // Put in ccmram
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartTickTask(void *argument);
void StartStatsTask(void *argument);
void startCanHandler1(void *argument);
void startCanHandler2(void *argument);
void startCanClean(void *argument);

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

void configureTimerForRunTimeStats(void) {
    ulHighFrequencyTimerTicks = 0;
    HAL_TIM_Base_Start_IT(&htim13);
}

unsigned long getRunTimeCounterValue(void) {
    return ulHighFrequencyTimerTicks;
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
    MX_TIM13_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    /* USER CODE BEGIN 2 */

    printf("\n\n\n--------\nStarting\n");

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();
    /* Create the mutex(es) */
    /* creation of ledMutex */
    ledMutexHandle = osMutexNew(&ledMutex_attributes);

    /* creation of printMutex */
    printMutexHandle = osMutexNew(&printMutex_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of ledSemaphore */
    ledSemaphoreHandle = osSemaphoreNew(1, 1, &ledSemaphore_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of tickQueue */
    tickQueueHandle = osMessageQueueNew(16, sizeof(uint32_t), &tickQueue_attributes);

    /* creation of canMessage */
    canMessageHandle = osMessageQueueNew(16, sizeof(uint32_t), &canMessage_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of ledTask */
    ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

    /* creation of tickTask */
    tickTaskHandle = osThreadNew(StartTickTask, NULL, &tickTask_attributes);

    /* creation of statsTask */
    statsTaskHandle = osThreadNew(StartStatsTask, NULL, &statsTask_attributes);

    /* creation of canHandler1 */
    canHandler1Handle = osThreadNew(startCanHandler1, NULL, &canHandler1_attributes);

    /* creation of canHandler2 */
    canHandler2Handle = osThreadNew(startCanHandler2, NULL, &canHandler2_attributes);

    /* creation of canClean */
    canCleanHandle = osThreadNew(startCanClean, NULL, &canClean_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* creation of canHandlerEvent */
    canHandlerEventHandle = osEventFlagsNew(&canHandlerEvent_attributes);

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
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
    RCC_OscInitStruct.PLL.PLLQ = 4;
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
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void)
{

    /* USER CODE BEGIN TIM13_Init 0 */

    /* USER CODE END TIM13_Init 0 */

    /* USER CODE BEGIN TIM13_Init 1 */

    /* USER CODE END TIM13_Init 1 */
    htim13.Instance = TIM13;
    htim13.Init.Prescaler = 10 - 1;
    htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim13.Init.Period = 840 - 1;
    htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM13_Init 2 */

    /* USER CODE END TIM13_Init 2 */

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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN 5 */

    uint32_t loop_cnt = 0;

    /* Infinite loop */
    for (;;)
            {
        osDelay(100);

        if (loop_cnt % 10 == 0) {

            uint32_t tick = osKernelGetTickCount();

            osMessageQueuePut(tickQueueHandle, &tick, 0, osWaitForever);

        }

        if (loop_cnt % 5 == 0) {

            osSemaphoreRelease(ledSemaphoreHandle);

        }

        if (loop_cnt % 20 == 0) {
            osEventFlagsSet(canHandlerEventHandle, 0b00000001);
        }

        if (loop_cnt % 40 == 0) {
            osEventFlagsSet(canHandlerEventHandle, 0b00000010);
        }

        ++loop_cnt;
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
 * @brief Function implementing the ledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
    /* USER CODE BEGIN StartLedTask */

    osStatus_t ret;

    /* Infinite loop */
    for (;;) {

        ret = osSemaphoreAcquire(ledSemaphoreHandle, osWaitForever);

        if (!ret) {

            osMutexWait(ledMutexHandle, osWaitForever);

            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            osMutexRelease(ledMutexHandle);

        }

    }
    /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartTickTask */
/**
 * @brief Function implementing the tickTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTickTask */
void StartTickTask(void *argument)
{
    /* USER CODE BEGIN StartTickTask */

    osStatus_t ret;

    /* Infinite loop */
    for (;;) {

        uint32_t tick;

        ret = osMessageQueueGet(tickQueueHandle, &tick, NULL, osWaitForever);

        if (ret == osOK) {

            osMutexWait(printMutexHandle, osWaitForever);

            printf("Tick %lu \n", tick / 1000);

            osMutexRelease(printMutexHandle);

        }

    }

    /* USER CODE END StartTickTask */
}

/* USER CODE BEGIN Header_StartStatsTask */
/**
 * @brief Function implementing the statsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStatsTask */
void StartStatsTask(void *argument)
{
    /* USER CODE BEGIN StartStatsTask */

    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    unsigned long ulTotalRunTime;
    float runtime_percentage;

    /* Infinite loop */
    for (;;) {

        osDelay(10000);

        uxArraySize = uxTaskGetNumberOfTasks();
        pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)); // a little bit scary!

        osMutexWait(printMutexHandle, osWaitForever);

        if (pxTaskStatusArray != NULL) {

            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize,
                    &ulTotalRunTime);

            printf("Task count = %lu\n", uxArraySize);
            printf("No       Name          P  S   Usage     Count     HW\n");

            for (x = 0; x < uxArraySize; x++) {

                runtime_percentage = (float) (100
                        * (float) pxTaskStatusArray[x].ulRunTimeCounter
                        / (float) ulTotalRunTime);

                printf("Task %2lu: %-12s %2lu %2d %8.4f (%8lu) %4i\n",
                        x,
                        pxTaskStatusArray[x].pcTaskName,
                        pxTaskStatusArray[x].uxCurrentPriority,
                        pxTaskStatusArray[x].eCurrentState,
                        runtime_percentage,
                        pxTaskStatusArray[x].ulRunTimeCounter,
                        pxTaskStatusArray[x].usStackHighWaterMark);

            }

            vPortFree(pxTaskStatusArray);

        } else {
            printf("Unable to allocate stack space\n");
        }

        osMutexRelease(printMutexHandle);

    }

    /* USER CODE END StartStatsTask */
}

/* USER CODE BEGIN Header_startCanHandler1 */
/**
 * @brief Function implementing the canHandler1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCanHandler1 */
void startCanHandler1(void *argument)
{
    /* USER CODE BEGIN startCanHandler1 */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
//        ret = osMessagePeek(canQueueHandle);
//        ret = osMessageQueueGet(tickQueueHandle, &tick, NULL, osWaitForever);
    }
    /* USER CODE END startCanHandler1 */
}

/* USER CODE BEGIN Header_startCanHandler2 */
/**
 * @brief Function implementing the canHandler2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCanHandler2 */
void startCanHandler2(void *argument)
{
    /* USER CODE BEGIN startCanHandler2 */
    /* Infinite loop */
    for (;;)
            {
        osDelay(1);
    }
    /* USER CODE END startCanHandler2 */
}

/* USER CODE BEGIN Header_startCanClean */
/**
 * @brief Function implementing the canClean thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCanClean */
void startCanClean(void *argument)
{
    /* USER CODE BEGIN startCanClean */
    /* Infinite loop */
    for (;;) {

        osEventFlagsWait(canHandlerEventHandle, 0b00000011, osFlagsWaitAll, osWaitForever);

        osMutexWait(printMutexHandle, osWaitForever);
        printf("Got Event Flags\n");
        osMutexRelease(printMutexHandle);

        osEventFlagsClear(canHandlerEventHandle, 0b00000011);
    }
    /* USER CODE END startCanClean */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

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
