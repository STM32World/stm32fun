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
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint16_t *buffer;
    float angle;
    float angle_change;
    float amplification;
} sine_queue_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_BUFFER_SIZE 64
#define SAMPLE_FREQ 100000
#define OUTPUT_MID 2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
DMA_HandleTypeDef hdma_dac2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
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
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for statusTask */
osThreadId_t statusTaskHandle;
const osThreadAttr_t statusTask_attributes = {
        .name = "statusTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pulseTask */
osThreadId_t pulseTaskHandle;
const osThreadAttr_t pulseTask_attributes = {
        .name = "pulseTask",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sineTask */
osThreadId_t sineTaskHandle;
const osThreadAttr_t sineTask_attributes = {
        .name = "sineTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tickQueue */
osMessageQueueId_t tickQueueHandle;
const osMessageQueueAttr_t tickQueue_attributes = {
        .name = "tickQueue"
};
/* Definitions for sineQueue */
osMessageQueueId_t sineQueueHandle;
const osMessageQueueAttr_t sineQueue_attributes = {
        .name = "sineQueue"
};
/* Definitions for printMutex */
osMutexId_t printMutexHandle;
const osMutexAttr_t printMutex_attributes = {
        .name = "printMutex"
};
/* Definitions for ledMutex */
osMutexId_t ledMutexHandle;
const osMutexAttr_t ledMutex_attributes = {
        .name = "ledMutex"
};
/* Definitions for ledSemaphore */
osSemaphoreId_t ledSemaphoreHandle;
const osSemaphoreAttr_t ledSemaphore_attributes = {
        .name = "ledSemaphore"
};
/* Definitions for pulseSemaphore */
osSemaphoreId_t pulseSemaphoreHandle;
const osSemaphoreAttr_t pulseSemaphore_attributes = {
        .name = "pulseSemaphore"
};
/* USER CODE BEGIN PV */

volatile unsigned long ulHighFrequencyTimerTicks;

#ifdef configAPPLICATION_ALLOCATED_HEAP
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram")));
#endif

uint16_t dma_buffer_1[2 * DMA_BUFFER_SIZE];
uint16_t dma_buffer_2[2 * DMA_BUFFER_SIZE];

sine_queue_t dacs[2] = {
        {
                &dma_buffer_1[0],
                0,
                1000 * (2 * M_PI / SAMPLE_FREQ),
                0.99
        },
        {
                &dma_buffer_2[0],
                0,
                999 * (2 * M_PI / SAMPLE_FREQ),
                0.50
        }

};

const float two_pi = 2 * M_PI;

//float angle = 0;
//float angle_change = 440 * (2 * M_PI / SAMPLE_FREQ);
//float amplifier = 0.9;

//osMessageQueueId_t sineQueueHandle;
//const osMessageQueueAttr_t sineQueue_attributes = {
//        .name = "sineQueue"
//};

uint32_t conv_half_ch1, conv_ch1, conv_half_ch2, conv_ch2, sine_task;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM4_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartTickTask(void *argument);
void StartStatusTask(void *argument);
void StartPulseTask(void *argument);
void StartSineTask(void *argument);

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

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    ++conv_half_ch1;
    dacs[0].buffer = &dma_buffer_1[0];
    uint32_t dac_addr = (uint32_t) &dacs[0];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    ++conv_ch1;
    dacs[0].buffer = &dma_buffer_1[DMA_BUFFER_SIZE];
    uint32_t dac_addr = (uint32_t) &dacs[0];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    ++conv_half_ch2;
    dacs[1].buffer = &dma_buffer_2[0];
    uint32_t dac_addr = (uint32_t) &dacs[1];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    ++conv_ch2;
    dacs[1].buffer = &dma_buffer_2[DMA_BUFFER_SIZE];
    uint32_t dac_addr = (uint32_t) &dacs[1];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
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
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_TIM13_Init();
    MX_TIM4_Init();
    MX_DAC_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */

    printf("\n\n\n--------\nStarting\n");

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim6);

    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) &dma_buffer_1, 2 * DMA_BUFFER_SIZE, DAC_ALIGN_12B_R);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*) &dma_buffer_2, 2 * DMA_BUFFER_SIZE, DAC_ALIGN_12B_R);

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();
    /* Create the mutex(es) */
    /* creation of printMutex */
    printMutexHandle = osMutexNew(&printMutex_attributes);

    /* creation of ledMutex */
    ledMutexHandle = osMutexNew(&ledMutex_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of ledSemaphore */
    ledSemaphoreHandle = osSemaphoreNew(1, 1, &ledSemaphore_attributes);

    /* creation of pulseSemaphore */
    pulseSemaphoreHandle = osSemaphoreNew(1, 1, &pulseSemaphore_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of tickQueue */
    tickQueueHandle = osMessageQueueNew(16, sizeof(uint32_t), &tickQueue_attributes);

    /* creation of sineQueue */
    sineQueueHandle = osMessageQueueNew(16, sizeof(uint32_t), &sineQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* creation of sineQueue */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of ledTask */
    ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

    /* creation of tickTask */
    tickTaskHandle = osThreadNew(StartTickTask, NULL, &tickTask_attributes);

    /* creation of statusTask */
    statusTaskHandle = osThreadNew(StartStatusTask, NULL, &statusTask_attributes);

    /* creation of pulseTask */
    pulseTaskHandle = osThreadNew(StartPulseTask, NULL, &pulseTask_attributes);

    /* creation of sineTask */
    sineTaskHandle = osThreadNew(StartSineTask, NULL, &sineTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    Error_Handler();

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
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void)
{

    /* USER CODE BEGIN DAC_Init 0 */

    /* USER CODE END DAC_Init 0 */

    DAC_ChannelConfTypeDef sConfig = { 0 };

    /* USER CODE BEGIN DAC_Init 1 */

    /* USER CODE END DAC_Init 1 */

    /** DAC Initialization
     */
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
            {
        Error_Handler();
    }

    /** DAC channel OUT1 config
     */
    sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
            {
        Error_Handler();
    }

    /** DAC channel OUT2 config
     */
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC_Init 2 */

    /* USER CODE END DAC_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 84 - 1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 100 - 1;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
            {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
            {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 84 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 10 - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
            {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */

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
    htim13.Init.Prescaler = 0;
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
    huart1.Init.OverSampling = UART_OVERSAMPLING_8;
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
    /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    /* DMA1_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED_Pin PULSE_Pin */
    GPIO_InitStruct.Pin = LED_Pin | PULSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

    uint8_t toggle = 0;

    /* Infinite loop */
    for (;;)
            {
        osDelay(500);

        toggle = !toggle;

        osSemaphoreRelease(ledSemaphoreHandle);

        uint32_t tick = osKernelGetTickCount();

        if (!toggle) { // Only every second time
            osMessageQueuePut(tickQueueHandle, &tick, 0, osWaitForever);
        }

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
            printf("Tick %lu (c1 = %lu, c1h = %lu ch2 = %lu, c2h = %lu s = %lu)\n", tick / 1000, conv_ch1, conv_half_ch1, conv_ch2, conv_half_ch2, sine_task);
            osMutexRelease(printMutexHandle);

        }

    }

    /* USER CODE END StartTickTask */
}

/* USER CODE BEGIN Header_StartStatusTask */
/**
 * @brief Function implementing the statusTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStatusTask */
void StartStatusTask(void *argument)
{
    /* USER CODE BEGIN StartStatusTask */
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
            printf("No      Name          S   Usage     Count     HW\n");

            for (x = 0; x < uxArraySize; x++) {

                runtime_percentage = (float) (100
                        * (float) pxTaskStatusArray[x].ulRunTimeCounter
                        / (float) ulTotalRunTime);

                printf("Task %lu: %-12s %2d %8.4f (%8lu) %4i\n",
                        x,
                        pxTaskStatusArray[x].pcTaskName,
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

    /* USER CODE END StartStatusTask */
}

/* USER CODE BEGIN Header_StartPulseTask */
/**
 * @brief Function implementing the pulseTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPulseTask */
void StartPulseTask(void *argument)
{
    /* USER CODE BEGIN StartPulseTask */

    osStatus_t ret;

    /* Infinite loop */
    for (;;) {

        ret = osSemaphoreAcquire(pulseSemaphoreHandle, osWaitForever);

        if (!ret) {
            HAL_GPIO_TogglePin(PULSE_GPIO_Port, PULSE_Pin);
        }

    }

    /* USER CODE END StartPulseTask */
}

/* USER CODE BEGIN Header_StartSineTask */
/**
 * @brief Function implementing the sineTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSineTask */
void StartSineTask(void *argument)
{
    /* USER CODE BEGIN StartSineTask */

    osStatus_t ret;

    uint32_t sine_p;

    /* Infinite loop */
    for (;;) {

        ret = osMessageQueueGet(sineQueueHandle, &sine_p, 0, osWaitForever);

        if (ret == osOK) {
            ++sine_task;

            // The message contain a pointer to the relevant sine_queue_t struct
            sine_queue_t *sine = (sine_queue_t*) sine_p;

            // Run through the buffer
            for (int i = 0; i < DMA_BUFFER_SIZE; ++i) {

                // Since we got the angle, calculating the dac value is quite simple.  The arm_cos_f32 function
                // is blindingly fast compared with the fpu
                sine->buffer[i] = OUTPUT_MID - (sine->amplification * (OUTPUT_MID * arm_cos_f32(sine->angle)));

                // Prepare angle for next point simply by adding the pre-calculated angle change (based on frequency)
                sine->angle += sine->angle_change;

                // If we've gone full circle - cycle back
                if (sine->angle >= two_pi) {
                    sine->angle -= two_pi;
                }
            }

        }

    }

    /* USER CODE END StartSineTask */
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

    if (htim->Instance == TIM4) {
        osSemaphoreRelease(pulseSemaphoreHandle);
    }

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
