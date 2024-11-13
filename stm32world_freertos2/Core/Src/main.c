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

IWDG_HandleTypeDef hiwdg;

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
/* Definitions for sineTask */
osThreadId_t sineTaskHandle;
const osThreadAttr_t sineTask_attributes = {
        .name = "sineTask",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for wdgTask */
osThreadId_t wdgTaskHandle;
const osThreadAttr_t wdgTask_attributes = {
        .name = "wdgTask",
        .stack_size = 128 * 4,
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
/* USER CODE BEGIN PV */

volatile unsigned long ulHighFrequencyTimerTicks;

#ifdef configAPPLICATION_ALLOCATED_HEAP
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram")));
#endif

//uint8_t buf[32232] __attribute__((section(".ccmram")));

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
                999.9 * (2 * M_PI / SAMPLE_FREQ),
                0.75
        }

};

const float two_pi = 2 * M_PI;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM13_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartTickTask(void *argument);
void StartStatsTask(void *argument);
void StartSineTask(void *argument);
void StartIwdgTask(void *argument);

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

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    //++conv_half_ch1;
    dacs[0].buffer = &dma_buffer_1[0];
    uint32_t dac_addr = (uint32_t) &dacs[0];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    //++conv_ch1;
    dacs[0].buffer = &dma_buffer_1[DMA_BUFFER_SIZE];
    uint32_t dac_addr = (uint32_t) &dacs[0];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    //++conv_half_ch2;
    dacs[1].buffer = &dma_buffer_2[0];
    uint32_t dac_addr = (uint32_t) &dacs[1];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    //++conv_ch2;
    dacs[1].buffer = &dma_buffer_2[DMA_BUFFER_SIZE];
    uint32_t dac_addr = (uint32_t) &dacs[1];
    osMessageQueuePut(sineQueueHandle, &dac_addr, 0, 0);
}

/// @brief  Possible STM32 system reset causes
typedef enum reset_cause_e
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_LOW_POWER_RESET,
    RESET_CAUSE_WINDOW_WATCHDOG_RESET,
    RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
    RESET_CAUSE_SOFTWARE_RESET,
    RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
    RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
    RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_t;

/// @brief      Obtain the STM32 system reset cause
/// @param      None
/// @return     The system reset cause
reset_cause_t reset_cause_get(void)
{
    reset_cause_t reset_cause;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
            {
        reset_cause = RESET_CAUSE_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
            {
        reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
            {
        reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
            {
        // This reset is induced by calling the ARM CMSIS
        // `NVIC_SystemReset()` function!
        reset_cause = RESET_CAUSE_SOFTWARE_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
            {
        reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
            {
        reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
    }
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
    // ensure first that the reset cause is NOT a POR/PDR reset. See note
    // below.
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
            {
        reset_cause = RESET_CAUSE_BROWNOUT_RESET;
    }
    else
    {
        reset_cause = RESET_CAUSE_UNKNOWN;
    }

    // Clear all the reset flags or else they will remain set during future
    // resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause;
}

// Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock
// Controller (RCC) header files, such as
// "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h",
// etc., indicate that the brownout flag, `RCC_FLAG_BORRST`, will be set in
// the event of a "POR/PDR or BOR reset". This means that a Power-On Reset
// (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag.
// See the doxygen just above their definition for the
// `__HAL_RCC_GET_FLAG()` macro to see this:
//      "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout
//      Reset flag will *also* be set in the event of a POR/PDR.
// Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after*
// first checking the `RCC_FLAG_PORRST` flag in order to ensure first that the
// reset cause is NOT a POR/PDR reset.

/// @brief      Obtain the system reset cause as an ASCII-printable name string
///             from a reset cause type
/// @param[in]  reset_cause     The previously-obtained system reset cause
/// @return     A null-terminated ASCII name string describing the system
///             reset cause
const char* reset_cause_get_name(reset_cause_t reset_cause)
{
    const char *reset_cause_name = "TBD";

    switch (reset_cause)
    {
    case RESET_CAUSE_UNKNOWN:
        reset_cause_name = "UNKNOWN";
        break;
    case RESET_CAUSE_LOW_POWER_RESET:
        reset_cause_name = "LOW_POWER_RESET";
        break;
    case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
        reset_cause_name = "WINDOW_WATCHDOG_RESET";
        break;
    case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
        reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
        break;
    case RESET_CAUSE_SOFTWARE_RESET:
        reset_cause_name = "SOFTWARE_RESET";
        break;
    case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
        reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
        break;
    case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
        reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
        break;
    case RESET_CAUSE_BROWNOUT_RESET:
        reset_cause_name = "BROWNOUT_RESET (BOR)";
        break;
    }

    return reset_cause_name;
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
    MX_TIM13_Init();
    MX_USART1_UART_Init();
    MX_DAC_Init();
    MX_TIM6_Init();
    MX_IWDG_Init();
    /* USER CODE BEGIN 2 */

    //buf[1] = 0xff;
    printf("\n\n\n--------\nStarting\n");

    reset_cause_t reset_cause = reset_cause_get();
    printf("The system reset cause is \"%s\"\n", reset_cause_get_name(reset_cause));

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

    /* creation of sineTask */
    sineTaskHandle = osThreadNew(StartSineTask, NULL, &sineTask_attributes);

    /* creation of wdgTask */
    wdgTaskHandle = osThreadNew(StartIwdgTask, NULL, &wdgTask_attributes);

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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

    /* USER CODE BEGIN IWDG_Init 0 */

    /* USER CODE END IWDG_Init 0 */

    /* USER CODE BEGIN IWDG_Init 1 */

    /* USER CODE END IWDG_Init 1 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
    hiwdg.Init.Reload = 3000;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN IWDG_Init 2 */

    /* USER CODE END IWDG_Init 2 */

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
            printf("No       Name          P  S   Usage       Count      HW\n");

            for (x = 0; x < uxArraySize; x++) {

                runtime_percentage = (float) (100
                        * (float) pxTaskStatusArray[x].ulRunTimeCounter
                        / (float) ulTotalRunTime);

                printf("Task %2lu: %-12s %2lu %2d %8.4f (%12lu) %5i\n",
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
            //++sine_task;

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

/* USER CODE BEGIN Header_StartIwdgTask */
/**
 * @brief Function implementing the wdgTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIwdgTask */
void StartIwdgTask(void *argument)
{
    /* USER CODE BEGIN StartIwdgTask */
    /* Infinite loop */
    for (;;)
            {
        HAL_IWDG_Refresh(&hiwdg); // Kick the dog!
        osDelay(1000);

    }
    /* USER CODE END StartIwdgTask */
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
