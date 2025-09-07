/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lbthomsen@gmail.com>
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

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
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for statusTask */
osThreadId_t statusTaskHandle;
const osThreadAttr_t statusTask_attributes = {
        .name = "statusTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for benchTask */
osThreadId_t benchTaskHandle;
const osThreadAttr_t benchTask_attributes = {
        .name = "benchTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityLow,
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
/* USER CODE BEGIN PV */

const char total_uptime_filename[] = "uptime.dat";

uint32_t total_uptime = 0;

// Counter updated from high freq timer
volatile unsigned long ulHighFrequencyTimerTicks;

// Pre-allocate all the heap for FreeRTOS.  FreeRTOS will do this by itself if this define is
// not set.
#ifdef configAPPLICATION_ALLOCATED_HEAP
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".ccmram"))); // Put in ccmram
#endif

int32_t a[10000] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM13_Init(void);
void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartTickTask(void *argument);
void StartStatusTask(void *argument);
void StartBenchTask(void *argument);

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
    MX_DMA_Init();
    MX_SDIO_SD_Init();
    MX_USART1_UART_Init();
    MX_FATFS_Init();
    MX_TIM13_Init();
    /* USER CODE BEGIN 2 */

    printf("\n\n\n\n--------\nStarting\n");

    uint32_t wbytes, rbytes; /* File write counts */

    if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
        printf("Unable to mount disk\n");
        Error_Handler();
    }

    if (f_open(&SDFile, total_uptime_filename, FA_OPEN_EXISTING | FA_READ) == FR_OK) {
        if (f_read(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &rbytes) == FR_OK) {
            printf("Total uptime = %lu\n", total_uptime);
            f_close(&SDFile);
        } else {
            printf("Unable to read\n");
            Error_Handler();
        }
    } else {
        // File did not exist - let's create it
        if (f_open(&SDFile, total_uptime_filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
            if (f_write(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &wbytes) == FR_OK) {
                printf("File %s created\n", total_uptime_filename);
                f_close(&SDFile);
            } else {
                printf("Unable to write\n");
                Error_Handler();
            }
        } else {
            printf("Unable to create\n");
            Error_Handler();
        }
    }

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

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

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

    /* creation of statusTask */
    statusTaskHandle = osThreadNew(StartStatusTask, NULL, &statusTask_attributes);

    /* creation of benchTask */
    benchTaskHandle = osThreadNew(StartBenchTask, NULL, &benchTask_attributes);

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

    while (1) {

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
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{

    /* USER CODE BEGIN SDIO_Init 0 */

    /* USER CODE END SDIO_Init 0 */

    /* USER CODE BEGIN SDIO_Init 1 */

    /* USER CODE END SDIO_Init 1 */
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 0;
    /* USER CODE BEGIN SDIO_Init 2 */
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    /* USER CODE END SDIO_Init 2 */

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
    htim13.Init.Prescaler = 84 - 1;
    htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim13.Init.Period = 10 - 1;
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
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : SD_DET_Pin */
    GPIO_InitStruct.Pin = SD_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

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
    /* Infinite loop */
    for (;;)
            {
        osDelay(1);
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
    /* Infinite loop */
    for (;;) {
        osDelay(500);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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

    /* Infinite loop */
    for (;;) {

        osDelay(1000);

        ++total_uptime;

        osMutexWait(printMutexHandle, osWaitForever);
        printf("Tick %lu (total = %lu)\n", osKernelGetTickCount() / 1000, total_uptime);
        osMutexRelease(printMutexHandle);

        uint32_t wbytes;
        if (f_open(&SDFile, total_uptime_filename, FA_OPEN_EXISTING | FA_WRITE) == FR_OK) {
            if (f_write(&SDFile, &total_uptime, sizeof(total_uptime), (void*) &wbytes) != FR_OK) {
                printf("Unable to write\n");
            }
            f_close(&SDFile);
        } else {
            printf("Unable to open file\n");
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

    /* USER CODE END StartStatusTask */
}

/* USER CODE BEGIN Header_StartBenchTask */
/**
 * @brief Function implementing the benchTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBenchTask */
void StartBenchTask(void *argument)
{
    /* USER CODE BEGIN StartBenchTask */
    /* Infinite loop */
    for (;;) {

        int u = 5000;
        int r = rand() % 10000;              // Get a random integer 0 <= r < 10k

        for (int i = 0; i < 10000; i++) {    // 10k outer loop iterations
            for (int j = 0; j < 100000; j++) { // 100k inner loop iterations, per outer loop iteration
                a[i] = a[i] + j % u;               // Simple sum
            }
            a[i] += r;                         // Add a random value to each element in array
        }

        osMutexWait(printMutexHandle, osWaitForever);
        printf("Bench: %lu\n", a[r]);                // Print out a single element from the array
        osMutexRelease(printMutexHandle);

        osDelay(10000);
    }
    /* USER CODE END StartBenchTask */
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
