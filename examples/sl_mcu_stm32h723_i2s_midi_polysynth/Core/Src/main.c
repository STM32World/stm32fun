/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : USB MIDI Polyphonic Synthesizer (Deferred Processing)
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
#include <math.h>
#include <arm_math.h>
#include "tusb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum wave_t {
    SINE_WAVE = 0,
    SAW_RIGHT_WAVE = 1,
    SAW_LEFT_WAVE = 2,
    TRIANGLE_WAVE = 3,
    SQUARE_WAVE = 4
};

typedef enum {
    ENVELOPE_IDLE = 0,
    ENVELOPE_ATTACK,
    ENVELOPE_DECAY,
    ENVELOPE_SUSTAIN,
    ENVELOPE_RELEASE
} env_stage_t;

typedef struct {
    float attack_rate;   // Gain increase per sample
    float decay_rate;    // Gain decrease per sample
    float sustain_level; // Target level [0.0f - 1.0f]
    float release_rate;  // Gain decrease per sample
} adsr_config_t;

typedef struct {
    uint8_t active;
    uint8_t note;
    float velocity_gain;
    enum wave_t wave_type;
    float angle;
    float angle_change;

    // ADSR State Data
    env_stage_t env_stage;
    float env_level;
} synth_voice_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TAU 6.28318530717958647692f

// 32 samples per half-buffer
#define I2S_DMA_BUFFER_SAMPLES 64

// Stereo (2 channels) * 2 half-buffers (Ping-Pong) * Samples
#define I2S_DMA_BUFFER_SIZE (2 * 2 * I2S_DMA_BUFFER_SAMPLES)

#define SAMPLE_FREQ 48000
#define MAX_VOICES 10

#define PCM16_MAX  32767.0f
#define PCM16_MIN -32768.0f

#define TOTAL_CALCULATIONS 500000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */

static const float MIDI_NOTE_TO_FREQ[128] = {
        8.1758f, 8.6620f, 9.1770f, 9.7227f, 10.3009f, 10.9134f, 11.5623f, 12.2499f,
        12.9783f, 13.7500f, 14.5676f, 15.4339f, 16.3516f, 17.3239f, 18.3540f, 19.4454f,
        20.6017f, 21.8268f, 23.1247f, 24.4997f, 25.9565f, 27.5000f, 29.1352f, 30.8677f,
        32.7032f, 34.6478f, 36.7081f, 38.8909f, 41.2034f, 43.6535f, 46.2493f, 48.9994f,
        51.9131f, 55.0000f, 58.2705f, 61.7354f, 65.4064f, 69.2957f, 73.4162f, 77.7817f,
        82.4069f, 87.3071f, 92.4986f, 97.9989f, 103.8262f, 110.0000f, 116.5409f, 123.4708f,
        130.8128f, 138.5913f, 146.8324f, 155.5635f, 164.8138f, 174.6141f, 185.0000f, 195.9977f,
        207.6523f, 220.0000f, 233.0819f, 246.9417f, 261.6256f, 277.1826f, 293.6648f, 311.1270f,
        329.6276f, 349.2282f, 369.9944f, 391.9954f, 415.3047f, 440.0000f, 466.1638f, 493.8833f,
        523.2511f, 554.3653f, 587.3295f, 622.2540f, 659.2551f, 698.4565f, 739.9888f, 783.9909f,
        830.6094f, 880.0000f, 932.3275f, 987.7666f, 1046.5023f, 1108.7305f, 1174.6591f, 1244.5079f,
        1318.5102f, 1396.9129f, 1479.9777f, 1567.9817f, 1661.2188f, 1760.0000f, 1864.6550f, 1975.5332f,
        2093.0045f, 2217.4610f, 2349.3181f, 2489.0159f, 2637.0205f, 2793.8259f, 2959.9554f, 3135.9635f,
        3322.4376f, 3520.0000f, 3729.3101f, 3951.0664f, 4186.0090f, 4434.9221f, 4698.6363f, 4978.0317f,
        5274.0410f, 5587.6518f, 5919.9108f, 6271.9270f, 6644.8752f, 7040.0000f, 7458.6202f, 7902.1328f,
        8372.0181f, 8869.8443f, 9397.2726f, 9956.0635f, 10548.0820f, 11175.3037f, 11839.8216f, 12543.8540f
};

int16_t i2s_dma_buffer[I2S_DMA_BUFFER_SIZE];
int16_t *dma_buffer_to_fill = NULL; // Deferred buffer processing pointer

synth_voice_t voices[MAX_VOICES] = { 0 };
enum wave_t global_wave_type = TRIANGLE_WAVE; // Sounds best when recording

// Master Volume Scaler [0.0f to 1.0f]
static float master_volume = 0.5f;

// ADSR profile: Attack 15ms, Decay 80ms, Sustain 70%, Release 120ms
adsr_config_t global_adsr = {
        .attack_rate = 1.0f / (0.015f * SAMPLE_FREQ),
        .decay_rate = (1.0f - 0.7f) / (0.080f * SAMPLE_FREQ),
        .sustain_level = 0.7f,
        .release_rate = 0.7f / (0.120f * SAMPLE_FREQ)
};

// Single-Pole IIR Low-Pass Filter state
static float lpf_state = 0.0f;
static float lpf_alpha = 0.15f;

void synth_set_cutoff(float cutoff_hz) {
    if (cutoff_hz > (SAMPLE_FREQ / 2.0f))
        cutoff_hz = SAMPLE_FREQ / 2.0f;
    float dt = 1.0f / (float) SAMPLE_FREQ;
    float rc = 1.0f / (2.0f * (float) M_PI * cutoff_hz);
    lpf_alpha = dt / (rc + dt);
}

void synth_set_master_volume(float vol) {
    if (vol > 1.0f)
        vol = 1.0f;
    if (vol < 0.0f)
        vol = 0.0f;
    master_volume = vol;
}

float synth_get_master_volume(void) {
    return master_volume;
}

uint8_t change_wave = 0;

uint32_t cb_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S2_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
    if (ch == '\n') {
        HAL_UART_Transmit(&huart1, (uint8_t*) "\r", 1, HAL_MAX_DELAY);
    }
    if (HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }
    return ch;
}

void process_buffer(int16_t *out_buffer) {
    float mix_buffer[I2S_DMA_BUFFER_SAMPLES] = { 0.0f };
    const float inv_two_pi = 1.0f / TAU;

    for (int v = 0; v < MAX_VOICES; v++) {
        synth_voice_t *voice = &voices[v];

        if (!voice->active)
            continue;

        for (int i = 0; i < I2S_DMA_BUFFER_SAMPLES; i++) {
            switch (voice->env_stage) {
            case ENVELOPE_ATTACK:
                voice->env_level += global_adsr.attack_rate;
                if (voice->env_level >= 1.0f) {
                    voice->env_level = 1.0f;
                    voice->env_stage = ENVELOPE_DECAY;
                }
                break;

            case ENVELOPE_DECAY:
                voice->env_level -= global_adsr.decay_rate;
                if (voice->env_level <= global_adsr.sustain_level) {
                    voice->env_level = global_adsr.sustain_level;
                    voice->env_stage = ENVELOPE_SUSTAIN;
                }
                break;

            case ENVELOPE_SUSTAIN:
                voice->env_level = global_adsr.sustain_level;
                break;

            case ENVELOPE_RELEASE:
                voice->env_level -= global_adsr.release_rate;
                if (voice->env_level <= 0.0001f) {
                    voice->env_level = 0.0f;
                    voice->env_stage = ENVELOPE_IDLE;
                    voice->active = 0;
                }
                break;

            default:
                break;
            }

            if (voice->active) {
                float sample = 0.0f;
                switch (voice->wave_type) {
                case SINE_WAVE:
                    sample = arm_cos_f32(voice->angle);
                    break;

                case SAW_RIGHT_WAVE: {
                    float phase = voice->angle * inv_two_pi;
                    sample = (2.0f * phase) - 1.0f;
                    break;
                }

                case SAW_LEFT_WAVE: {
                    float phase = voice->angle * inv_two_pi;
                    sample = 1.0f - (2.0f * phase);
                    break;
                }

                case TRIANGLE_WAVE: {
                    float phase = voice->angle * inv_two_pi;
                    sample = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
                    break;
                }

                case SQUARE_WAVE: {
                    float phase = voice->angle * inv_two_pi;
                    sample = (phase < 0.5f) ? 1.0f : -1.0f;
                    break;
                }
                }

                mix_buffer[i] += sample * voice->velocity_gain * voice->env_level;

                voice->angle += voice->angle_change;
                if (voice->angle >= TAU) {
                    voice->angle -= TAU;
                }
            }
        }
    }

    // Headroom calculation scaled by global master_volume [0.0f to 1.0f]
    const float master_gain = (32767.0f / (float) MAX_VOICES) * master_volume;

    for (int i = 0; i < I2S_DMA_BUFFER_SAMPLES; i++) {
        int idx = i * 2;

        float raw_val = mix_buffer[i] * master_gain;

        // Apply Low-Pass Filter
        lpf_state += lpf_alpha * (raw_val - lpf_state);

        float val = lpf_state;
        if (val > PCM16_MAX)
            val = PCM16_MAX;
        if (val < PCM16_MIN)
            val = PCM16_MIN;

        int16_t sample_16 = (int16_t) val;

        out_buffer[idx] = sample_16;
        out_buffer[idx + 1] = sample_16;
    }
}

/* Deferred ISR Callbacks - Correctly dynamic for buffer size */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI2) {
        dma_buffer_to_fill = &i2s_dma_buffer[0];
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI2) {
        dma_buffer_to_fill = &i2s_dma_buffer[I2S_DMA_BUFFER_SIZE / 2];
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BTN_Pin) {
        change_wave = 1;
    }
}

void synth_note_off(uint8_t note) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].active && voices[i].note == note && voices[i].env_stage != ENVELOPE_RELEASE) {
            voices[i].env_stage = ENVELOPE_RELEASE;
        }
    }
}

void synth_all_notes_off(void) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].active) {
            voices[i].env_stage = ENVELOPE_RELEASE;
        }
    }
}

void synth_note_on(uint8_t note, uint8_t velocity) {
    if (note > 127 || velocity == 0) {
        synth_note_off(note);
        return;
    }

    int slot = -1;

    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].active && voices[i].note == note) {
            slot = i;
            break;
        }
    }

    if (slot == -1) {
        for (int i = 0; i < MAX_VOICES; i++) {
            if (!voices[i].active) {
                slot = i;
                break;
            }
        }
    }

    if (slot == -1) {
        slot = 0;
    }

    voices[slot].note = note;
    voices[slot].velocity_gain = (float) velocity / 127.0f;
    voices[slot].angle_change = MIDI_NOTE_TO_FREQ[note] * (TAU / SAMPLE_FREQ);
    voices[slot].wave_type = global_wave_type;

    // Reset phase & envelope level to prevent zero-crossing clicks when starting a note
    voices[slot].angle = 0.0f;
    voices[slot].env_level = 0.0f;

    voices[slot].env_stage = ENVELOPE_ATTACK;
    voices[slot].active = 1;
}

void tud_midi_rx_cb(uint8_t itf)
{
    (void) itf;
    uint8_t packet[4];

    while (tud_midi_packet_read(packet))
    {
        uint8_t status = packet[1];
        uint8_t data1 = packet[2]; // MIDI Note / Controller
        uint8_t data2 = packet[3]; // Velocity / Control Value

        uint8_t msg_type = status & 0xF0;

        switch (msg_type)
        {
        case 0x90: // Note On
            printf("On : %d\n", data1);
            if (data2 > 0) {
                synth_note_on(data1, data2);
            } else {
                synth_note_off(data1);
            }
            break;

        case 0x80: // Note Off
            printf("Off: %d\n", data1);
            synth_note_off(data1);
            break;

        case 0xB0: // Control Change
            if (data1 == 7) { // MIDI CC #7: Master Volume
                synth_set_master_volume((float) data2 / 127.0f);
            } else if (data1 == 123 || data1 == 120) {
                synth_all_notes_off();
            }
            break;

        default:
            break;
        }
    }
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

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

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
    MX_CRC_Init();
    MX_I2S2_Init();
    MX_USB_OTG_HS_PCD_Init();
    /* USER CODE BEGIN 2 */

    printf("\n\n\nStarting USB MIDI Synth (Deferred Execution Model)\n");

    // Enable USB IRQ for TinyUSB event handling
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);

    // Start I2S DMA transmission using full buffer size
    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) i2s_dma_buffer, I2S_DMA_BUFFER_SIZE);

    // Initialize TinyUSB Device stack
    tusb_init();

    // Enable Soft Connect on OTG_HS peripheral (pulls D+ high via internal resistor)
    HAL_PCD_DevConnect(&hpcd_USB_OTG_HS);

    synth_set_cutoff(2400.0f);
    synth_set_master_volume(0.5f);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t now = 0;
    uint32_t loop_cnt = 0;
    uint32_t next_blink = 500;
    uint32_t next_tick = 1000;

    while (1) {

        // Process audio buffer IMMEDIATELY when requested by ISR
        if (dma_buffer_to_fill != NULL) {
            ++cb_count;
            int16_t *buf = dma_buffer_to_fill;
            dma_buffer_to_fill = NULL; // Clear flag before processing to prevent double-fills
            process_buffer(buf);
        }

        // Handle TinyUSB events
        tud_task();

        // Low priority tasks
        now = uwTick;

        if (change_wave) {
            global_wave_type = (enum wave_t) ((global_wave_type + 1) % 5);
            printf("Waveform Changed -> %d\n", global_wave_type);
            change_wave = 0;
        }

        if (now >= next_blink) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            next_blink = now + 500;
        }

        if (now >= next_tick) {
            printf("Tick %lu (loop=%lu cb=%lu)\n", now / 1000, loop_cnt, cb_count);
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 34;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 3072;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
            {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
            | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
            {
        Error_Handler();
    }
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void)
{

    /* USER CODE BEGIN I2S2_Init 0 */

    /* USER CODE END I2S2_Init 0 */

    /* USER CODE BEGIN I2S2_Init 1 */

    /* USER CODE END I2S2_Init 1 */
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_FULLDUPLEX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
    hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
    hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
    hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN I2S2_Init 2 */

    /* USER CODE END I2S2_Init 2 */

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
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
            {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
            {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
            {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
 * @brief USB_OTG_HS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_HS_PCD_Init(void)
{

    /* USER CODE BEGIN USB_OTG_HS_PCD_Init 0 */

    /* USER CODE END USB_OTG_HS_PCD_Init 0 */

    /* USER CODE BEGIN USB_OTG_HS_PCD_Init 1 */

    /* USER CODE END USB_OTG_HS_PCD_Init 1 */
    hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
    hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
    hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
    hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
    hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN USB_OTG_HS_PCD_Init 2 */

    /* USER CODE END USB_OTG_HS_PCD_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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

    /*Configure GPIO pin : BTN_Pin */
    GPIO_InitStruct.Pin = BTN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(BTN_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BTN_EXTI_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
