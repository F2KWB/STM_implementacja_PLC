/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FABRYKA - WERSJA SZYBKA (BEZ DELAY NA STARCIE)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- ZMIENNE GLOBALNE ---

// SYSTEM 1: ZBIORNIK
uint32_t water_level = 0;
uint8_t pump_status = 0;

// SYSTEM 2: PARKING
int cars_count = 0;
const int parking_limit = 5;
uint8_t btn_in_last = 1;
uint8_t btn_out_last = 1;

// SYSTEM 3: MIESZALNIK
typedef enum { IDLE, FILLING, MIXING, EMPTYING } MixerState;
MixerState mixer_state = IDLE;
uint32_t mixer_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
void ExecuteAutomaticLogic(void);
void PrintDashboard(void);
void RGB_Control(uint8_t r, uint8_t g, uint8_t b);

int __io_putchar(int ch) {
    if (ch == '\n') __io_putchar('\r');
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10);
    return 1;
}
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);

  // SZYBKI START (Bez delay, żeby Watchdog nie zabił procesora)
  if (hiwdg.Instance != NULL) HAL_IWDG_Refresh(&hiwdg); // Nakarm psa od razu!

  printf("\033[2J\033[H");
  printf("SYSTEM START OK.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
      ExecuteAutomaticLogic();
      PrintDashboard();

      // Nakarm psa (To jest najważniejsza linijka przeciw resetom)
      if (hiwdg.Instance != NULL) HAL_IWDG_Refresh(&hiwdg);

      HAL_Delay(100);
  }
}

/* USER CODE BEGIN 4 */

void RGB_Control(uint8_t r, uint8_t g, uint8_t b) {
    // Odwrócona logika dla Wspólnej Anody (0 = świeci)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (r ? GPIO_PIN_RESET : GPIO_PIN_SET)); // R
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (g ? GPIO_PIN_RESET : GPIO_PIN_SET)); // G
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (b ? GPIO_PIN_RESET : GPIO_PIN_SET)); // B
}

void ExecuteAutomaticLogic(void) {

    // === 1. ZBIORNIK (POPRAWIONA LOGIKA: <20% WŁĄCZ, >80% WYŁĄCZ) ===
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    water_level = (raw * 100) / 4095;
    HAL_ADC_Start(&hadc1);

    if (water_level < 20) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // D7 ON (Pompa włączona)
        pump_status = 1;
    }
    else if (water_level > 80) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // D7 OFF (Pompa wyłączona)
        pump_status = 0;
    }

    // === 2. PARKING ===
    int btn_in = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // D2
    int btn_out = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3); // D3

    if (btn_in == 0 && btn_in_last == 1) {
        if (cars_count < parking_limit) cars_count++;
    }
    btn_in_last = btn_in;

    if (btn_out == 0 && btn_out_last == 1) {
        if (cars_count > 0) cars_count--;
    }
    btn_out_last = btn_out;

    if (cars_count >= parking_limit) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // Czerwona ON
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // Zielona OFF
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // Zielona ON
    }

    // === 3. MIESZALNIK (POPRAWIONE KOLORY) ===
    int btn_start = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10); // D6

    switch (mixer_state) {
        case IDLE:
            RGB_Control(0, 0, 0); // OFF
            if (btn_start == 0) {
                mixer_state = FILLING;
                mixer_timer = HAL_GetTick();
            }
            break;

        case FILLING:
            RGB_Control(0, 0, 1); // BLUE ON
            if (HAL_GetTick() - mixer_timer > 3000) {
                mixer_state = MIXING;
                mixer_timer = HAL_GetTick();
            }
            break;

        case MIXING:
            RGB_Control(0, 1, 0); // GREEN ON
            if (HAL_GetTick() - mixer_timer > 5000) {
                mixer_state = EMPTYING;
                mixer_timer = HAL_GetTick();
            }
            break;

        case EMPTYING:
            RGB_Control(1, 0, 0); // RED ON
            if (HAL_GetTick() - mixer_timer > 3000) {
                mixer_state = IDLE;
            }
            break;
    }
}

void PrintDashboard(void) {
    printf("\033[H");
    printf("=== STATUS FABRYKI ===\r\n\r\n");

    // ZBIORNIK
    printf(" [1] ZBIORNIK: %3lu %%  ", water_level);
    printf("[");
    for(int i=0; i<10; i++) {
        if((water_level/10) > i) printf("#"); else printf(".");
    }
    printf("] ");

    if(pump_status == 1) printf("\033[36m[ POMPA WLEWA WODE ]\033[0m");
    else                 printf("[ ................ ]");
    printf("\r\n\r\n");

    // PARKING
    printf(" [2] PARKING:  %d / %d  ", cars_count, parking_limit);
    if(cars_count >= parking_limit) printf("\033[31m[ PELNY ]\033[0m");
    else                            printf("\033[32m[ WOLNY ]\033[0m");
    printf("\r\n\r\n");

    // MIESZALNIK
    printf(" [3] MIESZALNIK: ");
    switch(mixer_state) {
        case IDLE:     printf("GOTOWY (Wcisnij D6)"); break;
        case FILLING:  printf("\033[34mNALEWANIE (Niebieski)\033[0m"); break;
        case MIXING:   printf("\033[32mMIESZANIE (Zielony)  \033[0m"); break;
        case EMPTYING: printf("\033[31mWYLEWANIE (Czerwony) \033[0m"); break;
    }
    printf("       \r\n");
}

// --- FUNKCJE INICJALIZACYJNE ---
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_IWDG_Init(void) {
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
/* USER CODE END 4 */
