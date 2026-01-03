/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 SOFT-PLC (FINALNA WERSJA - NAPRAWIONY ZBIORNIK)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdbool.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
// I2C usunięte
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

// === STRUKTURY PLC ===
typedef struct {
    bool     IN;
    uint32_t PT;
    uint32_t ET;
    bool     Q;
    uint32_t prev_tick;
    bool     mem;
} PLC_Timer;

typedef struct {
    bool    CU;
    bool    CD;
    bool    R;
    int16_t PV;
    int16_t CV;
    bool    QU;
    bool    QD;
    bool    last_CU;
    bool    last_CD;
} PLC_Counter;

// --- ZMIENNE SYSTEMOWE ---
uint32_t scan_start = 0;
uint32_t scan_time = 0;

// --- INSTANCJE BLOKÓW ---
PLC_Timer   T_PumpDelay;
PLC_Timer   T_MixerSequence;
PLC_Counter C_Parking;

// --- ZMIENNE FIZYCZNE ---
uint8_t I_WaterLevel = 0;
bool    I_Btn_Entry = false;
bool    I_Btn_Exit  = false;
bool    I_Btn_Start = false;

bool Q_Pump        = false;
bool Q_Light_Red   = false;
bool Q_Light_Green = false;
bool Q_RGB_R       = false;
bool Q_RGB_G       = false;
bool Q_RGB_B       = false;

bool M_MixerActive = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
void PLC_TON(PLC_Timer *t, uint32_t tick);
void PLC_CTUD(PLC_Counter *c);
void ReadInputs(void);
void LogicSolve(void);
void WriteOutputs(void);
void PrintDashboard(void);
void Load_Retentive_Data(void);

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
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  // 1. ODBLOKUJ PAMIĘĆ RETENCYJNĄ
  HAL_PWR_EnableBkUpAccess();

  // 2. WCZYTAJ DANE Z PAMIĘCI
  Load_Retentive_Data();

  // Konfiguracja
  T_PumpDelay.PT = 2000;
  C_Parking.PV = 5;

  // Karmienie psa na starcie
  if (hiwdg.Instance != NULL) HAL_IWDG_Refresh(&hiwdg);

  printf("\033[2J\033[H");
  printf("SYSTEM START OK. Pamiec wczytana: %d aut.\r\n", C_Parking.CV);
  HAL_Delay(500);
  /* USER CODE END 2 */

  while (1)
  {
      scan_start = HAL_GetTick();

      // 1. INPUT (Tutaj była poprawka!)
      ReadInputs();

      // === AWARIA (SABOTAŻ: D2 + D3) ===
      if (I_Btn_Entry && I_Btn_Exit) {
          printf("\033[2J\033[H");
          printf("!!! AWARIA KRYTYCZNA - WATCHDOG ZADZIALA ZA CHWILE !!!\r\n");
          while(1);
      }

      // 2. LOGIC
      LogicSolve();

      // 3. OUTPUT
      WriteOutputs();

      // 4. WATCHDOG
      if (hiwdg.Instance != NULL) HAL_IWDG_Refresh(&hiwdg);

      // 5. WIZUALIZACJA
      static uint32_t last_print = 0;
      if (HAL_GetTick() - last_print > 250) {
          last_print = HAL_GetTick();
          PrintDashboard();
      }

      scan_time = HAL_GetTick() - scan_start;
      HAL_Delay(10);
  }
}

/* USER CODE BEGIN 4 */

// === FUNKCJA PAMIĘCI ===
void Load_Retentive_Data(void) {
    uint32_t saved_val = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
    if (saved_val > 100) saved_val = 0;
    C_Parking.CV = (int16_t)saved_val;
}

// === FUNKCJE PLC ===
void PLC_TON(PLC_Timer *t, uint32_t tick) {
    if (t->IN) {
        if (!t->mem) { t->prev_tick = tick; t->mem = true; }
        uint32_t delta = tick - t->prev_tick;
        if (t->ET + delta >= t->PT) { t->ET = t->PT; t->Q = true; }
        else { t->ET += delta; t->Q = false; }
        t->prev_tick = tick;
    } else {
        t->ET = 0; t->Q = false; t->mem = false;
    }
}

void PLC_CTUD(PLC_Counter *c) {
    bool changed = false;

    if (c->R) {
        c->CV = 0;
        changed = true;
    } else {
        if (c->CU && !c->last_CU) if (c->CV < 99) { c->CV++; changed = true; }
        if (c->CD && !c->last_CD) if (c->CV > 0)  { c->CV--; changed = true; }
    }

    c->last_CU = c->CU;
    c->last_CD = c->CD;
    c->QU = (c->CV >= c->PV);
    c->QD = (c->CV <= 0);

    if (changed) {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, (uint32_t)c->CV);
    }
}

// === LOGIKA GLOWNA ===
void ReadInputs(void) {
    // --- POPRAWIONY ODCZYT ADC (START -> POLL -> GET) ---
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        I_WaterLevel = (raw * 100) / 4095;
    }
    // Nie robimy HAL_ADC_Stop, żeby było szybciej w następnym cyklu,
    // ale Start przed każdym Poll jest bezpieczniejszy.

    // Przyciski (Pull-Up)
    I_Btn_Entry = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET);
    I_Btn_Exit  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
    I_Btn_Start = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
}

void LogicSolve(void) {
    uint32_t current_time = HAL_GetTick();

    // Parking
    C_Parking.CU = I_Btn_Entry;
    C_Parking.CD = I_Btn_Exit;
    C_Parking.R  = 0;
    C_Parking.PV = 5;
    PLC_CTUD(&C_Parking);
    Q_Light_Red   = C_Parking.QU;
    Q_Light_Green = !C_Parking.QU;

    // Zbiornik
    T_PumpDelay.IN = (I_WaterLevel < 20);
    T_PumpDelay.PT = 2000;
    PLC_TON(&T_PumpDelay, current_time);

    // Logika Set/Reset dla pompy
    if (T_PumpDelay.Q) Q_Pump = true;       // Włącz po opóźnieniu
    if (I_WaterLevel > 80) Q_Pump = false;  // Wyłącz natychmiast jak pełny

    // Mieszalnik
    if (I_Btn_Start) M_MixerActive = true;
    T_MixerSequence.IN = M_MixerActive;
    T_MixerSequence.PT = 10000;
    PLC_TON(&T_MixerSequence, current_time);

    uint32_t t = T_MixerSequence.ET;
    Q_RGB_R = 0; Q_RGB_G = 0; Q_RGB_B = 0;

    if (M_MixerActive) {
        if (t < 3000) Q_RGB_B = true;
        else if (t < 8000) Q_RGB_G = true;
        else if (t < 10000) Q_RGB_R = true;
        else M_MixerActive = false;
    }
}

void WriteOutputs(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, Q_Pump ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, Q_Light_Red ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, Q_Light_Green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, Q_RGB_R ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, Q_RGB_G ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, Q_RGB_B ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void PrintDashboard(void) {
    printf("\033[?25l\033[H");
    printf("=== SYSTEM FABRYKI (SCADA) ===               \r\n");
    printf("Status: PRACA NORMALNA                       \r\n\r\n");

    // PARKING
    printf("[2] PARKING (Licznik)                       \r\n");
    printf("    Auta:       [%2d / %2d]                  \r\n", C_Parking.CV, C_Parking.PV);
    if(C_Parking.QU) printf("    Wjazd:      \033[31m[ ZABRONIONY ]\033[0m            \r\n");
    else             printf("    Wjazd:      \033[32m[ DOZWOLONY  ]\033[0m            \r\n");
    printf("-------------------------------------------   \r\n");

    // ZBIORNIK - Teraz powinno działać poprawnie!
    printf("[1] ZBIORNIK WODY                           \r\n");
    printf("    Poziom:     %d%%    (Opoznienie: %d)      \r\n", I_WaterLevel, T_PumpDelay.IN);
    if(Q_Pump) printf("    POMPA:      \033[32m[ PRACA ]\033[0m                  \r\n");
    else       printf("    POMPA:      [ STOP  ]                  \r\n");
    printf("-------------------------------------------   \r\n");

    // MIESZALNIK
    printf("[3] MIESZALNIK PROCESOWY                    \r\n");
    printf("    Czas:       [%4lu / 10000 ms]           \r\n", T_MixerSequence.ET);
    printf("    Stan:       ");
    if(Q_RGB_B)      printf("\033[34m[ NALEWANIE ]\033[0m   ");
    else if(Q_RGB_G) printf("\033[32m[ MIESZANIE ]\033[0m   ");
    else if(Q_RGB_R) printf("\033[31m[ WYLEWANIE ]\033[0m   ");
    else             printf("[ OCZEKIWANIE ]   ");
    printf("\r\n                                            \r");
}

// === INICJALIZACJA ===

static void MX_RTC_Init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) Error_Handler();
}

void SystemClock_Config(void)
{
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
static void MX_ADC1_Init(void)
{
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
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}
static void MX_TIM2_Init(void)
{
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
static void MX_USART2_UART_Init(void)
{
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
static void MX_GPIO_Init(void)
{
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

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
/* USER CODE END 4 */
