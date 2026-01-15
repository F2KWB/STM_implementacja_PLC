/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FABRYKA PRO v8.0 (AUTO-RESET SERVICE)
  * @description    : Dodano automatyczne resetowanie licznika serwisu po 3s.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include "plc_blocks.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;
UART_HandleTypeDef huart2;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// ============================================================================
// 1. KONFIGURACJA
// ============================================================================
#define PLC_SCAN_TIME_MS    50

#define PARKING_CAPACITY    5
#define MIXER_CYCLES_LIMIT  2
#define MIXER_TIME_TOTAL    10000
#define MIXER_PHASE_1       3000
#define MIXER_PHASE_2       8000
#define TANK_MIN            10
#define TANK_MAX            90
#define PUMP_START_LVL      20
#define PUMP_STOP_LVL       80
#define PUMP_DELAY_MS       2000
#define FAN_COOLDOWN_MS     3000
#define AUTO_RESET_TIME_MS  3000    // Czas trwania "Automatycznego czyszczenia"

// ============================================================================
// 2. OBIEKTY LOGIKI PLC
// ============================================================================
TON_Block    T_Mixer;
TON_Block    T_PumpDelay;
CTU_Block    C_Parking;

R_TRIG_Block Trig_Exit;
TOF_Block    T_Fan;
CTD_Block    C_Service;

// NOWOŚĆ: Timer do automatycznego resetu
TON_Block    T_AutoReset;

// ============================================================================
// 3. DANE
// ============================================================================
typedef struct {
    bool Btn_Entry;
    bool Btn_Exit;
    bool Btn_Start;
    bool Car_Sensor;
    uint32_t Dist_CM;
    uint8_t Tank_Level;
} Inputs_T;

typedef struct {
    bool Pump;
    bool RGB_R, RGB_G, RGB_B;
    bool Led_Full;
    bool Led_Free;
    bool Fan_Cooling;
    bool Service_Req;
    bool Auto_Cleaning; // Lampa sygnalizująca proces resetu
} Outputs_T;

typedef struct {
    bool Mixer_Running;
    bool Alarm_Low;
    bool Alarm_High;
    bool Critical_Error;
} System_T;

Inputs_T  In;
Outputs_T Out;
System_T  Sys;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);

void ReadInputs(void);
void Logic_Factory(void);
void WriteOutputs(void);
void Safety_Check(void);
void PrintDashboard(void);
uint32_t HCSR04_Read(void);
void DrawBar(uint8_t val);

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10);
    return ch;
}
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) __io_putchar(*ptr++);
    return len;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  HAL_PWR_EnableBkUpAccess();

  // === INICJALIZACJA ===

  // Parking
  uint32_t saved_parking = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
  if(saved_parking > PARKING_CAPACITY) saved_parking = 0;
  C_Parking.CV = (uint16_t)saved_parking;
  C_Parking.PV = PARKING_CAPACITY;
  C_Parking.R  = false;

  // Serwis
  uint32_t saved_service = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  if (saved_service == 0 || saved_service > MIXER_CYCLES_LIMIT) {
      C_Service.CV = MIXER_CYCLES_LIMIT;
  } else {
      C_Service.CV = (uint16_t)saved_service;
  }
  C_Service.PV = MIXER_CYCLES_LIMIT;
  C_Service.LD = false;

  // Timery
  T_Mixer.PT = MIXER_TIME_TOTAL;
  T_PumpDelay.PT = PUMP_DELAY_MS;
  T_Fan.PT = FAN_COOLDOWN_MS;
  T_AutoReset.PT = AUTO_RESET_TIME_MS; // 3 sekundy na auto-reset

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  printf("\033[2J\033[H");
  printf("SYSTEM START: FABRYKA PRO v8.0 (AUTO-RESET)\r\n");

  while (1)
  {
      uint32_t loop_start = HAL_GetTick();

      ReadInputs();
      Safety_Check();
      Logic_Factory();
      WriteOutputs();

      if (hiwdg.Instance != NULL) HAL_IWDG_Refresh(&hiwdg);

      static uint32_t last_print = 0;
      if (loop_start - last_print > 250) {
          last_print = loop_start;
          PrintDashboard();
      }

      while((HAL_GetTick() - loop_start) < PLC_SCAN_TIME_MS) { __NOP(); }
  }
}

// ============================================================================
// LOGIKA FABRYKI
// ============================================================================

void Logic_Factory(void) {

    // --- 1. PARKING ---
    Trig_Exit.CLK = In.Btn_Exit;
    R_TRIG_Update(&Trig_Exit);

    if (Trig_Exit.Q) {
        if (C_Parking.CV > 0) C_Parking.CV--;
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, C_Parking.CV);
    }

    C_Parking.CU = In.Btn_Entry;
    CTU_Update(&C_Parking);

    if (C_Parking.CV > C_Parking.PV) C_Parking.CV = C_Parking.PV;

    static uint16_t last_park_cv = 0;
    if (C_Parking.CV != last_park_cv) {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, C_Parking.CV);
        last_park_cv = C_Parking.CV;
    }

    if (C_Parking.Q) { Out.Led_Full = true; Out.Led_Free = false; }
    else             { Out.Led_Full = false; Out.Led_Free = true; }


    // --- 2. ZBIORNIK ---
    Sys.Alarm_Low  = (In.Tank_Level < TANK_MIN);
    Sys.Alarm_High = (In.Tank_Level > TANK_MAX);

    T_PumpDelay.IN = (In.Tank_Level < PUMP_START_LVL);
    TON_Update(&T_PumpDelay);

    if (T_PumpDelay.Q) Out.Pump = true;
    if (In.Tank_Level > PUMP_STOP_LVL || Sys.Alarm_High) Out.Pump = false;

    T_Fan.IN = Out.Pump;
    TOF_Update(&T_Fan);
    Out.Fan_Cooling = T_Fan.Q;


    // --- 3. MIESZALNIK ---

    // =========================================================
    // >>> SEKCJA AUTOMATYCZNEGO RESETU (AUTO-CLEANING) <<<
    // =========================================================

    // Jeśli licznik serwisu (C_Service) zszedł do 0 (Q=true),
    // Uruchom timer "T_AutoReset".
    if (C_Service.Q) {
        T_AutoReset.IN = true;
        Out.Auto_Cleaning = true; // Flaga dla wyświetlacza
    } else {
        T_AutoReset.IN = false;
        Out.Auto_Cleaning = false;
    }

    // Aktualizuj timer resetu
    TON_Update(&T_AutoReset);

    // Jeśli timer doliczył do końca (minęły 3 sekundy czyszczenia)...
    if (T_AutoReset.Q) {
        C_Service.LD = true; // ...załaduj automatycznie 5 do licznika!
    } else {
        C_Service.LD = false;
    }

    // =========================================================

    // Blokada startu podczas czyszczenia
    bool service_ok = !C_Service.Q;

    if (In.Btn_Start && service_ok) Sys.Mixer_Running = true;

    T_Mixer.IN = Sys.Mixer_Running;
    TON_Update(&T_Mixer);

    bool cycle_finished_pulse = false;

    Out.RGB_R = 0; Out.RGB_G = 0; Out.RGB_B = 0;
    if (Sys.Mixer_Running) {
        uint32_t t = T_Mixer.ET;
        if (t < MIXER_PHASE_1)      Out.RGB_B = true;
        else if (t < MIXER_PHASE_2) Out.RGB_G = true;
        else                        Out.RGB_R = true;

        if (T_Mixer.Q) {
            cycle_finished_pulse = true;
            Sys.Mixer_Running = false;
            T_Mixer.IN = false;
            TON_Update(&T_Mixer);
        }
    }

    C_Service.CD = cycle_finished_pulse;
    CTD_Update(&C_Service);

    static uint16_t last_service_cv = 0;
    if (C_Service.CV != last_service_cv) {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, C_Service.CV);
        last_service_cv = C_Service.CV;
    }

    Out.Service_Req = C_Service.Q;
}

// ============================================================================
// OBSŁUGA PERYFERIÓW
// ============================================================================

void ReadInputs(void) {
    static uint32_t adc_sum = 0;
    static uint8_t adc_count = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_sum += HAL_ADC_GetValue(&hadc1);
        adc_count++;
    }
    if (adc_count >= 10) {
        In.Tank_Level = ((adc_sum / 10) * 100) / 4095;
        adc_sum = 0; adc_count = 0;
    }
    In.Dist_CM = HCSR04_Read();
    In.Car_Sensor = (In.Dist_CM > 0 && In.Dist_CM < 15);
    bool btn_phys_entry = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET);
    In.Btn_Entry = btn_phys_entry || In.Car_Sensor;
    In.Btn_Exit  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
    In.Btn_Start = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
}

void WriteOutputs(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, Out.Pump ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, Out.Led_Full ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, Out.Led_Free ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, Out.RGB_R ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, Out.RGB_G ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, Out.RGB_B ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Safety_Check(void) {
    if (In.Car_Sensor && In.Btn_Exit) Sys.Critical_Error = true;
    if (Sys.Critical_Error) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        printf("\033[2J\033[H!!! CRITICAL ERROR !!!\r\n");
        while(1);
    }
}

uint32_t HCSR04_Read(void) {
    uint32_t local_time = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); for(int i=0;i<500;i++);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   for(int i=0;i<2000;i++);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    uint32_t t = 50000;
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) { if(t-- == 0) return 0; }
    t = 50000;
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) { local_time++; if(t-- == 0) break; }
    return local_time / 50;
}

void DrawBar(uint8_t val) {
    printf("[");
    for(int i=0; i<20; i++) printf((i < val/5) ? "#" : ".");
    printf("]");
}

void PrintDashboard(void) {
    printf("\033[?25l\033[H");
    printf("PROJEKT_SP_ALGORYTM_PLC_NA_STM32\033[K\r\n");
    printf("Uzylismy_z_wlasnej_biblioteki: TON, TOF, CTU, CTD, R_TRIG\033[K\r\n\r\n");

    printf("[1] ZBIORNIK (TON + TOF)                    \033[K\r\n");
    printf("    Poziom: "); DrawBar(In.Tank_Level); printf(" %3d%%\033[K\r\n", In.Tank_Level);
    printf("    Pompa:  %s  (timer_on: %4lu ms)\033[K\r\n", Out.Pump ? "\033[32m[ON] \033[0m" : "[OFF]", T_PumpDelay.ET);
    printf("    Wentyl: %s  (timer_off:%4lu ms)\033[K\r\n",
           Out.Fan_Cooling ? "\033[36m[RUN]\033[0m" : "[---]", T_Fan.ET);

    printf("--------------------------------------------\033[K\r\n");

    printf("[2] PARKING (CTU + R_TRIG)                  \033[K\r\n");
    printf("    Licznik: [%d / %d] (obecna_ilosc /max_ilosc)\033[K\r\n", C_Parking.CV, C_Parking.PV);
    printf("    Status:  %s\033[K\r\n",
           Out.Led_Full ? "\033[31m[ PELNY ]\033[0m" : "\033[32m[ WOLNY ]\033[0m");

    printf("--------------------------------------------\033[K\r\n");

    printf("[3] MIESZALNIK (TON + CTD + AUTO-RESET)     \033[K\r\n");
    printf("    Status:  ");
    if(Out.RGB_B) printf("\033[34m[ NALEWANIE ]\033[0m");
    else if(Out.RGB_G) printf("\033[32m[ MIESZANIE ]\033[0m");
    else if(Out.RGB_R) printf("\033[31m[ WYLEWANIE ]\033[0m");
    else printf("[ STOP ]     ");
    printf(" (%4lu ms)\033[K\r\n", T_Mixer.ET);

    // Wizualizacja Serwisu
    if (Out.Auto_Cleaning) {
        // Miganie napisu podczas czyszczenia
        printf("    Serwis:  \033[33m[ AUTO-CLEANING... %lu ms ]\033[0m\033[K\r\n",
               T_AutoReset.PT - T_AutoReset.ET);
    } else {
        printf("    Serwis:  [%d cykle] %s\033[K\r\n", C_Service.CV,
               Out.Service_Req ? "\033[31m[WAIT...]\033[0m" : "\033[32m[OK]\033[0m");
    }

    printf("\033[J");
}

void SystemClock_Config(void) { RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI; RCC_OscInitStruct.HSIState = RCC_HSI_ON; RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; RCC_OscInitStruct.LSIState = RCC_LSI_ON; RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; RCC_OscInitStruct.PLL.PLLM = 1; RCC_OscInitStruct.PLL.PLLN = 10; RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7; RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; HAL_RCC_OscConfig(&RCC_OscInitStruct); RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4); }
static void MX_GPIO_Init(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET); GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_1; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_10; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_10; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); }
static void MX_USART2_UART_Init(void) { huart2.Instance = USART2; huart2.Init.BaudRate = 115200; huart2.Init.WordLength = UART_WORDLENGTH_8B; huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX; huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16; HAL_UART_Init(&huart2); }
static void MX_ADC1_Init(void) { ADC_ChannelConfTypeDef sConfig = {0}; hadc1.Instance = ADC1; hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; hadc1.Init.Resolution = ADC_RESOLUTION_12B; hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; hadc1.Init.LowPowerAutoWait = DISABLE; hadc1.Init.ContinuousConvMode = DISABLE; hadc1.Init.NbrOfConversion = 1; hadc1.Init.DiscontinuousConvMode = DISABLE; hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; hadc1.Init.DMAContinuousRequests = DISABLE; hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED; hadc1.Init.OversamplingMode = DISABLE; HAL_ADC_Init(&hadc1); sConfig.Channel = ADC_CHANNEL_5; sConfig.Rank = ADC_REGULAR_RANK_1; sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; sConfig.SingleDiff = ADC_SINGLE_ENDED; sConfig.OffsetNumber = ADC_OFFSET_NONE; sConfig.Offset = 0; HAL_ADC_ConfigChannel(&hadc1, &sConfig); }
static void MX_IWDG_Init(void) { hiwdg.Instance = IWDG; hiwdg.Init.Prescaler = IWDG_PRESCALER_32; hiwdg.Init.Window = 4095; hiwdg.Init.Reload = 300; HAL_IWDG_Init(&hiwdg); }
static void MX_RTC_Init(void) { hrtc.Instance = RTC; hrtc.Init.HourFormat = RTC_HOURFORMAT_24; hrtc.Init.AsynchPrediv = 127; hrtc.Init.SynchPrediv = 255; hrtc.Init.OutPut = RTC_OUTPUT_DISABLE; HAL_RTC_Init(&hrtc); }
static void MX_TIM2_Init(void) { TIM_ClockConfigTypeDef sClockSourceConfig = {0}; TIM_MasterConfigTypeDef sMasterConfig = {0}; htim2.Instance = TIM2; htim2.Init.Prescaler = 7999; htim2.Init.CounterMode = TIM_COUNTERMODE_UP; htim2.Init.Period = 999; htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; HAL_TIM_Base_Init(&htim2); sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig); sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig); }
void Error_Handler(void) { __disable_irq(); while (1) {} }
