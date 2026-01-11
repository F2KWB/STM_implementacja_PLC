/* USER CODE BEGIN Header */
/* PLC*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdbool.h>

// Biblioteka zawierająca definicje TON (Timer) i CTU (Licznik)
#include "plc_blocks.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;       // Uchwyt do przetwornika analogowego (Poziom wody)
IWDG_HandleTypeDef hiwdg;      // Uchwyt do Watchdoga (Resetuje układ przy awarii)
UART_HandleTypeDef huart2;     // Uchwyt do UART (Wysyłanie danych do PC)
RTC_HandleTypeDef hrtc;        // Uchwyt do Zegara RTC + Rejestrów Backup (Pamięć)
TIM_HandleTypeDef htim2;       // Uchwyt do Timera 2 (Wymagany przez bibliotekę HAL)

/* USER CODE BEGIN PV */

// ============================================================================
// 1. KONFIGURACJA PARAMETRÓW
// ============================================================================

// --- PARKING ---
#define PARKING_CAPACITY    5       // Ile aut mieści się na parkingu

// --- MIESZALNIK (Czasy w milisekundach) ---
#define MIXER_TIME_TOTAL    10000   // Całkowity czas procesu (10s)
#define MIXER_PHASE_1       3000    // Czas trwania Fazy 1 (Niebieska)
#define MIXER_PHASE_2       8000    // Czas zakończenia Fazy 2 (Zielona)

// --- ZBIORNIK WODY (Progi w %) ---
#define TANK_MIN            10      // Alarm: Poziom zbyt niski
#define TANK_MAX            90      // Alarm: Poziom zbyt wysoki
#define PUMP_START_LVL      20      // Włącz pompę, gdy woda spadnie poniżej 20%
#define PUMP_STOP_LVL       80      // Wyłącz pompę, gdy woda osiągnie 80%
#define PUMP_DELAY_MS       2000    // Czas opóźnienia załączenia pompy (2s)

// ============================================================================
// 2. OBIEKTY LOGIKI PLC (Z BIBLIOTEKI PLC_BLOCKS)
// ============================================================================

TON_Block T_Mixer;      // Timer sterujący sekwencją mieszalnika
TON_Block T_PumpDelay;  // Timer opóźniający start pompy
CTU_Block C_Parking;    // Licznik zliczający auta na parkingu

// ============================================================================
// === 3. STRUKTURY DANYCH (ORGANIZACJA ZMIENNYCH) ============================
// ============================================================================

// Wejścia
typedef struct {
    bool Btn_Entry;     // Przycisk wjazdu
    bool Btn_Exit;      // Przycisk wyjazdu
    bool Btn_Start;     // Przycisk startu procesu
    bool Car_Sensor;    // Logika czujnika (True = Auto wykryte)
    uint32_t Dist_CM;   // Odległość w cm (z czujnika)
    uint8_t Tank_Level; // Poziom wody (0-100%)
} Inputs_T;

// Wyjścia
typedef struct {
    bool Pump;          // Pompa wody
    bool RGB_R;         // Dioda RGB - Czerwona
    bool RGB_G;         // Dioda RGB - Zielona
    bool RGB_B;         // Dioda RGB - Niebieska
    bool Led_Full;      // Dioda Czerwona (Parking pełny)
    bool Led_Free;      // Dioda Zielona (Parking wolny)
} Outputs_T;

// Flagi Systemowe (Pamięć wewnętrzna sterownika)
typedef struct {
    bool Mixer_Running; // Czy mieszalnik aktualnie pracuje?
    bool Alarm_Low;     // Flaga alarmu niskiego poziomu
    bool Alarm_High;    // Flaga alarmu wysokiego poziomu
    bool Critical_Error;// Flaga błędu krytycznego (Safety)
} System_T;

// Globalne instancje struktur (Dostępne w całym programie)
Inputs_T  In;
Outputs_T Out;
System_T  Sys;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// Funkcje generowane przez CubeMX
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);

// Funkcje naszej aplikacji
void ReadInputs(void);          // Odczyt czujników
void Logic_Factory(void);       // Główna logika (PLC)
void WriteOutputs(void);        // Sterowanie wyjściami
void Safety_Check(void);        // System bezpieczeństwa
void PrintDashboard(void);      // Wyświetlanie na ekranie
uint32_t HCSR04_Read(void);     // Obsługa czujnika odległości

// Przekierowanie printf na UART
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10);
    return ch;
}
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) __io_putchar(*ptr++);
    return len;
}

/**
  * @brief  Główna funkcja programu (Entry Point)
  */
int main(void)
{
  // 1. Inicjalizacja sprzętu (HAL)
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();

  // 2. Włączenie dostępu do pamięci podtrzymywanej bateryjnie (Backup Domain)
  HAL_PWR_EnableBkUpAccess();

  // === INICJALIZACJA LOGIKI PLC ===

  // A. Odtworzenie stanu licznika parkingu po restarcie zasilania
  // Czytamy rejestr nr 0 z domeny Backup
  uint32_t saved_cnt = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);

  // Zabezpieczenie: Jeśli odczytana wartość to śmieci (np. > 5), wyzeruj
  if(saved_cnt > PARKING_CAPACITY) saved_cnt = 0;

  // Wpisanie wartości do struktury licznika z biblioteki
  C_Parking.CV = (uint16_t)saved_cnt;
  C_Parking.PV = PARKING_CAPACITY; // Ustawienie limitu
  C_Parking.R  = false;            // Reset wyłączony

  // B. Konfiguracja Timerów
  T_Mixer.PT = MIXER_TIME_TOTAL;
  T_PumpDelay.PT = PUMP_DELAY_MS;

  // C. Kalibracja ADC (dla dokładniejszego pomiaru)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // 3. Komunikat startowy
  printf("\033[2J\033[H"); // Wyczyść terminal
  printf("SYSTEM START: FABRYKA PRO v4.0\r\n");

  // === 4. GŁÓWNA PĘTLA STEROWANIA (SCAN CYCLE) ===
  // Ta pętla wykonuje się w nieskończoność
  while (1)
  {
      uint32_t loop_start = HAL_GetTick(); // Zapisz czas startu cyklu

      // KROK 1: ODCZYT WEJŚĆ (Input Scan)
      ReadInputs();

      // KROK 2: KONTROLA BEZPIECZEŃSTWA (Safety)
      Safety_Check();

      // KROK 3: WYKONANIE LOGIKI (Logic Execution)
      Logic_Factory();

      // KROK 4: ZAPIS WYJŚĆ (Output Scan)
      WriteOutputs();

      // KROK 5: OBSŁUGA WATCHDOGA
      // Jeśli ta komenda nie wykona się przez 4 sekundy, procesor się zresetuje
      if (hiwdg.Instance != NULL) HAL_IWDG_Refresh(&hiwdg);

      // KROK 6: WIZUALIZACJA (Co 250ms, żeby ekran nie migał)
      static uint32_t last_print = 0;
      if (loop_start - last_print > 250) {
          last_print = loop_start;
          PrintDashboard();
      }

      // KROK 7: STABILIZACJA CZASU CYKLU (Deterministic Scan)
      // Czekaj aktywnie, aż minie dokładnie 50ms od początku pętli.
      // Dzięki temu sterownik działa ze stałą częstotliwością 20Hz.
      while((HAL_GetTick() - loop_start) < 50) {
          __NOP(); // Nic nie rób (No Operation)
      }
  }
}

// ============================================================================
// LOGIKA FABRYKI (TU UŻYWAMY BIBLIOTEKI PLC_BLOCKS)
// ============================================================================

void Logic_Factory(void) {

    // --- SEKCJA 1: PARKING (Licznik CTU) ---

    // [WYJAZD] Obsługa ręczna
    static bool prev_exit = false;
    if (In.Btn_Exit && !prev_exit) {
        if (C_Parking.CV > 0) C_Parking.CV--; // Zmniejsz licznik (nie zejdzie poniżej 0)
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, C_Parking.CV); // Zapisz od razu
    }
    prev_exit = In.Btn_Exit;

    // [WJAZD] Obsługa przez bibliotekę
    C_Parking.CU = In.Btn_Entry;

    // >>> AKTUALIZACJA BLOKU LICZNIKA <<<
    CTU_Update(&C_Parking);

    // --- POPRAWKA: TWARDY LIMIT LICZNIKA ---
    // Jeśli licznik przeskoczył powyżej limitu (PV), cofnij go do maksimum.
    // Dzięki temu nigdy nie pokaże 6, 7, 8...
    if (C_Parking.CV > C_Parking.PV) {
        C_Parking.CV = C_Parking.PV;
    }
    // ---------------------------------------

    // [ZAPIS PAMIĘCI]
    // Jeśli licznik się zmienił (wzrósł lub został ograniczony), zapisz do pamięci
    static uint16_t last_cv = 0;
    if (C_Parking.CV != last_cv) {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, C_Parking.CV);
        last_cv = C_Parking.CV;
    }

    // [SYGNALIZACJA]
    if (C_Parking.Q) {
        Out.Led_Full = true;  // Parking Pełny
        Out.Led_Free = false;
    } else {
        Out.Led_Full = false; // Parking Wolny
        Out.Led_Free = true;
    }


    // --- SEKCJA 2: ZBIORNIK I POMPA (Timer TON) ---

    Sys.Alarm_Low  = (In.Tank_Level < TANK_MIN);
    Sys.Alarm_High = (In.Tank_Level > TANK_MAX);

    T_PumpDelay.IN = (In.Tank_Level < PUMP_START_LVL);
    TON_Update(&T_PumpDelay);

    if (T_PumpDelay.Q) Out.Pump = true;
    if (In.Tank_Level > PUMP_STOP_LVL || Sys.Alarm_High) Out.Pump = false;


    // --- SEKCJA 3: MIESZALNIK (Timer TON + Sekwencja) ---

    if (In.Btn_Start) Sys.Mixer_Running = true;

    T_Mixer.IN = Sys.Mixer_Running;
    TON_Update(&T_Mixer);

    Out.RGB_R = 0; Out.RGB_G = 0; Out.RGB_B = 0;

    if (Sys.Mixer_Running) {
        uint32_t t = T_Mixer.ET;

        if (t < MIXER_PHASE_1)      Out.RGB_B = true;
        else if (t < MIXER_PHASE_2) Out.RGB_G = true;
        else                        Out.RGB_R = true;

        if (T_Mixer.Q) {
            Sys.Mixer_Running = false;
            T_Mixer.IN = false;
            TON_Update(&T_Mixer);
        }
    }
}

// ============================================================================
// OBSŁUGA SPRZĘTU (HAL - Hardware Abstraction Layer)
// ============================================================================

void ReadInputs(void) {
    // 1. Odczyt ADC (Analogowy poziom wody)
    HAL_ADC_Start(&hadc1); // Uruchom konwersję
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) { // Czekaj max 10ms
        uint32_t raw_val = HAL_ADC_GetValue(&hadc1);
        // Przeliczenie 0-4095 na 0-100%
        In.Tank_Level = (raw_val * 100) / 4095;
    }

    // 2. Odczyt Czujnika Ultradźwiękowego (Odległość)
    In.Dist_CM = HCSR04_Read();

    // Logika Czujnika: Auto wykryte jeśli bliżej niż 15cm i pomiar poprawny (>0)
    In.Car_Sensor = (In.Dist_CM > 0 && In.Dist_CM < 15);

    // 3. Odczyt Przycisków (Zwierają do masy -> Stan niski oznacza wciśnięcie)
    bool btn_phys_entry = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET);

    // Suma logiczna: Wjazd aktywny jeśli wciśnięto przycisk LUB czujnik wykrył auto
    In.Btn_Entry = btn_phys_entry || In.Car_Sensor;

    In.Btn_Exit  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
    In.Btn_Start = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
}

void WriteOutputs(void) {
    // Przepisanie stanów logicznych na fizyczne piny procesora

    // Pompa (PA8)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, Out.Pump ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Parking (PB4, PB5)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, Out.Led_Full ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, Out.Led_Free ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Mieszalnik RGB (PA9, PC7, PB6)
    // Zakładamy Wspólną Anodę: Stan RESET włącza diodę, SET wyłącza.
    // Jeśli masz Wspólną Katodę, usuń wykrzykniki (zamień logikę)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, Out.RGB_R ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, Out.RGB_G ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, Out.RGB_B ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Safety_Check(void) {
    // BŁĄD KRYTYCZNY: Auto stoi na czujniku wjazdu (blokuje szlaban),
    // a ktoś próbuje wymusić wyjazd (symulacja kolizji logicznej).
    if (In.Car_Sensor && In.Btn_Exit) {
        Sys.Critical_Error = true;
    }

    if (Sys.Critical_Error) {
        // SAFE STATE: Natychmiast wyłącz niebezpieczne urządzenia
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Pompa STOP

        // Wyświetl błąd na ekranie
        printf("\033[2J\033[H");
        printf("!!! BLAD KRYTYCZNY: KOLIZJA LOGICZNA !!!\r\n");
        printf("System zatrzymany. Czekam na reset Watchdoga...\r\n");

        // Zablokuj procesor w pętli nieskończonej.
        // Po ok. 4 sekundach zadziała Watchdog (IWDG) i zresetuje układ.
        while(1);
    }
}

// Obsługa czujnika HC-SR04
uint32_t HCSR04_Read(void) {
    uint32_t local_time = 0;

    // 1. Wysłanie impulsu Trigger (10us)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); for(int i=0;i<500;i++);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   for(int i=0;i<2000;i++);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

    // 2. Czekanie na sygnał Echo (z zabezpieczeniem timeout)
    uint32_t t = 50000;
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) {
        if(t-- == 0) return 0; // Błąd: Brak echa
    }

    // 3. Pomiar czasu trwania stanu wysokiego
    t = 50000;
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) {
        local_time++;
        if(t-- == 0) break; // Zabezpieczenie przed zawieszeniem
    }

    return local_time / 50; // Kalibracja (zależna od zegara MCU)
}

// Pomocnicza funkcja rysująca pasek postępu
void DrawBar(uint8_t val) {
    printf("[");
    for(int i=0; i<20; i++) printf((i < val/5) ? "#" : ".");
    printf("]");
}

// Funkcja rysująca Dashboard (Interfejs Użytkownika)
void PrintDashboard(void) {
    // \033[H - Ustaw kursor na początek
    // \033[K - Wyczyść linię (usuwa stare znaki)

    printf("\033[H");
    printf("=== FABRYKA PRO V4.0 (PLC ENGINE) ===        \033[K\r\n\r\n");

    printf("[1] ZBIORNIK (ADC + TON)                    \033[K\r\n");
    printf("    Poziom: "); DrawBar(In.Tank_Level); printf(" %d%%\033[K\r\n", In.Tank_Level);
    printf("    Pompa:  %s  (T_Pump.ET: %lu ms)\033[K\r\n",
           Out.Pump ? "\033[32m[ON] \033[0m" : "[OFF]", T_PumpDelay.ET);

    printf("--------------------------------------------\033[K\r\n");

    printf("[2] PARKING (CTU + Backup MEM)              \033[K\r\n");
    printf("    Licznik: [%d / %d] (CV / PV)\033[K\r\n", C_Parking.CV, C_Parking.PV);
    printf("    Status:  %s\033[K\r\n",
           Out.Led_Full ? "\033[31m[ PELNY ]\033[0m" : "\033[32m[ WOLNY ]\033[0m");
    printf("    Sensor:  %lu cm\033[K\r\n", In.Dist_CM);

    printf("--------------------------------------------\033[K\r\n");

    printf("[3] MIESZALNIK (TON + Sekwencja)            \033[K\r\n");
    printf("    Status:  ");
    if(Out.RGB_B) printf("\033[34m[ NALEWANIE ]\033[0m");
    else if(Out.RGB_G) printf("\033[32m[ MIESZANIE ]\033[0m");
    else if(Out.RGB_R) printf("\033[31m[ WYLEWANIE ]\033[0m");
    else printf("[ STOP ]     ");

    printf(" (Czas: %lu / %d ms)\033[K\r\n", T_Mixer.ET, MIXER_TIME_TOTAL);

    // Wyczyść resztę ekranu poniżej
    printf("\033[J");
}

// ============================================================================
// KONFIGURACJA PERYFERIÓW (GENEROWANA PRZEZ CUBEMX)
// ============================================================================

void SystemClock_Config(void) { RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI; RCC_OscInitStruct.HSIState = RCC_HSI_ON; RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; RCC_OscInitStruct.LSIState = RCC_LSI_ON; RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; RCC_OscInitStruct.PLL.PLLM = 1; RCC_OscInitStruct.PLL.PLLN = 10; RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7; RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; HAL_RCC_OscConfig(&RCC_OscInitStruct); RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4); }
static void MX_GPIO_Init(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET); GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_1; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_10; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_10; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); }
static void MX_USART2_UART_Init(void) { huart2.Instance = USART2; huart2.Init.BaudRate = 115200; huart2.Init.WordLength = UART_WORDLENGTH_8B; huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX; huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16; HAL_UART_Init(&huart2); }
static void MX_ADC1_Init(void) { ADC_ChannelConfTypeDef sConfig = {0}; hadc1.Instance = ADC1; hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; hadc1.Init.Resolution = ADC_RESOLUTION_12B; hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; hadc1.Init.LowPowerAutoWait = DISABLE; hadc1.Init.ContinuousConvMode = DISABLE; hadc1.Init.NbrOfConversion = 1; hadc1.Init.DiscontinuousConvMode = DISABLE; hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; hadc1.Init.DMAContinuousRequests = DISABLE; hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED; hadc1.Init.OversamplingMode = DISABLE; HAL_ADC_Init(&hadc1); sConfig.Channel = ADC_CHANNEL_5; sConfig.Rank = ADC_REGULAR_RANK_1; sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; sConfig.SingleDiff = ADC_SINGLE_ENDED; sConfig.OffsetNumber = ADC_OFFSET_NONE; sConfig.Offset = 0; HAL_ADC_ConfigChannel(&hadc1, &sConfig); }
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;

  // Zmiana 1: Dzielnik 32 sprawia, że 1 jednostka licznika to ok. 1 ms
  // (32kHz / 32 = 1000 Hz)
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;

  hiwdg.Init.Window = 4095; // Okno otwarte cały czas (standard)

  // Zmiana 2: Wartość 300 oznacza teraz 300 milisekund
  // To idealny czas dla cyklu PLC trwającego 50ms (6x zapas)
  hiwdg.Init.Reload = 300;

  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}
static void MX_RTC_Init(void) { hrtc.Instance = RTC; hrtc.Init.HourFormat = RTC_HOURFORMAT_24; hrtc.Init.AsynchPrediv = 127; hrtc.Init.SynchPrediv = 255; hrtc.Init.OutPut = RTC_OUTPUT_DISABLE; HAL_RTC_Init(&hrtc); }

static void MX_TIM2_Init(void) { TIM_ClockConfigTypeDef sClockSourceConfig = {0}; TIM_MasterConfigTypeDef sMasterConfig = {0}; htim2.Instance = TIM2; htim2.Init.Prescaler = 7999; htim2.Init.CounterMode = TIM_COUNTERMODE_UP; htim2.Init.Period = 999; htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; HAL_TIM_Base_Init(&htim2); sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig); sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig); }

void Error_Handler(void) { __disable_irq(); while (1) {} }
