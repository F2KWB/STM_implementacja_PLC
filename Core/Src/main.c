/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include "plc_blocks.h"
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
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define LICZBA_PAMIECI_M 16
#define LICZBA_WEJSC 4
#define LICZBA_WYJSC 4
uint8_t stanyWejsc[LICZBA_WEJSC] = { 0 };
uint8_t stanyWyjsc[LICZBA_WYJSC] = { 0 };
uint8_t M[LICZBA_PAMIECI_M] = { 0 };


// --- Zmienne dla logiki automatycznej ---
#define LICZBA_TIMEROW 2
TON_Block Timer1 = { .PT = 2000 }; // Timer 2 sekundy
//TOF_Block Timer2 = { .PT = 5000 }; // Timer 5 sekund
//CTU_Block CTU_1 = { .PV = 4 }; // Ustaw Preset Value
//CTD_Block CTD_1 = { .PV = 4 };

//---------------------------------------------------------
// ... Twoje poprzednie zmienne ...
bool last_M0 = false;
bool last_Q = false;
//---------------------------------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void ReadInputs(void);
void HandlePCCommand(uint8_t rx_byte);
void WriteOutputs(void);
void ExecuteAutomaticLogic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	if (ch == '\n') {
		__io_putchar('\r');
	}

	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);

	return 1;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_IWDG_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2); // Uruchom licznik sprzętowy 2
	HAL_TIM_Base_Start(&htim4); // Uruchom licznik sprzętowy 4
	/* USER CODE END 2 */


	/* Infinite loop */
		/* USER CODE BEGIN WHILE */
		while (1) {
			// 1. Wczytanie fizycznych wejść
			ReadInputs();

			// 2. Wykonanie logiki automatycznej (PLC)
			ExecuteAutomaticLogic();

			// 3. DETEKCJA ZMIAN I RAPORTOWANIE (Nowy blok)
			// Sprawdzamy, czy obecny stan różni się od zapamiętanego
			if (M[0] != last_M0 || Timer1.Q != last_Q) {
			    HandlePCCommand('s');
			    last_M0 = M[0];
			    last_Q = Timer1.Q;
			}

			// 4. Sprawdzenie komend z PC (UART)
			uint8_t rx_byte_local; // Deklaracja tylko RAZ w tym miejscu
			if (HAL_UART_Receive(&huart2, &rx_byte_local, 1, 10) == HAL_OK) {
				HandlePCCommand(rx_byte_local);
			}

			// 5. Ustawienie fizycznych wyjść
			WriteOutputs();

			// 6. Kontrola czasu i watchdog
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(50);

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
			// Zostaw to miejsce puste - cała logika jest powyżej w sekcji WHILE
		}
		/* USER CODE END 3 */
//	/* Infinite loop */
//	/* USER CODE BEGIN WHILE */
//	while (1) {
//		/* USER CODE END WHILE */
//
//		/* USER CODE BEGIN 3 */
//		/* ### POCZĄTEK CYKLU SKANU PLC ### */
//
//		// 1. Wczytanie fizycznych wejść
//		ReadInputs();
//
//		// >>> 2. Wykonanie logiki automatycznej <<<
//		ExecuteAutomaticLogic();
//
//		// 3. Sprawdzenie, czy nadeszła komenda z PC
//		uint8_t rx_byte_local; // Użyj lokalnej zmiennej
//		if (HAL_UART_Receive(&huart2, &rx_byte_local, 1, 10) == HAL_OK) {
//			HandlePCCommand(rx_byte_local);
//		}
//
//		// 4. Ustawienie fizycznych wyjść
//		WriteOutputs();
//
//		// 5. Kontrola czasu i watchdog
//		HAL_IWDG_Refresh(&hiwdg);
//		HAL_Delay(50);
//
//		/* ### KONIEC CYKLU SKANU PLC ### */
//	}
//	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 7999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, OUT_4_Pin | OUT_1_Pin | OUT_2_Pin | OUT_3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : IN_4_Pin */
	GPIO_InitStruct.Pin = IN_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(IN_4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OUT_4_Pin OUT_1_Pin OUT_2_Pin OUT_3_Pin */
	GPIO_InitStruct.Pin = OUT_4_Pin | OUT_1_Pin | OUT_2_Pin | OUT_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : IN_1_Pin IN_2_Pin IN_3_Pin */
	GPIO_InitStruct.Pin = IN_1_Pin | IN_2_Pin | IN_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//ODCZYT WEJŚĆ
void ReadInputs(void) {
	stanyWejsc[0] = !HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin);
	stanyWejsc[1] = !HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin);
	stanyWejsc[2] = !HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin);
	stanyWejsc[3] = !HAL_GPIO_ReadPin(IN_4_GPIO_Port, IN_4_Pin);
}
//-------------------------------------------------------------------
/* USER CODE BEGIN 4 */
void HandlePCCommand(uint8_t rx_byte) {
    switch (rx_byte) {
        case 's':
        case 'S':
            printf("\033[2J\033[H"); // Czyści ekran
            printf("======= PANEL OPERATORSKI PLC =======\r\n");
            printf("STATUS: %s\r\n", M[0] ? "PRACA (RUN)" : "STOP");
            printf("-------------------------------------\r\n");
            printf("Silnik (OUT_2): [%s]\r\n", Timer1.Q ? "ON " : "OFF");
            printf("Czas timera:    %lu / %lu ms\r\n", Timer1.ET, Timer1.PT);
            printf("=====================================\r\n");
            break;
    }
}
/* USER CODE END 4 */
////LOGIKA PROGRAMU
//void HandlePCCommand(uint8_t rx_byte) {
//    switch (rx_byte)
//    {
//        // Jedyne co zostawiamy, to Status
//        case 'S':
//        case 's':
//            printf("===== STATUS =====\r\n");
//            // Pokaż stany fizycznych wejść
//            printf("WEJSCIA: I0(Start)=%d, I1(Stop)=%d\r\n",
//                   stanyWejsc[0], stanyWejsc[1]);
//
//            // Pokaż stany kluczowych bitów programu
//            printf("M[0]=%d\r\n", M[0]);
//            printf("T1 (TON): IN=%d, Q=%d, ET=%lu/%lu ms\r\n",
//                   Timer1.IN, Timer1.Q, Timer1.ET, Timer1.PT);
//
//            // Pokaż stany fizycznych wyjść
//            printf("WYJSCIA: O0(Run)=%d, O1(Silnik)=%d\r\n",
//                   stanyWyjsc[0], stanyWyjsc[1]);
//            printf("==================\r\n");
//            break;
//
//        default:
//            printf("Nieznana komenda: %c (Uzyj 's' dla statusu)\r\n", rx_byte);
//            break;
//    }
//}
//-------------------------------------------------------------------
void ExecuteAutomaticLogic(void) {
    // Jeśli wciśnięto START (I0) -> Ustaw bit pamięci M0
    if (stanyWejsc[0] == 1) {
        M[0] = 1;
    }
    if (stanyWejsc[1] == 1) {
            M[0] = 0;
        }
    Timer1.IN = M[0];
    TON_Update(&Timer1);

    stanyWyjsc[0] = M[0];
    	    stanyWyjsc[1] = Timer1.Q;
    	    stanyWyjsc[2] = 0;
    	    stanyWyjsc[3] = 0;
}

// ZAPIS WYJŚĆ
	void WriteOutputs(void) {
		HAL_GPIO_WritePin(OUT_1_GPIO_Port, OUT_1_Pin, stanyWyjsc[0]);
		HAL_GPIO_WritePin(OUT_2_GPIO_Port, OUT_2_Pin, stanyWyjsc[1]);
		HAL_GPIO_WritePin(OUT_3_GPIO_Port, OUT_3_Pin, stanyWyjsc[2]);
		HAL_GPIO_WritePin(OUT_4_GPIO_Port, OUT_4_Pin, stanyWyjsc[3]);
	}
	/* USER CODE END 4 */

	/**
	 * @brief  This function is executed in case of error occurrence.
	 * @retval None
	 */
	void Error_Handler(void) {
		/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		__disable_irq();
		while (1) {
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
