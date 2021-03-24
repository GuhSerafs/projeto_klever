/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 * Criado por: Gustavo Duarte Serafim
 * Última Edição: 23/03/2021
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Bytes Relativos às Tarefas
#define FLAG_GERENCIADOR 	0x02
#define FLAG_LIGA 			0x04
#define FLAG_DESLIGA 		0x08
#define FLAG_TOGGLE 		0x10
#define FLAG_ADC 			0x20
#define FLAG_LOOPBACK 		0x40
#define MSG_ACK				0x8f

// Indices Importanes do Frame de Dados
#define INDICE_INICIO_FRAME 	0
#define INDICE_CMD_FRAME 		1
#define INDICE_QTD_PARAMS		2
#define INDICE_INICIO_PARAMS	3

// Constantes do Projeto
#define QUEUE_SIZE 	32
#define PERIODO_ADC 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for Gerenciador */
osThreadId_t GerenciadorHandle;
const osThreadAttr_t Gerenciador_attributes = { .name = "Gerenciador",
		.priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 4 };
/* Definitions for LerAnalogica */
osThreadId_t LerAnalogicaHandle;
const osThreadAttr_t LerAnalogica_attributes = { .name = "LerAnalogica",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for ComandarLed */
osThreadId_t ComandarLedHandle;
const osThreadAttr_t ComandarLed_attributes = { .name = "ComandarLed",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for MainQueue */
osMessageQueueId_t MainQueueHandle;
const osMessageQueueAttr_t MainQueue_attributes = { .name = "MainQueue" };
/* USER CODE BEGIN PV */

uint8_t rx_ans = 0;
uint8_t rx_buffer[QUEUE_SIZE] = { 0 };
uint8_t tx_buffer[QUEUE_SIZE] = { 0 };
uint16_t valor_analogico = 0;

osEventFlagsId_t flags_comando;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void GerenciadorTask(void *argument);
void LerAnalogicaTask(void *argument);
void ComandarLedTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart2, &rx_ans, 1);
	HAL_ADC_Start_IT(&hadc1);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of MainQueue */
	MainQueueHandle = osMessageQueueNew(32, sizeof(uint8_t),
			&MainQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of Gerenciador */
	GerenciadorHandle = osThreadNew(GerenciadorTask, NULL,
			&Gerenciador_attributes);

	/* creation of LerAnalogica */
	LerAnalogicaHandle = osThreadNew(LerAnalogicaTask, NULL,
			&LerAnalogica_attributes);

	/* creation of ComandarLed */
	ComandarLedHandle = osThreadNew(ComandarLedTask, NULL,
			&ComandarLed_attributes);

	/* USER CODE BEGIN RTOS_THREADS */

	// Obtém um ID para o FlagsComando
	flags_comando = osEventFlagsNew(NULL);

	// Inicializa as flags do comando zerando-as
	osEventFlagsClear(flags_comando,
			FLAG_GERENCIADOR | FLAG_LIGA | FLAG_DESLIGA | FLAG_TOGGLE | FLAG_ADC
					| FLAG_LOOPBACK);

	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static uint8_t rx_index = 0;
	static uint8_t params = 0;
	static uint16_t chksum = 0;

	// Flag necessária para saber se abre ou não a comunicação ao final do recebimento
	uint8_t sucesso = 0;

	if (huart->Instance == USART2) {
		// Implementação do protocolo

		if (rx_ans == 0x01 && rx_index == INDICE_INICIO_FRAME) { // Inicio do frame de dados
			rx_buffer[rx_index] = rx_ans;
			rx_index++;
			chksum += rx_ans;
		} else if (rx_index == INDICE_CMD_FRAME) { // Segundo byte de dados
			if (rx_ans == FLAG_LIGA || rx_ans == FLAG_DESLIGA
					|| rx_ans == FLAG_TOGGLE || rx_ans == FLAG_ADC
					|| rx_ans == FLAG_LOOPBACK) {
				// Recebeu um comando válido
				rx_buffer[rx_index] = rx_ans;
				rx_index++;
				chksum += rx_ans;
			} else {
				// Não responde e reseta comunicação
				rx_index = 0;
				params = 0;
				chksum = 0;
			}
		} else if (rx_index == INDICE_QTD_PARAMS) { // Terceiro byte de dados
			// Momento em que o numero de parametros é fornecido
			params = rx_ans;
			rx_buffer[rx_index] = rx_ans;
			rx_index++;
			chksum += rx_ans;
		} else if (rx_index > (params + INDICE_QTD_PARAMS)) { // Momento em que recebe o checksum
			if ((rx_ans & 0xff) == (chksum & 0xff)) {
				// Mensagem válida recebida
				sucesso = SET;
				// Colocar o comando na fila
				osMessageQueuePut(MainQueueHandle, &rx_buffer[INDICE_CMD_FRAME],
						0xff, 0);

				// Se o comando for o loopback, coloca os parâmetros recebidos na fila
				if (rx_buffer[INDICE_CMD_FRAME] == FLAG_LOOPBACK) {
					for (int i = INDICE_QTD_PARAMS;
							i <= (params + INDICE_QTD_PARAMS); i++) {
						osMessageQueuePut(MainQueueHandle, &rx_buffer[i], 0xff,
								0);

						if (i > QUEUE_SIZE)
							break;
					}
				}

				// Enviar uma flag para a thread de gerenciamento
				osEventFlagsSet(flags_comando, FLAG_GERENCIADOR);

				// Resetar a comunicação
				rx_index = 0;
				params = 0;
				chksum = 0;

			} else {
				// Não responde e reseta comunicação
				rx_index = 0;
				params = 0;
				chksum = 0;
			}
		} else if (rx_index > 0 && rx_index < QUEUE_SIZE) {
			// Recebimento dos parâmetros extras
			rx_buffer[rx_index] = rx_ans;
			rx_index++;
			chksum += rx_ans;
		} else {
			// Se alguma das condições falhar, reseta o recebimento do frame
			rx_index = 0;
			params = 0;
			chksum = 0;
		}

		// Abrir o Rx novamente se o recebimento ainda não estiver finalizado
		// Se houver sucesso, interrompe o recebimento até atender o comando
		if (!sucesso)
			HAL_UART_Receive_IT(&huart2, &rx_ans, 1);

	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	// Fazer as tarefas necessárias
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_GerenciadorTask */
/**
 * @brief  Function implementing the Gerenciador thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_GerenciadorTask */
void GerenciadorTask(void *argument) {
	/* USER CODE BEGIN 5 */
	// Variáveis globais do gerenciador
	uint8_t cmd = 0;
	uint8_t params = 0;
	uint16_t chksum = 0;
	/* Infinite loop */
	for (;;) {
		osEventFlagsWait(flags_comando, FLAG_GERENCIADOR, osFlagsWaitAll,
		HAL_MAX_DELAY);

		// Inicia a construção do buffer de resposta
		tx_buffer[INDICE_INICIO_FRAME] = 0x01;

		osMessageQueueGet(MainQueueHandle, &cmd, (void*) 0xff, 0);

		switch (cmd) {
		case FLAG_LOOPBACK:
			// Envia um eco da mensagem
			tx_buffer[INDICE_CMD_FRAME] = FLAG_LOOPBACK;
			osMessageQueueGet(MainQueueHandle, &params, (void*) 0xff, 0);
			tx_buffer[INDICE_QTD_PARAMS] = params;

			for (int i = INDICE_INICIO_PARAMS;
					i <= params + INDICE_INICIO_PARAMS; i++) {
				osMessageQueueGet(MainQueueHandle, &tx_buffer[i], (void*) 0xff,
						0);
			}
			chksum = 0;
			for (int i = 0; i < params + INDICE_INICIO_PARAMS; i++) {
				chksum += tx_buffer[i];
			}
			tx_buffer[params + INDICE_INICIO_PARAMS] = (chksum & 0xff);

			HAL_UART_Transmit_IT(&huart2, tx_buffer,
					params + INDICE_INICIO_PARAMS + 1);
			break;
		case FLAG_LIGA:
		case FLAG_DESLIGA:
		case FLAG_TOGGLE:
			// Faz a leitura dos parâmetros para liberar a fila
			osMessageQueueGet(MainQueueHandle, &params, (void*) 0xff, 0);

			// Coloca de volta a flag na fila, para ser consumida na função específica
			osMessageQueuePut(MainQueueHandle, &cmd, 0xff, 0);
			tx_buffer[INDICE_CMD_FRAME] = MSG_ACK;

			// Termina de construir o buffer de sáida
			tx_buffer[INDICE_QTD_PARAMS] = 0x00; // comando de LED não vai parâmetros

			// Faz o checksum
			tx_buffer[INDICE_QTD_PARAMS + 1] = (tx_buffer[INDICE_INICIO_FRAME]
					+ tx_buffer[INDICE_CMD_FRAME]) & 0xff;

			// Envia o frame de dados pra serial
			HAL_UART_Transmit_IT(&huart2, tx_buffer, 4);

			// Ativa a thread pra ligar o LED
			osEventFlagsSet(flags_comando, cmd);
			break;

		case FLAG_ADC:
			// Faz a leitura dos parâmetros para liberar a fila
			osMessageQueueGet(MainQueueHandle, &params, (void*) 0xff, 0);
			tx_buffer[INDICE_CMD_FRAME] = FLAG_ADC;

			// Bytes de resposta = 2 bytes
			tx_buffer[INDICE_QTD_PARAMS] = 0x02;

			// MSB do valor analógico
			tx_buffer[INDICE_INICIO_PARAMS] = (valor_analogico >> 8) & 0x0f;

			// LSB do valor analógico
			tx_buffer[INDICE_INICIO_PARAMS + 1] = valor_analogico & 0xff;

			// Checksum
			chksum = 0;
			for (int i = 0; i < (INDICE_INICIO_PARAMS+2); i++) {
				chksum += tx_buffer[i];
			}
			tx_buffer[(INDICE_INICIO_PARAMS+2)] = (chksum & 0xff);

			HAL_UART_Transmit_IT(&huart2, tx_buffer, (INDICE_INICIO_PARAMS+3));
			break;

		default:
			break;
		}

		// Fim do switch case, o comando foi atendido...
		// Habilita novamente a entrada serial
		HAL_UART_Receive_IT(&huart2, &rx_ans, 0x1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LerAnalogicaTask */
/**
 * @brief Function implementing the LerAnalogica thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LerAnalogicaTask */
void LerAnalogicaTask(void *argument) {
	/* USER CODE BEGIN LerAnalogicaTask */
	/* Infinite loop */
	for (;;) {
		// Armazena o valor contido no DR do ADC
		valor_analogico = HAL_ADC_GetValue(&hadc1);

		// Engatilha outra conversão
		HAL_ADC_Start_IT(&hadc1);
		osDelay(PERIODO_ADC);
	}
	/* USER CODE END LerAnalogicaTask */
}

/* USER CODE BEGIN Header_ComandarLedTask */
/**
 * @brief Function implementing the ComandarLed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ComandarLedTask */
void ComandarLedTask(void *argument) {
	/* USER CODE BEGIN ComandarLedTask */
	uint8_t cmd = 0;
	/* Infinite loop */
	for (;;) {
		osEventFlagsWait(flags_comando,
		FLAG_LIGA | FLAG_DESLIGA | FLAG_TOGGLE, osFlagsWaitAny,
		HAL_MAX_DELAY);

		// Leitura do comando
		osMessageQueueGet(MainQueueHandle, &cmd, (void*) 0xff, 0);

		switch (cmd) {
		case FLAG_LIGA:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
			break;
		case FLAG_DESLIGA:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
			break;
		case FLAG_TOGGLE:
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			break;
		}
	}
	/* USER CODE END ComandarLedTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM10 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM10) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
