/*
 BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
 All rights reserved

 File Name     : H12R0.c
 Description   : Source code for module H12R0.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Define UART variables */
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] = { { .paramPtr = NULL,
		.paramFormat = FMT_FLOAT, .paramName = "" } };

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;


uint32_t Volt_buffer[1] = { 0 };
float raw_adc, adc_volt, measured_volt;


TIM_HandleTypeDef htim3;

TaskHandle_t VoltageHandle = NULL;
TimerHandle_t xTimer = NULL;
uint8_t startMeasurementRanging = STOP_MEASUREMENT_RANGING;




float DATA_To_SEND = 0.0f;     //float
/* Private variables ---------------------------------------------------------*/



#define IDLE_CASE                0
#define STREAM_CLI_CASE          1
#define STREAM_PORT_CASE         2
#define STREAM_BUFFER_CASE       3
#define STREAM_CLI_VERBOSE_CASE  4
#define SAMPLE_CLI_CASE          6
#define SAMPLE_PORT_CASE         7
#define SAMPLE_BUFFER_CASE       8
#define SAMPLE_CLI_VERBOSE_CASE  9


uint8_t global_port, global_module, global_mode;
uint32_t global_period, global_timeout;


float volt_buffer;
float *ptr_volt_buffer;

uint8_t H12R0_DATA_FORMAT = FMT_FLOAT;

float H12R0_Volt;

/* Private function prototypes -----------------------------------------------*/
int
SendResults(float message, uint8_t Mode, uint8_t unit, uint8_t Port,
		uint8_t Module, float *Buffer);


float
CalculationVolt(void);

//static void HandleTimeout(TimerHandle_t xTimer);

Module_Status
SampleRead(float *temp);


Module_Status
StreamVToPort(uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout);

void
VoltageTask(void *argument);
void
TimerTask(void *argument);
static void
CheckForEnterKey(void);
static void
HandleTimeout(TimerHandle_t xTimer);

/* Create CLI commands --------------------------------------------------------*/

static portBASE_TYPE
sampleCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);
static portBASE_TYPE
streamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);
static portBASE_TYPE
stopCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);
static portBASE_TYPE
unitCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);
static portBASE_TYPE
demoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*-----------------------------------------------------------*/

/* CLI command structure : demo */
const CLI_Command_Definition_t demoCommandDefinition =
		{ (const int8_t*) "demo", /* The command string to type. */
				(const int8_t*) "demo:\r\n Run a demo program to test module functionality\r\n\r\n",
				demoCommand, /* The function to run. */
				0
		/* one parameter is expected. */
		};

/* CLI command structure : sample */
const CLI_Command_Definition_t sampleCommandDefinition =
		{ (const int8_t*) "sample", /* The command string to type. */
				(const int8_t*) "sample:\r\n Take one sample with the determinated unit \r\n\r\n",
				sampleCommand, /* The function to run. */
				0
		/* one parameter is expected. */
		};

/* CLI command structure : stream */
//const CLI_Command_Definition_t streamCommandDefinition =
//		{ (const int8_t*) "stream", /* The command string to type. */
//				(const int8_t*) "stream:\r\n Stream temp to the CLI, buffer or port with period (ms) and total time (ms). \r\n\r\n",
//				streamCommand, /* The function to run. */
//				-1
//		/* Multiparameters are expected. */
//		};

const CLI_Command_Definition_t streamCommandDefinition =
		{ (const int8_t*) "stream", /* The command string to type. */
				(const int8_t*) "stream:\r\n Stream volt to the CLI, buffer or port with period (ms) and total time (ms). \r\n\r\n",
				streamCommand, /* The function to run. */
				-1
		/* Multiparameters are expected. */
		};

/* CLI command structure : unit */
const CLI_Command_Definition_t unitCommandDefinition = { (const int8_t*) "unit", /* The command string to type. */

(const int8_t*) "unit:\r\n Set the measurement unit (V)\r\n\r\n",
		unitCommand, /* The function to run. */
		1
/* one parameter is expected. */
};


/* CLI command structure : stop */
const CLI_Command_Definition_t stopCommandDefinition =
		{ (const int8_t*) "stop", /* The command string to type. */
				(const int8_t*) "stop:\r\n Stop streaming \r\n\r\n",
				stopCommand, /* The function to run. */
				0
		/* No parameters are expected. */
		};



/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 -----------------------------------------------------------------------
 */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	__SYSCFG_CLK_ENABLE()
	;

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/*-----------------------------------------------------------*/
/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void) {
	BOS_Status result = BOS_OK;
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 2, temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] = { 0 };

	HAL_FLASH_Unlock();

	/* Erase RO area */
	FLASH_PageErase(RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	if (FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if (myID) {
		temp = (uint16_t) (N << 8) + myID;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS, temp);
		FlashStatus = FLASH_WaitForLastOperation(
				(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

		/* Save topology */
		for (uint8_t i = 1; i <= N; i++) {
			for (uint8_t j = 0; j <= MaxNumOfPorts; j++) {
				if (array[i - 1][0]) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
					RO_START_ADDRESS + add, array[i - 1][j]);
					add += 2;
					FlashStatus = FLASH_WaitForLastOperation(
							(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for (uint8_t s = 0; s < numOfRecordedSnippets; s++) {
		if (snippets[s].cond.conditionType) {
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy((uint8_t*) &snipBuffer[1], (uint8_t*) &snippets[s],
					sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for (uint8_t j = 0; j < (sizeof(snippet_t) / 2); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd,
						*(uint16_t*) &snipBuffer[j * 2]);
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for (uint8_t j = 0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd,
						*(uint16_t*) (snippets[s].cmd + j * 2));
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void) {
	// Clear the array 
	memset(array, 0, sizeof(array));
	N = 1;
	myID = 0;

	return SaveToRO();
}
/* --- H12R0 module initialization ---
 */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART6_UART_Init();

	xTaskCreate(VoltageTask, (const char* ) "VoltageTask",
			(2*configMINIMAL_STACK_SIZE), NULL,
			osPriorityNormal - osPriorityIdle, &VoltageHandle);

	MX_ADC_Init();
	ADC_Channel_config();
}




/*-----------------------------------------------------------*/

/* --- H12R0 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src,
		uint8_t dst, uint8_t shift) {

	Module_Status result = H12R0_OK;
	uint32_t period = 0;
	uint32_t timeout = 0;

	switch (code) {
	case (CODE_H12R0_STREAM_PORT):
			period = ((uint32_t) cMessage[port - 1][1 + shift] << 24)
					+ ((uint32_t) cMessage[port - 1][2 + shift] << 16)
					+ ((uint32_t) cMessage[port - 1][3 + shift] << 8)
					+ cMessage[port - 1][4 + shift];
			timeout = ((uint32_t) cMessage[port - 1][5 + shift] << 24)
					+ ((uint32_t) cMessage[port - 1][6 + shift] << 16)
					+ ((uint32_t) cMessage[port - 1][7 + shift] << 8)
					+ cMessage[port - 1][8 + shift];
			StreamVToPort(cMessage[port - 1][9 + shift],
					cMessage[port - 1][10 + shift], period, timeout);
			break;

	case (CODE_H12R0_STOP):
			global_mode = IDLE_CASE;
			xTimerStop(xTimer, portMAX_DELAY);
			break;

	case (CODE_H12R0_SAMPLE):
			SampleRead(&H12R0_Volt);
			break;
	default:
		result = H12R0_ERR_UnknownMessage;
		break;
	}

	return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands 
 */
void RegisterModuleCLICommands(void) {

	FreeRTOS_CLIRegisterCommand(&sampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&streamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&stopCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&unitCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&demoCommandDefinition);

}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART6)
		return P3;
	else if (huart->Instance == USART3)
		return P4;
	else if (huart->Instance == USART1)
		return P5;

	return 0;
}

/*-----------------------------------------------------------*/

/* --- send results to x --- */
int SendResults(float message, uint8_t Mode, uint8_t Unit, uint8_t Port,
		uint8_t Module, float *Buffer) {
	float Raw_Msg = 0.0f;
	uint32_t RawMsgInt = 0;
	int8_t *pcOutputString;
	static const int8_t *pcWeightMsg = (int8_t*) "Volt (%s): %f\r\n";
	static const int8_t *pcWeightVerboseMsg = (int8_t*) "%f\r\n";
	static const int8_t *pcWeightMsgUINT = (int8_t*) "Volt (%s): %d\r\n";
	static const int8_t *pcWeightVerboseMsgUINT = (int8_t*) "%d\r\n";
	char *strUnit;
	static uint8_t temp[4];
	/* specify the unit */

	Raw_Msg = message;

	/* Get CLI output buffer */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	if (Mode != STREAM_CLI_VERBOSE_CASE && Mode != STREAM_PORT_CASE)
	{
			strUnit = malloc(6 * sizeof(char));
			memset(strUnit, 0, (6 * sizeof(char)));
			sprintf((char*) strUnit, "Volt");
	}

	// Send the value to appropriate outlet
	switch (Mode) {
	case SAMPLE_CLI_CASE:
	case STREAM_CLI_CASE:
		if (H12R0_DATA_FORMAT == FMT_UINT32) {
			RawMsgInt = Raw_Msg * 10;
			sprintf((char*) pcOutputString, (char*) pcWeightMsgUINT, strUnit,
					RawMsgInt);
			writePxMutex(PcPort, (char*) pcOutputString,
					strlen((char*) pcOutputString), cmd500ms,
					HAL_MAX_DELAY);
			CheckForEnterKey();
		} else if (H12R0_DATA_FORMAT == FMT_FLOAT) {
			sprintf((char*) pcOutputString, (char*) pcWeightMsg, strUnit,
					Raw_Msg);
			writePxMutex(PcPort, (char*) pcOutputString,
					strlen((char*) pcOutputString), cmd500ms,
					HAL_MAX_DELAY);
			CheckForEnterKey();
		}
		break;

	case SAMPLE_PORT_CASE:
	case STREAM_PORT_CASE:
		if (H12R0_DATA_FORMAT == FMT_UINT32) {
			RawMsgInt = Raw_Msg * 10;
			if (Module == myID) {
				temp[0] = *((__IO uint8_t*) (&RawMsgInt) + 3);
				temp[1] = *((__IO uint8_t*) (&RawMsgInt) + 2);
				temp[2] = *((__IO uint8_t*) (&RawMsgInt) + 1);
				temp[3] = *((__IO uint8_t*) (&RawMsgInt) + 0);
				writePxMutex(Port, (char*) &temp, 4 * sizeof(uint8_t), 10, 10);
			} else {
				messageParams[0] = Port;
				messageParams[1] = *((__IO uint8_t*) (&RawMsgInt) + 3);
				messageParams[2] = *((__IO uint8_t*) (&RawMsgInt) + 2);
				messageParams[3] = *((__IO uint8_t*) (&RawMsgInt) + 1);
				messageParams[4] = *((__IO uint8_t*) (&RawMsgInt) + 0);
				SendMessageToModule(Module, CODE_PORT_FORWARD,
						sizeof(uint32_t) + 1);
			}

		} else if (H12R0_DATA_FORMAT == FMT_FLOAT) {
			if (Module == myID) {
				temp[0] = *((__IO uint8_t*) (&Raw_Msg) + 3);
				temp[1] = *((__IO uint8_t*) (&Raw_Msg) + 2);
				temp[2] = *((__IO uint8_t*) (&Raw_Msg) + 1);
				temp[3] = *((__IO uint8_t*) (&Raw_Msg) + 0);
				writePxMutex(Port, (char*) &temp, 4 * sizeof(uint8_t), 10, 10);
			} else {
				messageParams[0] = Port;
				messageParams[1] = *((__IO uint8_t*) (&Raw_Msg) + 3);
				messageParams[2] = *((__IO uint8_t*) (&Raw_Msg) + 2);
				messageParams[3] = *((__IO uint8_t*) (&Raw_Msg) + 1);
				messageParams[4] = *((__IO uint8_t*) (&Raw_Msg) + 0);
				SendMessageToModule(Module, CODE_PORT_FORWARD,
						sizeof(float) + 1);
			}
		}

		break;

	case SAMPLE_BUFFER_CASE:
	case STREAM_BUFFER_CASE:
		memset(Buffer, 0, sizeof(float));
		memcpy(Buffer, &Raw_Msg, sizeof(float));
		break;
	}
	if (Mode != STREAM_CLI_VERBOSE_CASE && Mode != STREAM_PORT_CASE) {
		free(strUnit);
	}
	return (H12R0_OK);
}

/*-----------------------------------------------------------*/

/* --- Voltage Read stream task
 */
void VoltageTask(void *argument) {

	HAL_ADC_Start_IT(&hadc);

	while (1) {

		uint32_t t0 = 0;

		DATA_To_SEND = CalculationVolt();

		switch (global_mode) {
		case STREAM_CLI_CASE:
			t0 = HAL_GetTick();
			SendResults(DATA_To_SEND, global_mode, 0, 0, 0, NULL);
			while (HAL_GetTick() - t0 < (global_period - 1)) {
				taskYIELD();
			}
			break;

		case STREAM_PORT_CASE:
			t0 = HAL_GetTick();
			SendResults(DATA_To_SEND, global_mode, 0, global_port,
								global_module, NULL);
			while (HAL_GetTick() - t0 < global_period) {
				taskYIELD();
			}
			break;

		case STREAM_BUFFER_CASE:
			t0 = HAL_GetTick();
			SendResults(DATA_To_SEND, global_mode, 0, 0, 0,ptr_volt_buffer);
			while (HAL_GetTick() - t0 < global_period) {
				taskYIELD();
			}
			break;

		default:
			global_mode = IDLE_CASE;
			break;
		}
		taskYIELD();
	}
}
/* -----------------------------------------------------------------------
 |							private function
 -----------------------------------------------------------------------
 */
float CalculationVolt(void)
{
	raw_adc = Volt_buffer[0];
	adc_volt = (raw_adc * 3.3) / 4095;
	measured_volt = adc_volt * 4;
	return measured_volt;
}


/*-----------------------------------------------------------*/
/* ---  Check for CLI stop key  --- */
static void CheckForEnterKey(void) {
	// Look for ENTER key to stop the stream
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
		if (UARTRxBuf[PcPort - 1][chr] == '\r') {
			UARTRxBuf[PcPort - 1][chr] = 0;
			startMeasurementRanging = STOP_MEASUREMENT_RANGING;
			global_mode = IDLE_CASE;		      // Stop the streaming task
			xTimerStop(xTimer, 0);            // Stop the timeout timer
			break;
		}
	}
}

/*-----------------------------------------------------------*/

/* --- software timer callback function
 */
static void HandleTimeout(TimerHandle_t xTimer) {
	uint32_t tid = 0;

	/* Get Timer ID */
	tid = (uint32_t) pvTimerGetTimerID(xTimer);
	if (TIMERID_TIMEOUT_MEASUREMENT == tid) {
		global_mode = IDLE_CASE;		      // Stop the streaming task
		startMeasurementRanging = STOP_MEASUREMENT_RANGING; // stop streaming
	}
}

/* -----------------------------------------------------------------------
 |																APIs	 																 	|
 -----------------------------------------------------------------------
 */

Module_Status SampleRead(float *volt)
{
	*volt = measured_volt;
	return (H12R0_OK);
}



Module_Status StreamVToPort(uint8_t Port, uint8_t Module, uint32_t Period,
		uint32_t Timeout) {
	global_port = Port;
	global_module = Module;
	global_period = Period;
	global_timeout = Timeout;
	global_mode = STREAM_PORT_CASE;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate("Timeout Measurement",
				pdMS_TO_TICKS(global_timeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer, portMAX_DELAY);
	}
	return (H12R0_OK);
}

Module_Status StreamVToBuffer(float *Buffer, uint32_t Period, uint32_t Timeout) {
	global_period = Period;
	global_timeout = Timeout;
	ptr_volt_buffer = Buffer;
	global_mode = STREAM_BUFFER_CASE;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate("Measurement Timeout",
				pdMS_TO_TICKS(global_timeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout);
		/* Start the timeout timer */
		xTimerStart(xTimer, portMAX_DELAY);
	}

	return (H12R0_OK);
}
/*------------------------------------------------*/
Module_Status Average(uint8_t samples, float * aveAns) {

	static float avr = 0, sum = 0, DATA_To_AVR;

	for (int i = 0; i < samples; i++)
	{
		DATA_To_AVR = CalculationVolt();
		sum += DATA_To_AVR;
	}
	*aveAns = sum / samples;
	return H12R0_OK;
}

/*-----------------------------------------------------------*/
Module_Status Stop(void) {
	global_mode = IDLE_CASE;

	xTimerStop(xTimer, 0);

	return H12R0_OK;
}

/* --- stream voltage read to CLI
*/
Module_Status StreamVToCLI(uint32_t Period, uint32_t Timeout)
{

	global_period=Period;
	global_timeout=Timeout;
	global_mode=STREAM_CLI_CASE;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	if (global_timeout > 0)
	{
		startMeasurementRanging = START_MEASUREMENT_RANGING;
	}

	return (H12R0_OK);
}

/* -----------------------------------------------------------------------
 |															Commands																 	|
 -----------------------------------------------------------------------
 */

static portBASE_TYPE unitCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	Module_Status result = H12R0_OK;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	static const int8_t *pcMessageWrongParam = (int8_t*) "Wrong parameter!\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* 1st parameter for naming of uart port: P1 to P6 */
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);

	if (!strncmp((const char*) pcParameterString1, "v", 1)) {
			strcpy((char*) pcWriteBuffer,
					(char*) "Used measurement unit: volt\r\n");
		} else {
		result = H12R0_ERR_WrongParams;
	}

	/* Respond to the command */
	if (H12R0_ERR_WrongParams == result) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageWrongParam);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/
static portBASE_TYPE sampleCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *pcMessageError = (int8_t*) "Wrong parameter\r\n";

	Module_Status result = H12R0_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	SampleRead(&DATA_To_SEND);

	global_mode = SAMPLE_CLI_CASE;
	SendResults(DATA_To_SEND, global_mode, 0, 0, 0, NULL);


	if (result != H12R0_OK)
		strcpy((char*) pcWriteBuffer, (char*) pcMessageError);
	/* clean terminal output */
	memset((char*) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE stopCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	volt_buffer = 0;
	Stop();

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/
static portBASE_TYPE streamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *pcMessageBuffer =
			(int8_t*) "Streaming measurements to internal buffer. Access in the CLI using module parameters: volt \n\r";
	static const int8_t *pcMessageModule =
			(int8_t*) "Streaming measurements to port P%d in module #%d\n\r";
	static const int8_t *pcMessageCLI =
			(int8_t*) "Streaming measurements to the CLI\n\n\r";
	static const int8_t *pcMessageError = (int8_t*) "Wrong parameter\r\n";
	static const int8_t *pcMessageWrongName = (int8_t*) "Wrong module name\r\n";
	int8_t *pcParameterString1; /* period */
	int8_t *pcParameterString2; /* timeout */
	int8_t *pcParameterString3; /* port or buffer */
	int8_t *pcParameterString4; /* module */
	portBASE_TYPE xParameterStringLength1 = 0;
	portBASE_TYPE xParameterStringLength2 = 0;
	portBASE_TYPE xParameterStringLength3 = 0;
	portBASE_TYPE xParameterStringLength4 = 0;
	uint32_t period = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	Module_Status result = H12R0_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the 1st parameter string: channel */
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	/* Obtain the 2nd parameter string: period */
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);
	/* Obtain the 3rd parameter string: timeout */
	pcParameterString3 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 3,
			&xParameterStringLength3);
	/* Obtain the 4th parameter string: port */
	pcParameterString4 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 4,
			&xParameterStringLength4);

	if (NULL != pcParameterString1) {
		period = atoi((char*) pcParameterString1);
	} else {
		result = H12R0_ERR_WrongParams;
	}

	if (NULL != pcParameterString2) {
		if (!strncmp((const char*) pcParameterString2, "inf", 3)) {
			timeout = portMAX_DELAY;
		} else {
			timeout = atoi((char*) pcParameterString2);
		}
	} else {
		result = H12R0_ERR_WrongParams;
	}
	/* streaming data to internal buffer (module parameter) */
	if (NULL != pcParameterString3
			&& !strncmp((const char*) pcParameterString3, "buffer", 6)) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageBuffer);
		StreamVToBuffer(&volt_buffer, period, timeout);
		// Return right away here as we don't want to block the CLI
		return pdFALSE;
	}
	/* streaming data to port */
	else if (NULL != pcParameterString3 && NULL != pcParameterString4
			&& pcParameterString3[0] == 'p') {
		port = (uint8_t) atol((char*) pcParameterString3 + 1);
		module = (uint8_t) GetID((char*) pcParameterString4);
		if (module != (uint8_t) BOS_ERR_WrongName) {
			sprintf((char*) pcWriteBuffer, (char*) pcMessageModule, port,
					module);
			StreamVToPort(port, module, period, timeout);
			// Return right away here as we don't want to block the CLI
			return pdFALSE;
		} else {
			strcpy((char*) pcWriteBuffer, (char*) pcMessageWrongName);
		}
	}

	/* Stream to the CLI */
		else if (NULL == pcParameterString3)
		{

				strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessageCLI);
				writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
				StreamVToCLI(period, timeout);
				/* Wait till the end of stream */
				while(startMeasurementRanging != STOP_MEASUREMENT_RANGING){taskYIELD();}
				/* clean terminal output */
				memset((char *) pcWriteBuffer, 0, strlen((char *)pcWriteBuffer));
		}

	else {
		result = H12R0_ERR_WrongParams;
	}

	if (H12R0_ERR_WrongParams == result) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageError);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;

}

/*-----------------------------------------------------------*/
portBASE_TYPE demoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	static const int8_t *pcMessage =
			(int8_t*) "Streaming voltage reading: \r\n";
	static const int8_t *pcMessageError = (int8_t*) "Wrong parameter\r\n";
	portBASE_TYPE xParameterStringLength1 = 0;
	Module_Status result = H12R0_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);




	/* Respond to the command */

		strcpy((char*) pcWriteBuffer, (char*) pcMessage);
		writePxMutex(PcPort, (char*) pcWriteBuffer,
				strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		StreamVToCLI(500, 10000);
		/* Wait till the end of stream */
		while (startMeasurementRanging != STOP_MEASUREMENT_RANGING) {
			Delay_ms(1);
		};


	if (result != H12R0_OK) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageError);
	}

	/* clean terminal output */
	memset((char*) pcWriteBuffer, 0, strlen((char*) pcWriteBuffer));

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-------------------------------------------*/


/*-------------------------------------------------------*/

