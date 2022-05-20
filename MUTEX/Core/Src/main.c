/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "Config.h"
#include "LCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	char Head[40];
	char temp[7];
}AMessgage;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/**
 * 0 : no se desea usar segger system view
 * 1 : se usa para la depuracion
 */
#define SYSTEM_VIEW				1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t task1_handle;
TaskHandle_t task2_handle;
TaskHandle_t task3_handle;
SemaphoreHandle_t xSemaphore;

AMessgage msg;
uint16_t data;					//adc read data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void task1(void *params);
void task2(void *params);
void task3(void *params);

void I2C_MasterSendData(I2C_TypeDef *I2Cx, uint8_t SlaveAddr,uint8_t *pTxbuffer, uint32_t Len);
void UART_Printf(char *format,...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	BaseType_t status;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
#if SYSTEM_VIEW
//  NVIC_SetPriorityGrouping(0);
  DWT->CTRL |= 1;
  SEGGER_SYSVIEW_Conf();
  /*solo se va usar cuando se trabaje en modo one shot*/
//  SEGGER_SYSVIEW_Start();

#endif

  status = xTaskCreate(task1,
		  	  	  	  "controlador de sensor externo",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  3,
					  &task1_handle);
  configASSERT(status == pdPASS);


  status = xTaskCreate(task2,
		  	  	  	  "tarea UART",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  3,
					  &task2_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(task3,
		  	  	  	  "controlador de sensor interno",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  3,
					  &task3_handle);
  configASSERT(status == pdPASS);

  /*creacion de mutex*/
  xSemaphore = xSemaphoreCreateMutex();
  configASSERT(xSemaphore != NULL);
  /* inicializa el kernel*/
  vTaskStartScheduler();
  /* USER CODE END 2 */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void task1(void *params){
	uint16_t adcValue;
	int temperature;
	float mV;
	sprintf(msg.Head,"controlador temperatura exteno");
	while(1){
		/*mutux take*/
		if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(1000))){

			/*lectura de valor de sensor*/
			HAL_ADC_Start(&hadc1);
			HAL_Delay(1);
			adcValue = HAL_ADC_GetValue(&hadc1);
			mV = adcValue * 3300.0 / 4096.0;			//voltaje en mv
			mV = mV / 10;								//temp en °C
			temperature = (int) mV;						//temperatura como entero
			sprintf(msg.temp,"%d",temperature);			//temperatura a char
			xSemaphoreGive(xSemaphore);
		}

		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}
void task2(void *params){

	while(1){

	}
}
void task3(void *params){

	while(1){

	}
}

/********************************************************************************************/

void I2C_MasterSendData(I2C_TypeDef *I2Cx, uint8_t SlaveAddr,uint8_t *pTxbuffer, uint32_t Len){

	volatile int tmp;

	/*verificar los datos*/
	if(Len <= 0 || pTxbuffer == ((void *)0))
		return;
	/*esperar que la linea estÃ© libre*/
	while((I2Cx->SR2 & I2C_SR2_BUSY));

	//1. generar la condiciÃ³n de start
	I2Cx->CR1 |= I2C_CR1_START;
	//2. esperar que la condiciÃ³n de inicio se haya generado
	while(!(I2Cx->SR1 & I2C_SR1_SB));

	//3. enviar la direccion del esclavo con r/w bit
	SlaveAddr = SlaveAddr <<1 ;
	SlaveAddr &=~(1U);						//write operation
	I2Cx->DR = SlaveAddr;
	//4. verificar que la fase de direccion se completÃ³
	while(!(I2Cx->SR1 & I2C_SR1_ADDR));
	//5. limpiar el flag
	tmp = I2Cx->SR1;
	tmp = I2Cx->SR2;
	(void)tmp;

	//6. enviar los datos
	while(Len > 0){

		while(!(I2Cx->SR1 & I2C_SR1_TXE));
		I2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	//7. esperar hasta que la transmision se complete
	while(!(I2Cx->SR1 & I2C_SR1_TXE));
	while(!(I2Cx->SR1 & I2C_SR1_BTF));

	//8. generar la condiciÃ³n de stop
	I2Cx->CR1 |= I2C_CR1_STOP;

	return;
}
/***************************************************************/
BaseType_t count;
void vApplicationIdleHook( void ){
	count++;
	UART_Printf("hook function->%d\r\n",count);

	//__WFI();		//modo de bajo consumo (despierta con una IT)
	//__WFE();		// despierta con un evento
	return;
}
/**
 * se va ejecutar cada que ocurra una ISR del tick
 */
void vApplicationTickHook( void ){


	return;
}


void UART_Printf(char *format,...){
    char str[80];

    /*Extract the the argument list using VA apis */
    va_list args;
    va_start(args, format);
    vsprintf(str, format,args);
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
    va_end(args);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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
