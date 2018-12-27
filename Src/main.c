
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "modbusSlave.h"
#include "stmFlash.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

long unsigned int UserBaudRate;
unsigned char UserSlaveAdd;
uint16_t tempArray[3];

uint8_t slaveAddTemp;
uint32_t baudRateTemp;


#define FLASH_SAVE_ADDR  0X0800EA60		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

unsigned char usart2_rx_buffer[128];
unsigned char usart2_tx_buffer[128];
unsigned int usart2_tx_len = 0;
unsigned char usart2_rx_flag = 0;

uint8_t r = 0;
uint8_t g = 0;
uint8_t b = 0;
uint8_t w = 0;

uint16_t pDMX_buf;
uint8_t DMX_buf[64];
uint16_t sum;
uint8_t low;
uint8_t high;

void DMX_SendPacket(void)
{
	pDMX_buf = 0;

	while (pDMX_buf <= 10) //1-512
	{
		/* send data packet to slaves*/
		if (USART1->SR & (1 << 6))
		{
			/*发送起始码 00*/
			if (0 == pDMX_buf)
			{
				USART1->DR = ((USART1->DR) & 0xfe00);//第九位置0
			}
			else
			{
				USART1->DR = 0x0100 | DMX_buf[pDMX_buf];//第九位置1
			}
			pDMX_buf++;
		}
	}
}


static void connectParameterInit()
{
	STMFLASH_Read(FLASH_SAVE_ADDR, tempArray, 3);
	
	if (tempArray[0] == 0xffff)
	{
		tempArray[0] = 1;
	}

	if ((tempArray[1] == 0xffff)&&(tempArray[2] == 0xffff))
	{
		tempArray[1] = 0;
		tempArray[2] = 9600;
	}

	UserBaudRate = (tempArray[1] << 16) + tempArray[2];
	UserSlaveAdd = (tempArray[0] & 0x00ff);
}

static void parameterProcessing()
{
	if (slaveAddTemp != localArray[100])
	{
		slaveAddTemp = localArray[100];
		UserSlaveAdd = localArray[100];
		STMFLASH_Write(FLASH_SAVE_ADDR, localArray+100, 1);
	}

	if (baudRateTemp != ((localArray[101] << 16) + localArray[102]))
	{
		baudRateTemp = ((localArray[101] << 16) + localArray[102]);
		UserBaudRate = ((localArray[101] << 16) + localArray[102]);
		STMFLASH_Write(FLASH_SAVE_ADDR+2, localArray + 101, 2);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  connectParameterInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)&usart2_rx_buffer, 128) != HAL_OK)    Error_Handler();
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  modbusSlaveScan();
	  parameterProcessing();

	  for (size_t i = 0; i < 64; i++)
	  {
		  DMX_buf[i] = localArray[i];
	  }
	  /*
	  if (usart2_rx_flag == 1)
	  {
		  
		  if ((usart2_tx_buffer[0] == 0xc8)&&(usart2_tx_buffer[1] == 0x01)&&(usart2_tx_buffer[2] == 0x01)  ) {

			  sum = usart2_tx_buffer[5] + usart2_tx_buffer[7] + usart2_tx_buffer[9] + usart2_tx_buffer[11] + 10;
			  low = sum & 0xff;
			  high = (sum & 0xff00) >> 8;
			  if ((usart2_tx_buffer[12] == low)&&(usart2_tx_buffer[13]==high))
			  {
				  HAL_GPIO_TogglePin(led_in_GPIO_Port, led_in_Pin);
				  r = usart2_tx_buffer[5];
				  g = usart2_tx_buffer[7];
				  b = usart2_tx_buffer[9];
				  w = usart2_tx_buffer[11];

				  DMX_buf[2] = r;
				  DMX_buf[3] = g;
				  DMX_buf[4] = b;
				  DMX_buf[5] = w;
			  }
		  }
		  usart2_rx_flag = 0;
	  }

	  */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		HAL_GPIO_TogglePin(led_out_GPIO_Port, led_out_Pin);
		DMX_SendPacket();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
