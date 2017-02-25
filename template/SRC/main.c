/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
#include "light_ws2812_cortex.h"
#include "stm32f4xx.h"
#include "./Bsp/usart/bsp_debug_usart.h"
#include "bsp_unique_id.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	void delay_ms(uint32_t delay)
{
	for(int i=0; i<delay; i++)
	{
		for(int j=0; j<168067; j++)
		{}
	}
}


void CO_TIMConfigTest(void)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		/* TIM3 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		uint16_t PrescalerValue = 83;			
		TIM_TimeBaseStructure.TIM_Period = 999;//1ms
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
		NVIC_InitTypeDef NVIC_InitStructure;
		/* Enable the TIM3 Update Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

 
		 // Clear all pending interrupts
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM3, ENABLE);  
}

int main(void)
{
 
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files before to branch to application main.
       To reconfigure the default setting of SystemInit() function, 
       refer to system_stm32f4xx.c file */
		uint32_t id[3] ={0};
		Debug_USART_Config();
		printf("Hello, the world!\n");

		get_unique_id(id);
		//eth_test();


		delay_ms(100);
		ws2812_config_gpio();
		//CO_TIMConfigTest();
		uint8_t rgb[31*3] = {
											0x33, 0x33, 0x33, 
											0x55, 0x55, 0x55, 
											0xaa, 0xaa, 0xaa, 
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
		
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
											
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
		
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF,
		
											0xFF, 0x0, 0x0,
											0x0, 0xFF, 0x0,
											0x0, 0x0, 0xFF

											};
	//ws2812_sendarray_armcc(rgb, 30*3);
	while(1)
	{
		for(int i = 0; i < 255; i++)
		{
				rgb[9] = i;
				ws2812_sendarray_armcc(rgb, 9*3);
				delay_ms(3);
		}
		for(int i = 255; i > 0; i--)
		{
				rgb[9] = i;
				ws2812_sendarray_armcc(rgb, 9*3);
				delay_ms(3);
		}
  }

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
//void TimingDelay_Decrement(void)
//{
//  if (uwTimingDelay != 0x00)
//  { 
//    uwTimingDelay--;
//  }
//}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
