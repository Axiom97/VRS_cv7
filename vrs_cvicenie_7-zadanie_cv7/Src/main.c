/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Function processing DMA Rx data. Counts how many capital and small letters are in sentence.
 * Result is supposed to be stored in global variable of type "letter_count_" that is defined in "main.h"
 *
 * @param1 - received sign
 */
void proccesDmaData(const uint8_t* data, uint16_t len);


/* Space for your global variables. */
static uint8_t start_flag=0;
uint16_t letter_count_small=0, letter_count_capital=0;
uint8_t index_buffera=0;

uint8_t buffer[35];
uint8_t message[]= "Buffer capacity: x bytes, occupied memory: x bytes, load [in %]: x%";
double percento=0;



char bufffer_1[4];
char bufffer_2[4];
char bufffer_3[6];



	// type your global variables here:

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /* Space for your local variables, callback registration ...*/

  USART2_RegisterCallback(proccesDmaData);


  while (1){
	  /* Periodic transmission of information about DMA Rx buffer state.
	   * Transmission frequency - 5Hz.
	   * Message format - "Buffer capacity: %d bytes, occupied memory: %d bytes, load [in %]: %f%"
	   * Example message (what I wish to see in terminal) - Buffer capacity: 1000 bytes, occupied memory: 231 bytes, load [in %]: 23.1%
	   */
	  LL_mDelay(200);

	  for(uint8_t index=0; index<=67; index++){
		  if(index==17){
			  itoa(DMA_USART2_BUFFER_SIZE, bufffer_1, 10);
			  for(uint8_t i=0; i<3; i++){
				  LL_USART_TransmitData8(USART2, bufffer_1[i]);
				  LL_mDelay(1);
			  }
		  }
		  else if(index==43){
			  itoa(DMA_USART2_BUFFER_SIZE-LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6), bufffer_2, 10);
			  for(uint8_t i=0; i<3; i++){
				  LL_USART_TransmitData8(USART2, bufffer_2[i]);
				  LL_mDelay(1);
			  }
			  memset(bufffer_2,'\000',4);
		  }
		  else if(index==65){
			  percento=(double)(DMA_USART2_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6));
			  percento=(100*percento)/DMA_USART2_BUFFER_SIZE;
			  sprintf(bufffer_3, "%f" ,percento);
			  for(uint8_t i=0; i<sizeof(bufffer_3); i++){
				  LL_USART_TransmitData8(USART2, bufffer_3[i]);
				  LL_mDelay(1);
			  }
			  memset(bufffer_3,'\000',6);
		  }
		  else{
			  LL_USART_TransmitData8(USART2, message[index]);
		  }
		  LL_mDelay(1);
	  }
  }
}


void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}

/*
 * Implementation of function processing data received via USART.
 */
void proccesDmaData(const uint8_t* data, uint16_t len)
{

	for(uint8_t i = 0; i<len; i++){
		if (*(data+i) == '#'){
			start_flag=1;
		}
		if(start_flag){
			buffer[index_buffera++]=*(data+i);
			if (*(data+i) == '$'){
				start_flag=0;
				index_buffera=0;
				for(uint8_t j = 0; j<sizeof(buffer); j++){
					if((buffer[j] > 96) && (buffer[j] < 123)){
						letter_count_small++;
					}
					if((buffer[j] > 64) && (buffer[j]<91)){
						letter_count_capital++;
					}
				}
				memset(buffer,'\000',sizeof(buffer));
			}

			if(index_buffera>34){
				index_buffera=0;
				start_flag=0;
				memset(buffer,'\000',sizeof(buffer));
			}

		}
	}
}


void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{ 

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
