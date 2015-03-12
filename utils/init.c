#include <stdio.h>
#include <stdlib.h>

#include "init.h"
#include "misc.h"
/*#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "actions.h"*/

//#include "define.h"

#include <AUSBEE/encoder.h>

//extern xSemaphoreHandle USART1ReceiveHandle;

void init_usart1_interrupt()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void init_can_rx_interrupt()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void init_can()
{
  platform_can_init(CAN1);

  CAN_InitTypeDef CAN_InitTypeDef_1;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  CAN_StructInit(&CAN_InitTypeDef_1);
  CAN_InitTypeDef_1.CAN_Prescaler = 336;
  CAN_InitTypeDef_1.CAN_Mode = CAN_Mode_Normal;
  CAN_InitTypeDef_1.CAN_AWUM = ENABLE;
  CAN_InitTypeDef_1.CAN_TXFP = ENABLE;
  if(CAN_Init(CAN1, &CAN_InitTypeDef_1) == CAN_InitStatus_Failed) {
    platform_led_set(PLATFORM_LED6);
    while(1);
  }

  // CAN filter init

  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0001;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0001;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0001;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0001;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
}

void init_encoders(void)
{
	// Timer utilisé pour l'échantillonage
	platform_enable_clock_timer(TIM8);
	ausbee_init_sampling_timer(TIM8, 16800, 1000);
	platform_encoder_init();
}

void init_lidar()
{
  platform_usart_init(USART1,115200);
  init_usart1_interrupt();


  /*USART1ReceiveHandle=xSemaphoreCreateMutex();
  if(USART1ReceiveHandle== NULL)
  {
    //platform_led_toggle(PLATFORM_LED6);
  }*/
}


//Fonction pour initialiser les gpio en fonction du robot
void init_gpio_robot()
{
}
