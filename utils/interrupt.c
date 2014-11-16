#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"

#include "platform.h"

#include <lidar.h>

#include "position_manager.h"
#include "motors_wrapper.h"

// Updating encoder value
void TIM8_UP_TIM13_IRQHandler(void) {

	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET) {


		//position_update();

		TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	}
}

void USART1_IRQHandler(void) {

	//verifie le flag d'interruption
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {

		platform_led_toggle(PLATFORM_LED6);

		//compulse the value received
		ausbee_lidar_push_char(USART_ReceiveData(USART1));

	}
}

