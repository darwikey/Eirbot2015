/**
 ********************************************************************
 * @file    encoder.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @author  Fabien DEPRAETRE
 * @version V1.0
 * @date    18-Mar-2014
 * @brief   Encoders driver implementation file.
 ********************************************************************
 * @attention
 *
 * This file is part of LIBAUSBEE.
 *
 * LIBAUSBEE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LIBAUSBEE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LIBAUSBEE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * <h2><centor>&copy;  Copyright 2013-2014 (C) EIRBOT </center></h2>
 ********************************************************************
 */
#include "AUSBEE/encoder.h"
#include "AUSBEE/device.h"

#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>

/**
  * @addtogroup Libausbee
  * @{
  */

/**
  * @defgroup Encoder
  * @brief Encoder driver module
  * @{
  */


void ausbee_init_sampling_timer(TIM_TypeDef *TIMX, int32_t prescaler, int32_t period)
{
  // TODO : Check if another timer can be used

  if (TIMX == TIM8)
  {
    TIM_TimeBaseInitTypeDef timeBaseInitTypeDef;

    TIM_TimeBaseStructInit(&timeBaseInitTypeDef);

    timeBaseInitTypeDef.TIM_Prescaler = prescaler;
    timeBaseInitTypeDef.TIM_Period = period;

    TIM_TimeBaseInit(TIM8, &timeBaseInitTypeDef);

    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM8, ENABLE);
  }
  else if (TIMX == TIM1)
  {
    TIM_TimeBaseInitTypeDef timeBaseInitTypeDef;

    TIM_TimeBaseStructInit(&timeBaseInitTypeDef);

    timeBaseInitTypeDef.TIM_Prescaler = prescaler;
    timeBaseInitTypeDef.TIM_Period = period;

    TIM_TimeBaseInit(TIM1, &timeBaseInitTypeDef);

    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
  }
}


void ausbee_encoder_init_timer(TIM_TypeDef* encoder_timer)
{
	// set them up as encoder inputs
	// set both inputs to rising polarity to let it use both edges
	TIM_EncoderInterfaceConfig (encoder_timer, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload (encoder_timer, 0xffff);

	// turn on the timer/counters
	TIM_Cmd (encoder_timer, ENABLE);

	// reset of the timer counter
	encoder_timer->CNT = 0;
}


// get the number of increment of an encoder since the last call of this function
inline int16_t ausbee_encoder_get_diff(TIM_TypeDef* encoder_timer)
{
	// value incremented by the encoder
	uint16_t counter = encoder_timer->CNT;
	int16_t diff = *(int16_t*)(&counter);
	// reset of the timer counter
	encoder_timer->CNT = 0;

	return diff;
}



/**
  * @}
  */
/**
  * @}
  */

/************** (C) COPYRIGHT 2013-2014 Eirbot **** END OF FILE ****/
