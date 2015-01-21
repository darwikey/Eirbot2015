/**********************************************************************
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
 * Copyright 2013-2014 (C) EIRBOT
 *
 * Authors :    Kevin JOLY <joly.kevin25@gmail.com>
 *
 **********************************************************************/
#ifndef LIBAUSBEE_LM18200_DRIVER_H
#define LIBAUSBEE_LM18200_DRIVER_H

#include <stm32f4xx_tim.h>

typedef struct {
	uint8_t timer_channel;
	uint16_t gpio_dir_pin;
	uint32_t pwm_frequency;
	GPIO_TypeDef* gpio_dir_port;
	TIM_TypeDef *TIMx;
} ausbee_lm18200_chip;

void ausbee_lm18200_init_chip(ausbee_lm18200_chip* chip);
void ausbee_lm18200_invert_output(ausbee_lm18200_chip* chip, uint8_t enable);
void ausbee_lm18200_set_duty_cycle(ausbee_lm18200_chip* chip, uint32_t duty_cycle);

#endif
