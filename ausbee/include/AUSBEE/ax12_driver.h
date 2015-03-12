/**
 ********************************************************************
 * @file    ax12.h
 * @author  Vincent CESSON <cesson.vincent@gmail.com>
 * @version V1.0
 * @date    20-Feb-2014
 * @brief   This file contains all the functions prototype  and 
 *			 definesfor the Ax12 library.
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

/* Define to prevent recursive inclusion */
#ifndef _AX12_H
#define _AX12_H

/* Includes */
#include <stdint.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_rcc.h>
/**
 * @addtogroup Libausbee
 * @{
 */

/**
 * @addtogroup AX12
 * @{
 */

typedef struct {
	USART_TypeDef* usart;
	int id;
} ausbee_ax12_chip;


/** Set the mode of the servo
 * @param mode
 *    0 = Positional, default
 *    1 = Continuous rotation
 */
int ausbee_ax12_set_mode(ausbee_ax12_chip* chip, int mode);

/** Set baud rate of all attached servos
 * @param mode
 *    0x01 = 1,000,000 bps
 *    0x03 =   500,000 bps
 *    0x04 =   400,000 bps
 *    0x07 =   250,000 bps
 *    0x09 =   200,000 bps
 *    0x10 =   115,200 bps
 *    0x22 =    57,600 bps
 *    0x67 =    19,200 bps
 *    0xCF =     9,600 bp
 */
int ausbee_ax12_set_baud(USART_TypeDef* usart, int baud);


/** Set goal angle in integer degrees, in positional mode
 *
 * @param degrees 0-300
 * @param flags, defaults to 0
 *    flags[0] = blocking, return when goal position reached
 *    flags[1] = register, activate with a broadcast trigger
 *
 */
int ausbee_ax12_set_goal(ausbee_ax12_chip* chip, int degrees, int flags);


/** Set the speed of the servo in continuous rotation mode
 *
 * @param speed, -1.0 to 1.0
 *   -1.0 = full speed counter clock wise
 *    1.0 = full speed clock wise
 */
int ausbee_ax12_set_CR_speed(ausbee_ax12_chip* chip, float speed);


/** Set the clockwise limit of the servo
 *
 * @param degrees, 0-300
 */
int ausbee_ax12_set_CW_limit(ausbee_ax12_chip* chip, int degrees);

/** Set the counter-clockwise limit of the servo
 *
 * @param degrees, 0-300
 */
int ausbee_ax12_set_CCW_limit(ausbee_ax12_chip* chip, int degrees);

// Change the ID

/** Change the ID of a servo
 *
 * @param CurentID 1-255
 * @param NewID 1-255
 *
 * If a servo ID is not know, the broadcast address of 0 can be used for CurrentID.
 * In this situation, only one servo should be connected to the bus
 */
int ausbee_ax12_set_ID(ausbee_ax12_chip* chip, int NewID);


/** Poll to see if the servo is moving
 *
 * @returns true is the servo is moving
 */
int ausbee_ax12_is_moving(ausbee_ax12_chip* chip);

/** Send the broadcast "trigger" command, to activate any outstanding registered commands
 */
void ausbee_ax12_trigger(ausbee_ax12_chip* chip);

/** Read the current angle of the servo
 *
 * @returns float in the range 0.0-300.0
 */
float ausbee_ax12_get_position(ausbee_ax12_chip* chip);

/** Read the temperature of the servo
 *
 * @returns float temperature
 */
float ausbee_ax12_get_temp(ausbee_ax12_chip* chip);

/** Read the supply voltage of the servo
 *
 * @returns float voltage
 */
float ausbee_ax12_get_volts(ausbee_ax12_chip* chip);


/** Led
 * 1 to light the led, 0 to turn off
 */
void ausbee_ax12_set_led(ausbee_ax12_chip* chip, int fState);


#endif /* _AX12_H */

/**
 * @}
 */

/**
 * @}
 */

/**************** (C) COPYRIGHT 2013-2014 Eirbot **** END OF FILE ****/
