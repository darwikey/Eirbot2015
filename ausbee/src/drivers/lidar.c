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
 * Based on informations of http://xv11hacking.wikispaces.com
 *
 **********************************************************************/

#include <AUSBEE/lidar.h>
#include "stdio.h"

static uint8_t frame[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
volatile static struct ausbee_lidar_data data[AUSBEE_LIDAR_PICCOLO_DATA_LENGTH];
static struct ausbee_lidar_distance distance = {0};


// fonction privée
void ausbee_lidar_parse_piccolo();

// rajoute une donnée dans le buffer
void ausbee_lidar_push_char(unsigned char rec) {

	// variable to know if the frame is at the beginning
	static uint8_t not_started = 1;

	//Index of the buffer
	static uint8_t index_buffer = 0;

	//if the beginning of the frame has not been reached
	if (not_started == 1) {
		//check if the frame is at the beginning
		if (rec == 0xFA) {
			not_started = 0;
			//add the element to buffer and increment the buffer
			frame[0] = (unsigned char) rec;
			index_buffer++;
		}
	} else {
		//add the element
		frame[index_buffer] = (unsigned char) rec;
		index_buffer++;
	}
	//reset variables
	if (index_buffer == AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH) {
		// parse the frame
		ausbee_lidar_parse_piccolo();

		//TODO xSemaphoreGive(USART1ReceiveHandle);
		index_buffer = 0;
		not_started = 1;
	}

}

// rempli le tableau data d'après le buffer frame
void ausbee_lidar_parse_piccolo() {

	distance.lock = 1;

	for (int i = 0; i < AUSBEE_LIDAR_PICCOLO_DATA_LENGTH; i++) {
		data[i].angle = (frame[1] - 0xA0) * 4 + i; // Get angle for each data structure in degree
		data[i].speed = ((frame[2] << 8) + frame[3]) >> 6; // Get actual rotation speed of the LIDAR
		data[i].distance_mm =
				frame[4 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH]
						+ ((frame[5 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH]
								& 0x3F) << 8); // Get the measured distance for each data structure
		data[i].signal_strength =
				frame[6 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH]
						+ (frame[7 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH] << 8); // Get signal strength

		// Check if no error occurs during measure
		if (frame[5 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH] & 0x80) {
			data[i].error = 1;
			data[i].error_code =
					frame[4 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH];
		} else {
			data[i].error = 0;
			data[i].error_code = 0x0;
		}

		// Fill the strength warning flag in the data structure
		if (frame[5 + i * AUSBEE_LIDAR_PICCOLO_DATA_LENGTH] & 0x40)
			data[i].strengthWarning = 1;
		else
			data[i].strengthWarning = 0;

		int index = data[i].angle;
		if (index >= 0 && index < 360) {
			distance.table[index] = data[i].distance_mm;
		}
	}

	distance.lock = 0;
}

// get the distance covered by the laser rays in mm
// angle is in degree (0 to 360°)
int ausbee_lidar_get_distance(uint16_t angle)
{
	if (!distance.lock && angle < 360)
	{
		return distance.table[angle];
	}
	else
	{
		printf("echec lidar get distance\n\r");
		return -1;
	}
}

