/**
 * @file    position_manager.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    12-Mar-2014
 * @brief   An odometry module. Definition file.
 */

#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <stdint.h>

void position_init(uint32_t ticks_per_m, double axle_track_mm);

void position_update();

void position_set_axle_track_mm(double axle_track_mm);
double position_get_axle_track_mm(void);

float position_get_left_encoder(void);
float position_get_right_encoder(void);

float position_get_distance_mm(void);
float position_get_angle_rad(void);
float position_get_angle_deg(void);
float position_get_x_mm(void);
float position_get_y_mm(void);

int32_t position_mm_to_ticks(float value_mm);

#endif /* POSITION_MANAGER_H */
