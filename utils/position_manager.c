/**
 * @file    position_manager.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    12-Mar-2014
 * @brief   An odometry module. Implementation file.
 */

#include "position_manager.h"
#include "encoder.h"
#include "platform.h"
#include <math.h>
#include <stdio.h>

#define PI 3.1415926535

struct position_manager {
	uint32_t ticks_per_m;
	double axle_track_mm; // ecart en mm entre les deux encodeurs

	int32_t left_encoder, right_encoder;

	double distance_mm;
	double angle_rad;
	double x_mm, y_mm;
} pm;

#define position_ticks_to_mm(value_ticks) ((value_ticks) * 1000.0 / pm.ticks_per_m)

void position_init(uint32_t ticks_per_m, double axle_track_mm) {
	pm.ticks_per_m = ticks_per_m;
	pm.axle_track_mm = axle_track_mm;

	pm.left_encoder = 0;
	pm.right_encoder = 0;

	pm.distance_mm = 0;
	pm.angle_rad = 0;

	pm.x_mm = 0;
	pm.y_mm = 0;
}

void position_update()
{
	// Reading encoder value
	int16_t left_enc_diff = ausbee_encoder_get_diff(PLATFORM_ENC1_TIMER);
	int16_t right_enc_diff = ausbee_encoder_get_diff(PLATFORM_ENC2_TIMER);

	pm.left_encoder += left_enc_diff;
	pm.right_encoder += right_enc_diff;

	if (left_enc_diff != 0) {
		platform_led_toggle(PLATFORM_LED2);
		//printf("encG : %d   (sumG : %d)\n\r", (int)left_enc_diff, (int)pm.left_encoder);
	}

	if (right_enc_diff != 0) {
		platform_led_toggle(PLATFORM_LED3);
		//printf("encD : %d   (sumD : %d)\n\r", (int)right_enc_diff, (int)pm.right_encoder);
	}


	// Distance
	double distance_diff_ticks = (left_enc_diff + right_enc_diff) / 2.0;
	double distance_diff_mm = position_ticks_to_mm(distance_diff_ticks);
	pm.distance_mm += distance_diff_mm;

	// Special case: no rotation
	if ((right_enc_diff - left_enc_diff) == 0) {
		pm.x_mm += -distance_diff_mm * sin(pm.angle_rad);
		pm.y_mm += distance_diff_mm * cos(pm.angle_rad);
		return;
	}

	// Angle TODO atan2 ?
	double angle_diff_rad = atan(
			position_ticks_to_mm(right_enc_diff - left_enc_diff)
					/ pm.axle_track_mm);

	// Special case: only rotation -> no need to update x and y
	if ((right_enc_diff + left_enc_diff) == 0) {
		pm.angle_rad += angle_diff_rad;
		return;
	}

	// Radius of curvature
	double r = pm.axle_track_mm / 2.0 * (right_enc_diff + left_enc_diff)
			/ (right_enc_diff - left_enc_diff);

	// Trajectory circle center coordinates
	double x0_mm = pm.x_mm - r * cos(pm.angle_rad);
	double y0_mm = pm.y_mm - r * sin(pm.angle_rad);

	// Update position
	pm.angle_rad += angle_diff_rad;
	pm.x_mm = x0_mm + r * cos(pm.angle_rad);
	pm.y_mm = y0_mm + r * sin(pm.angle_rad);
}

uint32_t position_get_axle_track_mm(void) {
	return pm.axle_track_mm;
}

float position_get_left_encoder(void) {
	return pm.left_encoder;
}

float position_get_right_encoder(void) {
	return pm.right_encoder;
}

float position_get_distance_mm(void) {
	return pm.distance_mm;
}

float position_get_angle_rad(void) {
	return pm.angle_rad;
}

float position_get_angle_deg(void) {
	return pm.angle_rad * 180.0 / PI;
}

float position_get_x_mm(void) {
	return pm.x_mm;
}

float position_get_y_mm(void) {
	return pm.y_mm;
}

int32_t position_mm_to_ticks(float value_mm) {
	return value_mm * pm.ticks_per_m / 1000.0;
}
