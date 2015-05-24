/**
 * @file    control_system.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.2
 * @date    18-Mar-2014
 * @brief   A controller system for a two-wheeled robot with encoders.
 *          Implementation file.
 */

#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"

#include <AUSBEE/pid.h>

#include "position_manager.h"
#include "smooth_traj_manager.h"
#include "motors_wrapper.h"
#include "control_system.h"
#include "platform.h"

#define PI 3.1415926535
#define DEG2RAD(a) ((a) * PI / 180.0)

// Global variable
struct control_system am;

// Private functions
void control_system_task(void *data);
static void control_system_set_motors_ref(float d_mm, float theta);
static void control_system_set_distance_mm_diff(float ref);
static void control_system_set_angle_rad_diff(float ref);


static void control_system_init_distance_angle()
{
  ausbee_pid_init(&(am.pid_distance), 0.04, 0.0005, 0.0);
  ausbee_pid_init(&(am.pid_angle),  0.02, 0.0004, 0.0);

  ausbee_pid_set_output_range(&(am.pid_distance), -100, 100);
  ausbee_pid_set_output_range(&(am.pid_angle),  -100, 100);

  // Quadramp setup
  ausbee_quadramp_init(&(am.quadramp_distance));
  ausbee_quadramp_init(&(am.quadramp_angle));

  // Setting quadramp eval period to the control system period
  ausbee_quadramp_set_eval_period(&(am.quadramp_distance), CONTROL_SYSTEM_PERIOD_S);
  ausbee_quadramp_set_eval_period(&(am.quadramp_angle),    CONTROL_SYSTEM_PERIOD_S);

  ausbee_quadramp_set_2nd_order_vars(&(am.quadramp_distance),
                                     DISTANCE_MAX_ACC,
                                     DISTANCE_MAX_ACC); // Translation acceleration (in mm/s^2)

  ausbee_quadramp_set_2nd_order_vars(&(am.quadramp_angle),
                                     DEG2RAD(ANGLE_MAX_ACC_DEG),
                                     DEG2RAD(ANGLE_MAX_ACC_DEG)); // Rotation acceleration (in rad/s^2)

  control_system_set_speed_high();

  // Initialise each control system manager
  ausbee_cs_init(&(am.csm_distance));
  ausbee_cs_init(&(am.csm_angle));

  // Set reference filter
  ausbee_cs_set_reference_filter(&(am.csm_distance), ausbee_quadramp_eval, (void*)&(am.quadramp_distance));
  ausbee_cs_set_reference_filter(&(am.csm_angle),    ausbee_quadramp_eval, (void*)&(am.quadramp_angle));

  // Set measure functions
  ausbee_cs_set_measure_fetcher(&(am.csm_distance), position_get_distance_mm);
  ausbee_cs_set_measure_fetcher(&(am.csm_angle), position_get_angle_rad);

  // We use a pid controller because we like it here at Eirbot
  ausbee_cs_set_controller(&(am.csm_distance), ausbee_pid_eval, (void*)&(am.pid_distance));
  ausbee_cs_set_controller(&(am.csm_angle), ausbee_pid_eval, (void*)&(am.pid_angle));

  // Set processing command
  ausbee_cs_set_process_command(&(am.csm_distance), control_system_set_distance_mm_diff);
  ausbee_cs_set_process_command(&(am.csm_angle), control_system_set_angle_rad_diff);

  control_system_set_distance_mm_diff(0);
  control_system_set_angle_rad_diff(0);
}

void control_system_start()
{
  //control_system_init_motors(am);
  control_system_init_distance_angle();

  xTaskCreate(control_system_task, (const signed char *)"ControlSystem", 250, NULL, 1, NULL);
}

void control_system_task(void *data)
{
	(void)data;

	for (;;)
	{

		position_update();

		if (smooth_traj_is_paused())
		{
			// don't move
			motors_wrapper_motor_set_duty_cycle(RIGHT_MOTOR, 0);
			motors_wrapper_motor_set_duty_cycle(LEFT_MOTOR, 0);
		}
		else
		{
			platform_led_toggle(PLATFORM_LED1);

			ausbee_cs_manage(&(am.csm_distance));
			ausbee_cs_manage(&(am.csm_angle));

			control_system_set_motors_ref(am.distance_mm_diff, am.angle_rad_diff);
		}
		//ausbee_cs_manage(&(am.csm_right_motor));
		//ausbee_cs_manage(&(am.csm_left_motor));

		vTaskDelay(CONTROL_SYSTEM_PERIOD_S * 1000 / portTICK_RATE_MS);
	}
}

static void control_system_set_motors_ref(float d_mm, float theta)
{
  uint32_t axle_track_mm = position_get_axle_track_mm();

  d_mm = -d_mm;

  int32_t right_motor_ref = position_mm_to_ticks(d_mm + (1.0 * axle_track_mm * theta) / 2);
  int32_t left_motor_ref  = position_mm_to_ticks(d_mm - (1.0 * axle_track_mm * theta) / 2);

  //printf("cmd right %d   left %d\r\n", (int)right_motor_ref, (int)left_motor_ref);

  motors_wrapper_motor_set_duty_cycle(RIGHT_MOTOR, right_motor_ref);
  motors_wrapper_motor_set_duty_cycle(LEFT_MOTOR, left_motor_ref);

  //ausbee_cs_set_reference(&(am.csm_right_motor), right_motor_ref);
  //ausbee_cs_set_reference(&(am.csm_left_motor), left_motor_ref);
}

static void control_system_set_distance_mm_diff(float ref)
{
  am.distance_mm_diff = ref;
}

static void control_system_set_angle_rad_diff(float ref)
{
  am.angle_rad_diff = ref;
}

// User functions
void control_system_set_distance_mm_ref(float ref)
{
  ausbee_cs_set_reference(&(am.csm_distance), ref);
}

void control_system_set_angle_deg_ref(float ref_deg)
{
  float ref_rad = DEG2RAD(ref_deg);
  ausbee_cs_set_reference(&(am.csm_angle), ref_rad);
}

void control_system_set_angle_rad_ref(float ref_rad)
{
  ausbee_cs_set_reference(&(am.csm_angle), ref_rad);
}

void control_system_set_right_motor_ref(int32_t ref)
{
  ausbee_cs_set_reference(&(am.csm_right_motor), ref);
}

void control_system_set_left_motor_ref(int32_t ref)
{
  ausbee_cs_set_reference(&(am.csm_left_motor), ref);
}

void control_system_set_distance_max_speed(float max_speed)
{
  ausbee_quadramp_set_1st_order_vars(&(am.quadramp_distance), max_speed, max_speed);
}

void control_system_set_distance_max_acc(float max_acc)
{
  ausbee_quadramp_set_2nd_order_vars(&(am.quadramp_distance), max_acc, max_acc);
}

void control_system_set_angle_max_speed(float max_speed)
{
  ausbee_quadramp_set_1st_order_vars(&(am.quadramp_angle), DEG2RAD(max_speed), DEG2RAD(max_speed));
}

void control_system_set_angle_max_acc(float max_acc)
{
  ausbee_quadramp_set_2nd_order_vars(&(am.quadramp_angle), DEG2RAD(max_acc), DEG2RAD(max_acc));
}

void control_system_set_speed_ratio(float ratio)
{
  if (ratio < 0) {
    ratio = 0;
  }
  else if (ratio > 1) {
    ratio = 1;
  }

  control_system_set_distance_max_speed(ratio*DISTANCE_MAX_SPEED); // Translation speed (in mm/s)

  control_system_set_angle_max_speed(ratio*ANGLE_MAX_SPEED_DEG); // Rotation speed (in rad/s)
}

void control_system_set_speed_high()
{
  control_system_set_speed_ratio(1);
}

void control_system_set_speed_medium()
{
  control_system_set_speed_ratio(0.7);
}

void control_system_set_speed_low()
{
  control_system_set_speed_ratio(0.5);
}

struct ausbee_pid* control_system_get_pid_distance()
{
	return &am.pid_distance;
}

struct ausbee_pid* control_system_get_pid_angle()
{
	return &am.pid_angle;
}

//TODO prendre en compte les angles != 0
void control_system_reset_angle()
{
	am.quadramp_angle.prev_in = 0.f;
	am.quadramp_angle.prev_out = 0.f;
	am.quadramp_angle.prev_var = 0.f;
}
