#include "motors_wrapper.h"
#include <stdlib.h>
#include <stdio.h>

static struct motors_wrapper mots = {NULL, NULL, 0, 0};

void motors_wrapper_init(struct ausbee_l298_chip *left_motor,
                         struct ausbee_l298_chip *right_motor)
{
	mots.left_motor  = left_motor;
	mots.right_motor = right_motor;

	mots.right_motor_moving_forward = 1;
	mots.left_motor_moving_forward  = 1;

	motors_wrapper_motor_set_duty_cycle(LEFT_MOTOR, 0);
	motors_wrapper_motor_set_duty_cycle(RIGHT_MOTOR, 0);
}

void motors_wrapper_motor_set_duty_cycle(enum motor id_motor, float duty_cycle)
{
	struct ausbee_l298_chip *motor = NULL;
	uint8_t* moving_forward = NULL;

	if (id_motor == LEFT_MOTOR) {
		motor = mots.left_motor;
		moving_forward = &mots.left_motor_moving_forward;
	}
	else if (id_motor == RIGHT_MOTOR) {
		motor = mots.right_motor;
		moving_forward = &mots.right_motor_moving_forward;
	}

  if (motor == NULL || moving_forward == NULL) {
    printf("[motors_wrapper] Error: Motors were not set properly.\n");
    return;
  }

  // Moving backward
  if (duty_cycle < 0) {
    *moving_forward = 0;
    ausbee_l298_invert_output(*motor, 1);
    ausbee_l298_set_duty_cycle(*motor, (uint32_t)(-duty_cycle));
  }
  // Moving forward
  else {
    *moving_forward = 1;
    ausbee_l298_invert_output(*motor, 0);
    ausbee_l298_set_duty_cycle(*motor, (uint32_t)duty_cycle);
  }
}

// Function that can be used in function pointer
void motors_wrapper_right_motor_set_duty_cycle(float duty_cycle)
{
  motors_wrapper_motor_set_duty_cycle(RIGHT_MOTOR, duty_cycle);
}

// Function that can be used in function pointer
void motors_wrapper_left_motor_set_duty_cycle(float duty_cycle)
{
  motors_wrapper_motor_set_duty_cycle(LEFT_MOTOR, duty_cycle);
}

uint8_t motors_wrapper_right_motor_is_moving_forward(void)
{
  return mots.right_motor_moving_forward;
}

uint8_t motors_wrapper_left_motor_is_moving_forward(void)
{
  return mots.left_motor_moving_forward;
}
