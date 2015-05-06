#include "motors_wrapper.h"
#include <platform.h>
#include <stdlib.h>
#include <stdio.h>

enum motor_chip {
	NO_CHIP, L298_CHIP, LM18200_CHIP
};

struct motors_wrapper {
	enum motor_chip motor_chip;
	void* right_motor;
	void* left_motor;
	uint8_t right_motor_moving_forward;
	uint8_t left_motor_moving_forward;
};

static struct motors_wrapper mots = {NO_CHIP, NULL, NULL, 0, 0};

void motors_wrapper_init_l298(struct ausbee_l298_chip *left_motor,
                         struct ausbee_l298_chip *right_motor)
{
	mots.motor_chip = L298_CHIP;
	mots.left_motor  = left_motor;
	mots.right_motor = right_motor;

	mots.right_motor_moving_forward = 1;
	mots.left_motor_moving_forward  = 1;

	motors_wrapper_motor_set_duty_cycle(LEFT_MOTOR, 0);
	motors_wrapper_motor_set_duty_cycle(RIGHT_MOTOR, 0);
}

void motors_wrapper_init_lm18200(ausbee_lm18200_chip* left_motor, ausbee_lm18200_chip* right_motor)
{
	mots.motor_chip = LM18200_CHIP;
	mots.left_motor  = left_motor;
	mots.right_motor = right_motor;

	mots.right_motor_moving_forward = 1;
	mots.left_motor_moving_forward  = 1;

	motors_wrapper_motor_set_duty_cycle(LEFT_MOTOR, 0);
	motors_wrapper_motor_set_duty_cycle(RIGHT_MOTOR, 0);
}

void motors_wrapper_motor_set_duty_cycle(enum motor id_motor, float duty_cycle)
{
	void* motor = NULL;
	uint8_t* moving_forward = NULL;

	/*if (duty_cycle > 30)
	{
		duty_cycle = 30;
	}
	else if(duty_cycle < -30)
	{
		duty_cycle = -30;
	}*/

	if (id_motor == LEFT_MOTOR) {
		motor = mots.left_motor;
		moving_forward = &mots.left_motor_moving_forward;
	}
	else if (id_motor == RIGHT_MOTOR) {
		motor = mots.right_motor;
		moving_forward = &mots.right_motor_moving_forward;
	}

  if (motor == NULL || moving_forward == NULL || mots.motor_chip == NO_CHIP) {
    printf("[motors_wrapper] Error: Motors were not set properly.\n");
    return;
  }

  // Moving backward
  if (duty_cycle < 0) {
	*moving_forward = 0;

	if (mots.motor_chip == L298_CHIP) {
		ausbee_l298_invert_output(motor, 1);
		ausbee_l298_set_duty_cycle(motor, (uint32_t)(-duty_cycle));
	}
	else if (mots.motor_chip == LM18200_CHIP) {
		ausbee_lm18200_invert_output(motor, 1);
		ausbee_lm18200_set_duty_cycle(motor, (uint32_t)(-duty_cycle));
	}
  }
  // Moving forward
  else {
    *moving_forward = 1;

    if (mots.motor_chip == L298_CHIP) {
    	ausbee_l298_invert_output(motor, 0);
    	ausbee_l298_set_duty_cycle(motor, (uint32_t)duty_cycle);
    }
    else if (mots.motor_chip == LM18200_CHIP) {
    	ausbee_lm18200_invert_output(motor, 0);
    	ausbee_lm18200_set_duty_cycle(motor, (uint32_t)duty_cycle);
    }
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

// power the motor in order to test them
// sequence : left motor forward then backward
//            right motor forward then backward
// nether return
void motors_wrapper_test(void)
{
	for (int i = 0; i < 50; ++i)
	{
		motors_wrapper_left_motor_set_duty_cycle(i);
		platform_led_toggle(PLATFORM_LED0);
		for (volatile int t = 0; t < 1000000; t++); //delay
	}
	for (int i = 0; i < 50; ++i)
	{
		motors_wrapper_left_motor_set_duty_cycle(-i);
		platform_led_toggle(PLATFORM_LED1);
		for (volatile int t = 0; t < 1000000; t++);
	}
	for (int i = 0; i < 50; ++i)
	{
		motors_wrapper_right_motor_set_duty_cycle(i);
		platform_led_toggle(PLATFORM_LED2);
		for (volatile int t = 0; t < 1000000; t++);
	}
	for (int i = 0; i < 50; ++i)
	{
		motors_wrapper_right_motor_set_duty_cycle(-i);
		platform_led_toggle(PLATFORM_LED3);
		for (volatile int t = 0; t < 1000000; t++);
	}
	while(1);
}
