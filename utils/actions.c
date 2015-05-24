#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "platform.h"
#include "actions.h"
#include "servo.h"
#include "lm18200_driver.h"


ausbeeServo servo1;
ausbeeServo servo_grip;
ausbeeServo servo_clapet;
ausbeeServo servo4;
struct ausbee_l298_chip motor_lift;
struct ausbee_l298_chip lidar_voltage;


void init_actions(void)
{

	ausbeeInitStructServo(&servo1);
	ausbeeInitStructServo(&servo_grip);
	ausbeeInitStructServo(&servo_clapet);
	ausbeeInitStructServo(&servo4);

	servo1.TIMx = SERVO1_TIM;
	servo1.CHANx = SERVO1_CHAN;
	servo_grip.TIMx = SERVO2_TIM;
	servo_grip.CHANx = SERVO2_CHAN;
	servo_clapet.TIMx = SERVO3_TIM;
	servo_clapet.CHANx = SERVO3_CHAN;
	servo4.TIMx = SERVO4_TIM;
	servo4.CHANx = SERVO4_CHAN;

	ausbeeInitServo(&servo1);
	ausbeeInitServo(&servo_grip);
	ausbeeInitServo(&servo_clapet);
	ausbeeInitServo(&servo4);

	ausbeeSetAngleServo(&servo_clapet, 90);
	ausbeeSetAngleServo(&servo1, 50);
	ausbeeSetAngleServo(&servo_grip, 50);
	ausbeeSetAngleServo(&servo4, 90);

	platform_ausbee_motor1_init(&lidar_voltage);
	platform_ausbee_motor2_init(&motor_lift);

	ausbee_l298_set_duty_cycle(&lidar_voltage, 70); //3V et quelques
}


void action_open_grip(void)
{
	ausbeeSetAngleServo(&servo_grip, 80);
}

void action_open_clasp(void)
{
	ausbeeSetAngleServo(&servo_clapet, 90);
}

void action_close_clasp(void)
{
	ausbeeSetAngleServo(&servo_clapet, 0);
}
void action_half_open_grip(void)
{
	ausbeeSetAngleServo(&servo_grip, 12);
}

void action_close_grip(void)
{
	ausbeeSetAngleServo(&servo_grip, 7);
}

void action_raise_lift(void)
{
	ausbee_l298_invert_output(&motor_lift, 1);
	ausbee_l298_set_duty_cycle(&motor_lift, 100);

	vTaskDelay(500 / portTICK_RATE_MS);

	ausbee_l298_set_duty_cycle(&motor_lift, 0);
}

void action_lower_lift(void)
{
	ausbee_l298_invert_output(&motor_lift, 0);
	ausbee_l298_set_duty_cycle(&motor_lift, 70);

	vTaskDelay(700 / portTICK_RATE_MS);

	ausbee_l298_set_duty_cycle(&motor_lift, 0);
}
