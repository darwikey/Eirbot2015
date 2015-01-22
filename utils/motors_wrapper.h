#ifndef MOTORS_WRAPPER_H
#define MOTORS_WRAPPER_H

#include <stdint.h>
#include <AUSBEE/l298_driver.h>
#include <AUSBEE/lm18200_driver.h>


enum motor { LEFT_MOTOR, RIGHT_MOTOR };

void motors_wrapper_init_l298(struct ausbee_l298_chip* right_motor, struct ausbee_l298_chip* left_motor);
void motors_wrapper_init_lm18200(ausbee_lm18200_chip* right_motor, ausbee_lm18200_chip* left_motor);

void motors_wrapper_motor_set_duty_cycle(enum motor id_motor, float duty_cycle);
void motors_wrapper_right_motor_set_duty_cycle(float duty_cycle);
void motors_wrapper_left_motor_set_duty_cycle(float duty_cycle);

uint8_t motors_wrapper_right_motor_is_moving_forward(void);
uint8_t motors_wrapper_left_motor_is_moving_forward(void);

void motors_wrapper_test(void);

#endif /* MOTORS_WRAPPER_H */
