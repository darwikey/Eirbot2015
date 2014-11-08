#ifndef INIT_H
#define INIT_H

//#include "misc.h"
#include "platform.h"
#include <AUSBEE/servo.h>
#include <AUSBEE/l298_driver.h>
#include <AUSBEE/lidar.h>

void init_can();
void init_can_rx_interrupt();
void init_usart1_interrupt();
//void init_servo();
void init_encoders();
void init_lidar();
void init_gpio_robot();
#endif //INIT_H
