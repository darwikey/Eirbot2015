#include "platform.h"
#include "servo.h"
#include "l298_driver.h"
#include "position_manager.h"
#include "trajectory_manager.h"
#include "control_system_debug.h"
#include "init.h"
#include "stdio.h"
#include "define.h"
#include "motors_wrapper.h"
#include "lidar.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cli.h"


ausbeeServo servo1;
ausbeeServo servo2;
ausbeeServo servo3;
ausbeeServo servo4;
struct ausbee_l298_chip motor1;
struct ausbee_l298_chip motor2;

/* Private function prototypes -----------------------------------------------*/
void init(void);
void blink1(void* p);
void demo_square_task(void*);


int main(void)
{

	init();

	cli_start();
	xTaskCreate(blink1, (const signed char *)"LED1", 100, NULL, 1, NULL );
	//xTaskCreate(demo_square_task, (const signed char *)"DemoSquare", 100, NULL, 1, NULL );

	vTaskStartScheduler();

	while (1) {

		//platform_led_set(PLATFORM_LED0);
		//for (int i = 0; i < 1000000; i++);

		//platform_led_reset(PLATFORM_LED0);
		for (int i = 0; i < 500000; i++);
		//printf("pos  x = %d,  y = %d \n\r", (int)position_get_x_mm(), (int)position_get_y_mm());

 		for (int i = 0; i <360; i++)
 		{
 			//printf("%d    ", (int)ausbee_lidar_get_distance(i));
 			double d = ausbee_lidar_get_distance(i);
 			double angle = (double)i * 0.0174532;// to rad

 			printf("x%d#", (int)(d * cos(angle)));
 			printf("y%d#", (int)(d * sin(angle)));
 		}

 		printf("e\n");


	}

	return 1;
}





/* Private functions ---------------------------------------------------------*/
void init(void) {

	SystemInit();
	//SysTick_Config(SystemCoreClock / 1000);
	//SysTick_Config(24000000 / 900);

	platform_usart_init(USART3, 115200);
	printf("start init\n\r");

	platform_pwm_init(TIMERALL);
	platform_led_init();

	init_lidar();
	init_encoders();
	position_init(41826, 315);

	// Launching control system
	control_system_start();
	//control_system_debug_start();

	// Launching trajectory manager
	trajectory_init();
	trajectory_start();


#ifdef ENABLE_SERVO
	ausbeeInitStructServo(&servo1);
	ausbeeInitStructServo(&servo2);
	ausbeeInitStructServo(&servo3);
	ausbeeInitStructServo(&servo4);

	servo1.TIMx = SERVO1_TIM;
	servo1.CHANx = SERVO1_CHAN;
	servo2.TIMx = SERVO2_TIM;
	servo2.CHANx = SERVO2_CHAN;
	servo3.TIMx = SERVO3_TIM;
	servo3.CHANx = SERVO3_CHAN;
	servo4.TIMx = SERVO4_TIM;
	servo4.CHANx = SERVO4_CHAN;

	ausbeeInitServo(&servo1);
	ausbeeInitServo(&servo2);
	ausbeeInitServo(&servo3);
	ausbeeInitServo(&servo4);

#endif

	platform_motor1_init(&motor1);
	platform_motor2_init(&motor2);

	motors_wrapper_init(&motor2, &motor1);


	printf("end init\n\r");
}


void demo_square_task(void *data)
{
  for(;;)
  {
      //trajectory_goto_d_mm(100);
	  trajectory_goto_a_rel_deg(160);
      while(1);
      while(!trajectory_is_ended());
      trajectory_goto_a_rel_deg(90);
      while(!trajectory_is_ended());
  }
}


void blink1(void* p)
{
  for (;;) {
    platform_led_toggle(PLATFORM_LED0);

    vTaskDelay(1000 / portTICK_RATE_MS); // 1000 ms
  }
}

