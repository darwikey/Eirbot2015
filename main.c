#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "platform.h"
#include "servo.h"
#include "l298_driver.h"
#include "position_manager.h"
#include "init.h"
#include "stdio.h"
#include "define.h"
#include "motors_wrapper.h"
#include "lidar.h"
#include "math.h"


ausbeeServo servo1;
ausbeeServo servo2;
ausbeeServo servo3;
ausbeeServo servo4;
struct ausbee_l298_chip motor1;
struct ausbee_l298_chip motor2;


/* Private function prototypes -----------------------------------------------*/
void init(void);

int main(void)
{

	init();



	while (1) {

		//platform_led_set(PLATFORM_LED0);
		//for (int i = 0; i < 1000000; i++);

		//platform_led_reset(PLATFORM_LED0);
		for (int i = 0; i < 5000; i++);


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
	SysTick_Config(24000000 / 900);

	platform_usart_init(USART3, 115200);
	printf("start init\n\r");

	platform_pwm_init(TIMERALL);
	platform_led_init();
	init_lidar();
	init_encoders();
	position_init(54320, 237);


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

#ifdef ENABLE_MOTOR
	platform_motor1_init(&motor1);
	platform_motor2_init(&motor2);

	motors_wrapper_init(&motor1, &motor2);
#endif

	printf("end init\n\r");
}


// Interruption
void SysTick_Handler()
{
	static s32 timer = 0;
	static u8 angle = 0;

	if (timer%30==0)
	{
		angle++;

		if (angle>100)
		{
			angle=0;
		}
	}

 	if (timer++ > 3000)
 	{
 		timer = 0;

 		//Debug
 		//printf("pos  x = %d,  y = %d \n\r", (int)position_get_x_mm(), (int)position_get_y_mm());

 		// LED
 		platform_led_toggle(PLATFORM_LED7);

#ifdef ENABLE_SERVO
 		ausbeeSetAngleServo(&servo1, angle);
 		ausbeeSetAngleServo(&servo2, angle);
 		ausbeeSetAngleServo(&servo3, angle);
 		ausbeeSetAngleServo(&servo4, angle);
#endif

 	}

}

