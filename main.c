#include "platform.h"
#include "servo.h"
#include "l298_driver.h"
#include "lm18200_driver.h"
#include "ax12_driver.h"
#include "position_manager.h"
#include "astar.h"
#include "smooth_traj_manager.h"
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
#include "timers.h"


ausbeeServo servo1;
ausbeeServo servo2;
ausbeeServo servo_clapet;
ausbeeServo servo4;
ausbee_lm18200_chip motor1;
ausbee_lm18200_chip motor2;
ausbee_ax12_chip ax12_1;


/* Private function prototypes -----------------------------------------------*/
void init(void);
void blink1(void* p);
void demo_square_task(void*);
void send_by_can(int);
void test(void* p);
void testLidar(void* p);

int main(void)
{

	init();

//	while(1){
//		printf("adc1 : %d\n\r", (int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC1_PIN));
//		printf("adc2 : %d\n\r", (int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC2_PIN));
//		printf("adc3 : %d\n\r", (int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC3_PIN));
//		printf("adc4 : %d\n\r", (int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC4_PIN));
//	}

	//motors_wrapper_test();

	cli_start(); //invité de commande
	xTaskCreate(blink1, (const signed char *)"LED1", 100, NULL, 1, NULL );
	//xTaskCreate(demo_square_task, (const signed char *)"DemoSquare", 100, NULL, 1, NULL );
	xTaskCreate(testLidar, (const signed char *)"Test", 200, NULL, 2, NULL );

	//send_by_can(1);


	/*smooth_traj_goto_xy_mm(0, 500);
	smooth_traj_goto_xy_mm(500, 500);*/
	/*smooth_traj_goto_xy_mm(600, 700);
	smooth_traj_goto_xy_mm(0, 700);*/

	//start_match();
	vTaskStartScheduler();
	while (1) {

		//platform_led_set(PLATFORM_LED0);
		//for (int i = 0; i < 1000000; i++);

		platform_led_toggle(PLATFORM_LED0);
		for (volatile int i = 0; i < 500000; i++);
		//ausbee_ax12_set_goal(&ax12_1, 300, 0);
		ausbee_ax12_set_led(&ax12_1, 1);

		//printf("pos  x = %d,  y = %d \n\r", (int)position_get_x_mm(), (int)position_get_y_mm());

 		/*for (int i = 0; i <360; i++)
 		{
 			//printf("%d    ", (int)ausbee_lidar_get_distance(i));
 			double d = ausbee_lidar_get_distance(i);
 			double angle = (double)i * 0.0174532;// to rad

 			printf("x%d#", (int)(d * cos(angle)));
 			printf("y%d#", (int)(d * sin(angle)));
 		}

 		printf("e\n");*/


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
	platform_adc_init();

	init_can();
	init_lidar();
	init_encoders();
	init_timer();
	position_init(41826, 315);

	// Launching control system
	control_system_start();
	//control_system_debug_start(); // use printf to debug control system

	// Launching trajectory manager
	smooth_traj_init();
	smooth_traj_start();
	
	//init A*
	initObstacle();


	ausbeeInitStructServo(&servo1);
	ausbeeInitStructServo(&servo2);
	ausbeeInitStructServo(&servo_clapet);
	ausbeeInitStructServo(&servo4);

	servo1.TIMx = SERVO1_TIM;
	servo1.CHANx = SERVO1_CHAN;
	servo2.TIMx = SERVO2_TIM;
	servo2.CHANx = SERVO2_CHAN;
	servo_clapet.TIMx = SERVO3_TIM;
	servo_clapet.CHANx = SERVO3_CHAN;
	servo4.TIMx = SERVO4_TIM;
	servo4.CHANx = SERVO4_CHAN;

	ausbeeInitServo(&servo1);
	ausbeeInitServo(&servo2);
	ausbeeInitServo(&servo_clapet);
	ausbeeInitServo(&servo4);


	platform_motor1_init(&motor1);
	platform_motor2_init(&motor2);

	motors_wrapper_init_lm18200(&motor2, &motor1);

	platform_init_AX12(115200);
	ax12_1.id = 0xFE;
	ax12_1.usart = UART4;

	printf("end init\n\r");
}


void demo_square_task(void *data)
{
  for(;;)
  {
      /*trajectory_goto_d_mm(100);
	  while(!trajectory_is_ended());
      trajectory_goto_a_rel_deg(90);
      while(!trajectory_is_ended());*/
  }
}

void send_by_can(int cmd){

	CanTxMsg CAN_Tx;

	if(cmd == 1)
	{
		CAN_Tx.StdId = 0x80;
		CAN_Tx.Data[0] = 1;
	}

	else if(cmd == 2)
	{
		CAN_Tx.StdId = 0x81;
		CAN_Tx.Data[0] = 1;
	}

	CAN_Tx.ExtId = 0;
	CAN_Tx.IDE = CAN_Id_Standard; // format
	CAN_Tx.RTR = CAN_RTR_Data; // on transmet une donnée
	CAN_Tx.DLC = 1; // longueur du tableau data

	uint8_t mailbox_number = CAN_Transmit(CAN1, &CAN_Tx);

	uint8_t transmit_status = CAN_TransmitStatus(CAN1, mailbox_number);
	while(transmit_status != CAN_TxStatus_Ok)
	{
		transmit_status = CAN_TransmitStatus(CAN1, mailbox_number);
		if(transmit_status == CAN_TxStatus_Ok)
		{
			platform_led_set(PLATFORM_LED4);
			platform_led_reset(PLATFORM_LED7);
		}
		else if( transmit_status == CAN_TxStatus_Pending)
		{
			platform_led_set(PLATFORM_LED7);
		}
		else
		{
			platform_led_set(PLATFORM_LED6);
		}
	}
}

void blink1(void* p)
{
	//int i = 0;
	for (;;) {
		platform_led_toggle(PLATFORM_LED0);
//		ausbeeSetAngleServo(&servo1, (uint8_t)(i %100));
//		ausbeeSetAngleServo(&servo2, (uint8_t)(i %100));
//		ausbeeSetAngleServo(&servo_clapet, (uint8_t)(i %100));
//		ausbeeSetAngleServo(&servo4, (uint8_t)(i %100));
//		i+=10;
		vTaskDelay(1000 / portTICK_RATE_MS); // 1000 ms
	}
}

void testLidar(void* p)
{
	while(1){
		for (int i = 0; i <360; i++)
		{
			//printf("%d    ", (int)ausbee_lidar_get_distance(i));
			int d = ausbee_lidar_get_distance(i);
			double angle = (double)i * 0.0174532;// to rad
			printf("i%d#", (int)(i ));
			printf("d%d#",d );
			printf("x%d#", (int)(d * cos(angle)));
			printf("y%d#", (int)(d * sin(angle)));
			printf("--- \n\r");
			vTaskDelay(10 / portTICK_RATE_MS); // 1000 ms
		}

		printf("e\n");
	}
}

 long lExpireCounter = 0;
void vTimerCallback( xTimerHandle pxTimer )
 {

 const long xMaxExpiryCountBeforeStopping = 900;

 	   // Optionally do something if the pxTimer parameter is NULL.
 	   configASSERT( pxTimer );


 	 platform_led_toggle(PLATFORM_LED7);
 	 // Increment the number of times that pxTimer has expired.
     lExpireCounter += 1;

     // If the timer has expired 10 times then stop it from running.
     if( lExpireCounter == xMaxExpiryCountBeforeStopping )
     {
         // Do not use a block time if calling a timer API function from a
         // timer callback function, as doing so could cause a deadlock!
         xTimerStop( pxTimer, 0 );
     }
 }
xTimerHandle xTimer;
void init_timer()
{
	printf("start \n\r");
	xTimer = xTimerCreate("Timer",( 100 / portTICK_RATE_MS), pdTRUE, ( void * )1, vTimerCallback);
	printf("start timer test \n\r");
}

void start_match(){

	while((int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC1_PIN))vTaskDelay(500 / portTICK_RATE_MS);
		if( xTimer == NULL )
		          {
						printf("ca marche pas 1 \n\r");
		              // The timer was not created.
		          }
		          else
		          {
		        	  printf("ca marche peut etre \n\r");
		              // Start the timer.  No block time is specified, and even if one was
		              // it would be ignored because the scheduler has not yet been
		              // started.
		              if( xTimerStart( xTimer, 0 ) != pdPASS )
		              {
		            	  printf("en fait non \n\r");
		                  // The timer could not be set into the Active state.
		              }
		          }
}

void wait_inter(){
	while((int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC2_PIN) || !(int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC3_PIN))
	{
		printf("adc2 : %d\n\r", !(int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC2_PIN));
		printf("adc3 : %d\n\r", (int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC3_PIN));//printf("repositioning ... \n\r");
		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

void test(void* p)
{

	start_match();
	//while(1)vTaskDelay(500 / portTICK_RATE_MS);
	// reduit vitesse et acceleration
	control_system_set_speed_low();
	//control_system_set_distance_max_acc();

	printf("start repositioning \n");
	smooth_traj_goto_d_mm(400.0);

	wait_inter();

	smooth_traj_end();
	printf("repositioning done");
	smooth_traj_goto_d_mm(-100.0);
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
	smooth_traj_goto_a_deg(90.0);
	printf("j'ai bien tourner youpi \n");
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
	smooth_traj_goto_d_mm(400.0);
	printf("je suis perdu \n");

	wait_inter();

	smooth_traj_end();
	printf("repositioning done");
	smooth_traj_goto_d_mm(-100.0);
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
	smooth_traj_goto_a_deg(180.0);
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
	printf("test bizarre");
	smooth_traj_goto_d_mm(200.0);
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
	printf("finish");
	//ready to go

	// Repositionnement X, Y, angle
	vTaskDelay(500 / portTICK_RATE_MS); // 500 ms

 	position_set_xy_mm(1000.f, 470.f);
 	position_set_angle_deg(0.f);

	smooth_traj_goto_d_mm(200.0);
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);

	while(1){
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

	ausbeeSetAngleServo(&servo_clapet, 90);
}
