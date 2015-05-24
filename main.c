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
#include "actions.h"

ausbee_lm18200_chip motor1;
ausbee_lm18200_chip motor2;



//map
//	 	 1  4  7  10 13 16 19 22 25 28 31 34 37 40 43 46 49 52 55 58
//	 	  2  5  8  11 14 17 20 23 26 29 32 35 38 41 44 47 50 53 56 59
//		0  3  6  9  12 15 18 21 24 27 30 33 36 39 42 45 48 51 54 57 60
//
//0		1111111111111111111111111111111111111111111111111111111111111
//1		1111111111111111111111111111111111111111111111111111111111111
//2		1111111111111111111111111111111111111111111111111111111111111
//3		1110000000000000111111111111111111111111111110000000000000111
//4		1110000000000000111111111111111111111111111110000000000000111
//5		1110000000000000111111111111111111111111111110000000000000111
//6		1110000000000000111111111111111111111111111110000000000000111
//7		1110000000000000111111111111111111111111111110000000000000111
//8		1110000000000000111111111111111111111111111110000000000000111
//9		1110000000000000111111111111111111111111111110000000000000111
//10	1110000000000000111111111111111111111111111110000000000000111
//11	1110000000000000111111111111111111111111111110000000000000111
//12	1110000000000000111111111111111111111111111110000000000000111
//13	1110000000000000000000000000000000000000000000000000000000111
//14	1111111111100000000000000000000000000000000000000011111111111
//15	1111111111100000000000000000000000000000000000000011111111111
//16	1111111111100000000000000000000000000000000000000011111111111
//17	1111111111100000000000000000000000000000000000000011111111111
//18	1111111111100000000000000000000000000000000000000011111111111
//19	1111111111100000000000000000000000000000000000000011111111111
//20	1111111111100000000000000000000000000000000000000011111111111
//21	1111111111100000000000000000000000000000000000000011111111111
//22	1111111111100000000000000000000000000000000000000011111111111
//23	1111111111100000000000000000000000000000000000000011111111111
//24	1111111111100000000000000000000000000000000000000011111111111
//25	1111111111100000000000000000000000000000000000000011111111111
//26	1111111111100000000000000000000000000000000000000011111111111
//27	1110000000000000000000000000000000000000000000000000000000111
//28	1110000000000000000000000000000000000000000000000000000000111
//29	1110000000000000000000000000000000000000000000000000000000111
//30	1110000000000000000000000000000000000000000000000000000000111
//31	1110000000000000000000000000000000000000000000000000000000111
//32	1110000000000000000000000000000000000000000000000000000000111
//33	1110000000000000000000000000000000000000000000000000000000111
//34	1110000000000000000000000000000000000000000000000000000000111
//35	1110000000000000000000000000000000000000000000000000000000111
//36	1110000000000000000000000000000000000000000000000000000000111
//37	1110000000000000000000000000000000000000000000000000000000111
//38	1111111111111111111111111111111111111111111111111111111111111
//39	1111111111111111111111111111111111111111111111111111111111111
//40	1111111111111111111111111111111111111111111111111111111111111
/* Private function prototypes -----------------------------------------------*/
void init(void);
void blink1(void* p);
void demo_square_task(void*);
void send_by_can(int);
void test(void* p);
void lidarDetection(void* p);
void test_lift(void* p);

#define ABS(x) (((x) < 0)? -(x): (x))
#define GREEN 1
#define YELLOW 0

int detection;

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

	cli_start(); //invit�Ede commande
	xTaskCreate(blink1, (const signed char *)"LED1", 100, NULL, 1, NULL );
	//xTaskCreate(demo_square_task, (const signed char *)"DemoSquare", 100, NULL, 1, NULL );
	xTaskCreate(lidarDetection, (const signed char *)"lidar", 200, NULL, 2, NULL );
	xTaskCreate(test, (const signed char *)"Test", 200, NULL, 2, NULL );
	//send_by_can(1);

	position_set_xy_mm(1000.f, 470.f);

	/*smooth_traj_goto_xy_mm(0, 500);
	smooth_traj_goto_xy_mm(500, 500);*/
	/*smooth_traj_goto_xy_mm(600, 700);
	smooth_traj_goto_xy_mm(0, 700);*/

	//start_match();
	action_close_clasp();
	detection = 0;

	vTaskStartScheduler();
	while (1) {

		//platform_led_set(PLATFORM_LED0);
		//for (int i = 0; i < 1000000; i++);

		platform_led_toggle(PLATFORM_LED0);
		for (volatile int i = 0; i < 500000; i++);
		//ausbee_ax12_set_goal(&ax12_1, 300, 0);
		//ausbee_ax12_set_led(&ax12_1, 1);

		//printf("pos  x = %d,  y = %d \n\r", (int)position_get_x_mm(), (int)position_get_y_mm());


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

	init_actions();
	platform_motor1_init(&motor1);
	platform_motor2_init(&motor2);

	motors_wrapper_init_lm18200(&motor2, &motor1);

	/*platform_init_AX12(115200);
	ax12_1.id = 0xFE;
	ax12_1.usart = UART4;*/

	printf("end init\n\r");
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

void lidarDetection(void* p)
{
	int obstacleDetected = 0;
	while(1){

		if(detection)
		{
			for (int i = 180 + 30; i <= 360 - 30; i++)
				{
					//printf("%d    ", (int)ausbee_lidar_get_distance(i));
					double d = ausbee_lidar_get_distance(i);
					if (d < 100.0){
						continue;
					}

					const double epsilon = 30;
					double d1 = ausbee_lidar_get_distance(i-1);
					double d2 = ausbee_lidar_get_distance(i+1);
					if (ABS(d1 - d) > epsilon || ABS(d2 - d) > epsilon){
						continue;
					}

					double angle = (double)(i - 270) * 0.0174532;// to rad
					int16_t x = (int)(d * cos(angle));
					int16_t y = -(int)(d * sin(angle));
			//	 			printf("cos %f sin %f ", cos(angle), sin(angle));
			//	 			printf("dist x %d y %d angle %d \n\r", x, y, i);
					platform_led_toggle(PLATFORM_LED5);
//			putObstacle(getCoor((int)position_get_y_mm() + x, (int)position_get_x_mm() + y));
					if(d < 300 && !isObstacle(getCoor((int)position_get_y_mm() + y, (int)position_get_x_mm() + x)))
					{
						obstacleDetected = 1;
						printf("x %d, y %d , posx = %d posy = %d distX = %d distY = %d angle = %d dist = %f \n\r",(int)position_get_y_mm() + x,(int)position_get_x_mm() + y, (int)position_get_y_mm(),(int)position_get_x_mm(), y , x, i, (float)d);
					}
				}
			if(obstacleDetected)//pause
			{
				smooth_traj_pause();
			}
			else{
				smooth_traj_resume();
			}
			obstacleDetected = 0;
			if(!isTrajectoryValid())
			{
				stopAstarMovement();
			}
			clearGraphe();
			initObstacle();


			//		printf("e\n");
		}
		vTaskDelay(100 / portTICK_RATE_MS);
	}

}

long lExpireCounter = 0;
void vTimerCallback( xTimerHandle pxTimer )
 {

 const long xMaxExpiryCountBeforeStopping = 800;

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

	while((int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC1_PIN))vTaskDelay(100 / portTICK_RATE_MS);
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
	//while(!(int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC2_PIN) || !(int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC3_PIN))
	while(!(int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC3_PIN))
	{
		//printf("adc2 : %d\n\r", (int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC2_PIN));
		printf("adc3 : %d\n\r", (int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC3_PIN));//printf("repositioning ... \n\r");
		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

void test(void* p)
{

	/*smooth_traj_goto_d_mm(1000);
	vTaskDelay(1000 / portTICK_RATE_MS);
	smooth_traj_pause();
	vTaskDelay(2000 / portTICK_RATE_MS);
	smooth_traj_resume();*/

//	smooth_traj_goto_d_mm(400.0);
//	while(1)vTaskDelay(500 / portTICK_RATE_MS);
	// reduit vitesse et acceleration
	control_system_set_speed_low();
	detection = 0;
	//control_system_set_distance_max_acc();
	int color;
	if ((int)GPIO_ReadInputDataBit(ADC1234_PORT, ADC2_PIN)){
		color = YELLOW;
		printf("color = yellow\n\r");
	}
	else{
		color = GREEN;
		printf("color = green\n\r");
	}
	//color = GREEN
	printf("start repositioning \n\r");
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
	smooth_traj_goto_d_mm(150.0);
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
	printf("finish");
	//ready to go

	// Repositionnement X, Y, angle
	vTaskDelay(500 / portTICK_RATE_MS); // 500 ms

	if(color == YELLOW){
		position_set_xy_mm(1000.f, 420.f );
		position_set_angle_deg(0.f);
	}
	else{
		position_set_xy_mm(1000.f, 2580.f);
	}
	start_match();
	detection = 1;
	smooth_traj_goto_d_mm(200.0);
	while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);



	//set_startCoor(positionMmToCoordinate(position_get_y_mm(),position_get_x_mm()));
	//set_goalCoor(positionMmToCoordinate(1850, 150));
	//while(astarMv())vTaskDelay(50 / portTICK_RATE_MS);;
	//start code of the clasp
	if(color == YELLOW){
//		smooth_traj_goto_xy_mm(1700.f, 750.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		action_open_clasp();
//		//smooth_traj_goto_a_deg(0.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		smooth_traj_goto_xy_mm(1700.f, 750.f + 250.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		action_close_clasp();
//		smooth_traj_goto_d_mm(-750.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		action_open_clasp();
//		smooth_traj_goto_d_mm(200.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);

		smooth_traj_goto_xy_mm(1700.f, 750.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		action_open_clasp();
		smooth_traj_goto_a_deg(0.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_d_mm(300.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		action_close_clasp();
		/*smooth_traj_goto_xy_mm(1300.f, 870.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_a_deg(0.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_xy_mm(1300.f, 2130.f);
		smooth_traj_goto_xy_mm(1700.f, 2600.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_a_deg(0.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		action_open_clasp();
		smooth_traj_goto_d_mm(-300.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		action_close_clasp();*/

	}
	else{ // GREEN
//		smooth_traj_goto_xy_mm(1700.f, 2330.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		smooth_traj_goto_a_deg(0.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		action_open_clasp();
//		smooth_traj_goto_d_mm(-250.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);

		smooth_traj_goto_xy_mm(1700.f, 2300.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_a_deg(0.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		action_open_clasp();
		smooth_traj_goto_d_mm(-300.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		action_close_clasp();
		/*smooth_traj_goto_xy_mm(1300.f, 2130.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_a_deg(180.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_xy_mm(1300.f, 870.f);
		smooth_traj_goto_xy_mm(1750.f, 400.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		action_open_clasp();
		smooth_traj_goto_a_deg(0.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
		smooth_traj_goto_d_mm(300.f);
		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);*/


//		smooth_traj_goto_xy_mm(1500.f, 2330.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		smooth_traj_goto_xy_mm(1700.f, 2800.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		smooth_traj_goto_a_deg(0.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
//		action_open_clasp();
//		smooth_traj_goto_d_mm(-300.f);
//		while(!smooth_traj_is_ended())vTaskDelay(100 / portTICK_RATE_MS);
	}
	while(1){
		vTaskDelay(1000 / portTICK_RATE_MS);
	}


}


void test_lift(void* p)
{

	while(1){
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}
