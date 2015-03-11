/**
 *
 * \file platform.c
 * \brief Platform support implementation for AUSBEE mainboard V0.1
 * \authors Kevin JOLY <joly.kevin25@gmail.com> Vincent FAURE <vincenr.hr.faure@gmail.com>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stm32f4xx.h>
#include "platform.h"
#include "encoder.h"
#include "l298_driver.h"
#include "lm18200_driver.h"

//Private functions
void platform_motor1_init_io(void);
void platform_motor2_init_io(void);

void platform_hse_pll_init(void) {
	RCC_DeInit();

	RCC_HSEConfig(RCC_HSE_ON);
	RCC_WaitForHSEStartUp();

	RCC_PLLConfig(RCC_PLLSource_HSE, PLATFORM_PLLM, PLATFORM_PLLN,
			PLATFORM_PLLP, PLATFORM_PLLQ);

	RCC_PLLCmd(ENABLE);

	while (!(RCC->CR & RCC_CR_PLLRDY))
		;

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

void platform_pwm_init(uint8_t timer) {
	// control structures

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); //give clock to the GPIO

	TIM_TimeBaseInitTypeDef TimeBaseInit_PWM;
	GPIO_InitTypeDef InitTypeDef_PWM;

	TIM_TimeBaseStructInit(&TimeBaseInit_PWM); //initialize the struct

	//TimeBaseInit_PWM.TIM_Prescaler=3300; //168MHz/50Khz
	TimeBaseInit_PWM.TIM_Prescaler = 330;

	TimeBaseInit_PWM.TIM_CounterMode = TIM_CounterMode_Up; //counter mode up
	TimeBaseInit_PWM.TIM_Period = 1000; //50Khz/50Hz

	TimeBaseInit_PWM.TIM_ClockDivision = 0x0000; //is not used

	TimeBaseInit_PWM.TIM_RepetitionCounter = 0x0000; //is not used	 

	if ((timer & TIMER10) == TIMER10) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE); // Enable APB2 clock on TIM10
		TIM_TimeBaseInit(TIM10, &TimeBaseInit_PWM); // Initialize TIM10
		TIM_Cmd(TIM10, ENABLE);
	}

	if ((timer & TIMER11) == TIMER11) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE); // Enable APB2 clock on TIM11
		TIM_TimeBaseInit(TIM11, &TimeBaseInit_PWM); // Initialize TIM11
		TIM_Cmd(TIM11, ENABLE);
	}

	if ((timer & TIMER13) == TIMER13) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); // Enable APB1 clock on TIM13
		TIM_TimeBaseInit(TIM13, &TimeBaseInit_PWM); // Initialize TIM13
		TIM_Cmd(TIM13, ENABLE);
	}

	if ((timer & TIMER14) == TIMER14) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); // Enable APB1 clock on TIM14
		TIM_TimeBaseInit(TIM14, &TimeBaseInit_PWM); // Initialize TIM14
		TIM_Cmd(TIM14, ENABLE);
	}

	// IO
	GPIO_StructInit(&InitTypeDef_PWM);
	InitTypeDef_PWM.GPIO_Pin = Pin_PWM_GPIOF;
	InitTypeDef_PWM.GPIO_Speed = GPIO_Speed_50MHz; // Output maximum frequency at 100MHz
	InitTypeDef_PWM.GPIO_Mode = GPIO_Mode_AF; // Mode alternate function
	//InitTypeDef_PWM.GPIO_OType = GPIO_OType_PP;			// Mode Push-Pull
	InitTypeDef_PWM.GPIO_PuPd = GPIO_PuPd_NOPULL; // No pull-up/ pull down
	GPIO_Init(GPIOF, &InitTypeDef_PWM); // Initialize PWM Pin on GPIOF
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_TIM11);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_TIM13);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);
}

int platform_usart_init(USART_TypeDef *USARTx, uint32_t baudrate) {
	GPIO_InitTypeDef init_GPIO_USART;
	USART_InitTypeDef init_USART;

	USART_StructInit(&init_USART);
	GPIO_StructInit(&init_GPIO_USART);

	if (USARTx == USART3) {
		/* Enable clock on GPIOB and USART3 devices */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		/* Select AF mode USART3 for pin B10 and B11 */
		init_GPIO_USART.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		init_GPIO_USART.GPIO_Speed = GPIO_Speed_50MHz;
		init_GPIO_USART.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &init_GPIO_USART);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

		/* Init USART 3 */
		init_USART.USART_BaudRate = baudrate;
		init_USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USARTx, &init_USART);

		USART_Cmd(USARTx, ENABLE);
	} else if (USARTx == USART2) {
		//enable clock for GPIOA and USART2 devices
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		//Select AF mode USART2 for pin PA2 and PA3
		init_GPIO_USART.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
		init_GPIO_USART.GPIO_Speed = GPIO_Speed_50MHz;
		init_GPIO_USART.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &init_GPIO_USART);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

		//Init USART2
		init_USART.USART_BaudRate = baudrate;
		init_USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USARTx, &init_USART);

		USART_Cmd(USARTx, ENABLE);
	} else if (USARTx == USART1) {
		//enable clock for GPIOA and USART2 devices
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		//Select AF mode USART1 for pin PB6 and PB7
		init_GPIO_USART.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		init_GPIO_USART.GPIO_Speed = GPIO_Speed_50MHz;
		init_GPIO_USART.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &init_GPIO_USART);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

		//Init USART1
		init_USART.USART_BaudRate = baudrate;
		init_USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USARTx, &init_USART);

		USART_Cmd(USARTx, ENABLE);
	} else {
		return -1;
	}

	return 0;
}

/**
 * @brief  Initialize USART
 * @param  baudrate: USART baudrate
 * @retval None
 */
void platform_init_AX12(uint32_t baudrate)
{
	GPIO_InitTypeDef init_GPIO_USART;
	USART_InitTypeDef init_USART;

	USART_StructInit(&init_USART);
	GPIO_StructInit(&init_GPIO_USART);

	// Enable clock on GPIOBA and USART4 devices
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	// Select AF mode USART3 for pin A0 and A1
	init_GPIO_USART.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	init_GPIO_USART.GPIO_Speed = GPIO_Speed_50MHz;
	init_GPIO_USART.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &init_GPIO_USART);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

	// Init USART 4
	init_USART.USART_BaudRate = baudrate;
	init_USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &init_USART);

	USART_Cmd(UART4, ENABLE);
}


/* Brief function to initialize GPIO available on the board
 * 
 * gpio         : GPIOX where x is 1 to 9
 * type         : define the type of IO, could be GPIO_OType_PP (push pull) or GPIO_OType_OD (open drain)
 * mode         : mode of the pin, could be GPIO_Mode_IN,  GPIO_Mode_OUT, GPIO_Mode_AF (alternate function), GPIO_Mode_AN (analog)
 * speed        : speed of the gpio, could be GPIO_Low_Speed, GPIO_Medium_Speed, GPIO_Fast_Speed, GPIO_High_Speed
 * pull_up_down : define the type of pull up/down resistor, could be GPIO_PuPd_NOPULL, GPIO_PuPd_UP, GPIO_PuPd_DOWN
 * */
void platform_gpio_init(uint16_t gpio, uint8_t type, uint8_t mode,
		uint8_t speed, uint8_t pull_up_down) {
	//initialize the struct
	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);

	//Give the clock to the GPIO
	if (gpio & GPIO1 || gpio & GPIO2) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	}
	if (gpio & GPIO3 || gpio & GPIO4 || gpio & GPIO5 || gpio & GPIO6
			|| gpio & GPIO7 || gpio & GPIO8 || gpio & GPIO9) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	}

	//initialize value of the struct
	if (gpio & GPIO1) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO1_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOD, &GPIOInitStruct);
	}
	if (gpio & GPIO2) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO2_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOD, &GPIOInitStruct);
	}
	if (gpio & GPIO3) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO3_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOG, &GPIOInitStruct);
	}
	if (gpio & GPIO4) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO4_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOG, &GPIOInitStruct);
	}
	if (gpio & GPIO5) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO5_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOG, &GPIOInitStruct);
	}
	if (gpio & GPIO6) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO6_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOG, &GPIOInitStruct);
	}
	if (gpio & GPIO7) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO7_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOG, &GPIOInitStruct);
	}
	if (gpio & GPIO8) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO8_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOG, &GPIOInitStruct);
	}
	if (gpio & GPIO9) {
		GPIOInitStruct.GPIO_Pin = PLATFORM_GPIO9_PIN;
		GPIOInitStruct.GPIO_Speed = speed;
		GPIOInitStruct.GPIO_Mode = mode;
		GPIOInitStruct.GPIO_OType = type;
		GPIOInitStruct.GPIO_PuPd = pull_up_down;
		GPIO_Init(GPIOG, &GPIOInitStruct);
	}

}

void platform_gpio_set(uint16_t gpio) {
	if ((gpio & GPIO1) == GPIO1) {
		GPIO_SetBits(PLATFORM_GPIO1_PORT, PLATFORM_GPIO1_PIN);
	}
	if ((gpio & GPIO2) == GPIO2) {
		GPIO_SetBits(PLATFORM_GPIO2_PORT, PLATFORM_GPIO2_PIN);
	}
	if ((gpio & GPIO3) == GPIO3) {
		GPIO_SetBits(PLATFORM_GPIO3_PORT, PLATFORM_GPIO3_PIN);
	}
	if ((gpio & GPIO4) == GPIO4) {
		GPIO_SetBits(PLATFORM_GPIO4_PORT, PLATFORM_GPIO4_PIN);
	}
	if ((gpio & GPIO5) == GPIO5) {
		GPIO_SetBits(PLATFORM_GPIO5_PORT, PLATFORM_GPIO5_PIN);
	}
	if ((gpio & GPIO6) == GPIO6) {
		GPIO_SetBits(PLATFORM_GPIO6_PORT, PLATFORM_GPIO6_PIN);
	}
	if ((gpio & GPIO7) == GPIO7) {
		GPIO_SetBits(PLATFORM_GPIO7_PORT, PLATFORM_GPIO7_PIN);
	}
	if ((gpio & GPIO8) == GPIO8) {
		GPIO_SetBits(PLATFORM_GPIO8_PORT, PLATFORM_GPIO8_PIN);
	}
	if ((gpio & GPIO9) == GPIO9) {
		GPIO_SetBits(PLATFORM_GPIO9_PORT, PLATFORM_GPIO9_PIN);
	}
}

void platform_gpio_reset(uint16_t gpio) {
	if ((gpio & GPIO1) == GPIO1) {
		GPIO_ResetBits(PLATFORM_GPIO1_PORT, PLATFORM_GPIO1_PIN);
	}
	if ((gpio & GPIO2) == GPIO2) {
		GPIO_ResetBits(PLATFORM_GPIO2_PORT, PLATFORM_GPIO2_PIN);
	}
	if ((gpio & GPIO3) == GPIO3) {
		GPIO_ResetBits(PLATFORM_GPIO3_PORT, PLATFORM_GPIO3_PIN);
	}
	if ((gpio & GPIO4) == GPIO4) {
		GPIO_ResetBits(PLATFORM_GPIO4_PORT, PLATFORM_GPIO4_PIN);
	}
	if ((gpio & GPIO5) == GPIO5) {
		GPIO_ResetBits(PLATFORM_GPIO5_PORT, PLATFORM_GPIO5_PIN);
	}
	if ((gpio & GPIO6) == GPIO6) {
		GPIO_ResetBits(PLATFORM_GPIO6_PORT, PLATFORM_GPIO6_PIN);
	}
	if ((gpio & GPIO7) == GPIO7) {
		GPIO_ResetBits(PLATFORM_GPIO7_PORT, PLATFORM_GPIO7_PIN);
	}
	if ((gpio & GPIO8) == GPIO8) {
		GPIO_ResetBits(PLATFORM_GPIO8_PORT, PLATFORM_GPIO8_PIN);
	}
	if ((gpio & GPIO9) == GPIO9) {
		GPIO_ResetBits(PLATFORM_GPIO9_PORT, PLATFORM_GPIO9_PIN);
	}
}

/**
 * \fn uint8_t platform_GPIO_get_value(uint16_t gpio)
 * \brief Get a GPIO value.
 *
 * \param gpio GPIO number.
 *
 * \return GPIO value (0 or 1); 2 is return if an invalid GPIO is given as parameter.
 *
 */
uint8_t platform_gpio_get_value(uint16_t gpio) {
	uint8_t return_value = 2;
	if (gpio & GPIO1)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO1_PORT,
				PLATFORM_GPIO1_PIN);
	if (gpio & GPIO2)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO2_PORT,
				PLATFORM_GPIO2_PIN);
	if (gpio & GPIO3)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO3_PORT,
				PLATFORM_GPIO3_PIN);
	if (gpio & GPIO4)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO4_PORT,
				PLATFORM_GPIO4_PIN);
	if (gpio & GPIO5)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO5_PORT,
				PLATFORM_GPIO5_PIN);
	if (gpio & GPIO6)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO6_PORT,
				PLATFORM_GPIO6_PIN);
	if (gpio & GPIO7)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO7_PORT,
				PLATFORM_GPIO7_PIN);
	if (gpio & GPIO8)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO8_PORT,
				PLATFORM_GPIO8_PIN);
	if (gpio & GPIO9)
		return_value = GPIO_ReadInputDataBit(PLATFORM_GPIO9_PORT,
				PLATFORM_GPIO9_PIN);
	return return_value;
}

void platform_led_init(void) {
	GPIO_InitTypeDef GPIOInitStruct_G;
	GPIO_InitTypeDef GPIOInitStruct_D;

	GPIO_StructInit(&GPIOInitStruct_G);
	GPIO_StructInit(&GPIOInitStruct_D);

	/* Clock configuration for GPIOG and GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	/* Init LED on GPIOD */
	GPIOInitStruct_D.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIOInitStruct_D.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct_D.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOD, &GPIOInitStruct_D);

	/* Init LED on GPIOG */
	GPIOInitStruct_G.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
			| GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIOInitStruct_G.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct_G.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOG, &GPIOInitStruct_G);
}

void platform_adc_init(){
	platform_enable_clock_gpio(ADC1234_PORT);

	GPIO_InitTypeDef init_adc;
	GPIO_StructInit(&init_adc);
	init_adc.GPIO_Mode = GPIO_Mode_IN;
	init_adc.GPIO_Pin = ADC1_PIN | ADC2_PIN | ADC3_PIN | ADC4_PIN;
	GPIO_Init(ADC1234_PORT, &init_adc);
}

void platform_led_set(uint8_t led) {
	if (led & PLATFORM_LED0)
		GPIO_SetBits(GPIOD, GPIO_Pin_6);
	if (led & PLATFORM_LED1)
		GPIO_SetBits(GPIOD, GPIO_Pin_7);
	if (led & PLATFORM_LED2)
		GPIO_SetBits(GPIOG, GPIO_Pin_9);
	if (led & PLATFORM_LED3)
		GPIO_SetBits(GPIOG, GPIO_Pin_10);
	if (led & PLATFORM_LED4)
		GPIO_SetBits(GPIOG, GPIO_Pin_11);
	if (led & PLATFORM_LED5)
		GPIO_SetBits(GPIOG, GPIO_Pin_12);
	if (led & PLATFORM_LED6)
		GPIO_SetBits(GPIOG, GPIO_Pin_13);
	if (led & PLATFORM_LED7)
		GPIO_SetBits(GPIOG, GPIO_Pin_14);
}

void platform_led_reset(uint8_t led) {
	if (led & PLATFORM_LED0)
		GPIO_ResetBits(GPIOD, GPIO_Pin_6);
	if (led & PLATFORM_LED1)
		GPIO_ResetBits(GPIOD, GPIO_Pin_7);
	if (led & PLATFORM_LED2)
		GPIO_ResetBits(GPIOG, GPIO_Pin_9);
	if (led & PLATFORM_LED3)
		GPIO_ResetBits(GPIOG, GPIO_Pin_10);
	if (led & PLATFORM_LED4)
		GPIO_ResetBits(GPIOG, GPIO_Pin_11);
	if (led & PLATFORM_LED5)
		GPIO_ResetBits(GPIOG, GPIO_Pin_12);
	if (led & PLATFORM_LED6)
		GPIO_ResetBits(GPIOG, GPIO_Pin_13);
	if (led & PLATFORM_LED7)
		GPIO_ResetBits(GPIOG, GPIO_Pin_14);
}

void platform_led_toggle(uint8_t led) {
	if (led & PLATFORM_LED0)
		GPIO_ToggleBits(GPIOD, GPIO_Pin_6);
	if (led & PLATFORM_LED1)
		GPIO_ToggleBits(GPIOD, GPIO_Pin_7);
	if (led & PLATFORM_LED2)
		GPIO_ToggleBits(GPIOG, GPIO_Pin_9);
	if (led & PLATFORM_LED3)
		GPIO_ToggleBits(GPIOG, GPIO_Pin_10);
	if (led & PLATFORM_LED4)
		GPIO_ToggleBits(GPIOG, GPIO_Pin_11);
	if (led & PLATFORM_LED5)
		GPIO_ToggleBits(GPIOG, GPIO_Pin_12);
	if (led & PLATFORM_LED6)
		GPIO_ToggleBits(GPIOG, GPIO_Pin_13);
	if (led & PLATFORM_LED7)
		GPIO_ToggleBits(GPIOG, GPIO_Pin_14);
}


void platform_motor1_init(ausbee_lm18200_chip* motor1) {
	motor1->timer_channel = PLATFORM_CHANNEL_MOTOR1;
	motor1->gpio_dir_pin = PLATFORM_DIR_MOTOR1_PIN;
	motor1->pwm_frequency = 10000;
	motor1->gpio_dir_port = PLATFORM_DIR_MOTOR1_PORT;
	motor1->TIMx = PLATFORM_TIMER_MOTOR1;
	platform_motor1_init_io();

	ausbee_lm18200_init_chip(motor1);
}

void platform_motor2_init(ausbee_lm18200_chip* motor2) {
	motor2->timer_channel = PLATFORM_CHANNEL_MOTOR2;
	motor2->gpio_dir_pin = PLATFORM_DIR_MOTOR2_PIN;
	motor2->pwm_frequency = 10000;
	motor2->gpio_dir_port = PLATFORM_DIR_MOTOR2_PORT;
	motor2->TIMx = PLATFORM_TIMER_MOTOR2;
	platform_motor2_init_io();

	ausbee_lm18200_init_chip(motor2);
}

void platform_motor1_init_io(void) {
	// Set clocks
	platform_enable_clock_timer(PLATFORM_TIMER_MOTOR1);
	platform_enable_clock_gpio(PLATFORM_DIR_MOTOR1_PORT);
	platform_enable_clock_gpio(PLATFORM_PWM_MOTOR1_PORT);

	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);
	GPIOInitStruct.GPIO_Pin = PLATFORM_DIR_MOTOR1_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(PLATFORM_DIR_MOTOR1_PORT, &GPIOInitStruct);

	/* Init AF output */
	GPIOInitStruct.GPIO_Pin = PLATFORM_PWM_MOTOR1_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(PLATFORM_PWM_MOTOR1_PORT, &GPIOInitStruct);

	GPIO_PinAFConfig(PLATFORM_PWM_MOTOR1_PORT, PLATFORM_PWM_MOTOR1_PIN_SOURCE, PLATFORM_PWM_MOTOR1_GPIO_AF);

	//useless?
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void platform_motor2_init_io(void)
{
	// Set clocks
	platform_enable_clock_timer(PLATFORM_TIMER_MOTOR2);
	platform_enable_clock_gpio(PLATFORM_DIR_MOTOR2_PORT);
	platform_enable_clock_gpio(PLATFORM_PWM_MOTOR2_PORT);


	/* Init DIR signal */
	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);
	GPIOInitStruct.GPIO_Pin = PLATFORM_DIR_MOTOR2_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(PLATFORM_DIR_MOTOR2_PORT, &GPIOInitStruct);

	/* Init AF output */
	GPIOInitStruct.GPIO_Pin = PLATFORM_PWM_MOTOR2_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(PLATFORM_PWM_MOTOR2_PORT, &GPIOInitStruct);

	GPIO_PinAFConfig(PLATFORM_PWM_MOTOR2_PORT, PLATFORM_PWM_MOTOR2_PIN_SOURCE, PLATFORM_PWM_MOTOR2_GPIO_AF);
}


// Motor config for L298
/*void platform_motor1_init(struct ausbee_l298_chip* motor1) {
	motor1->timer_channel = 1;
	motor1->gpio_enable_pin = PLATFORM_ENABLE_MOTOR1_PIN;
	motor1->gpio_dir_pin = PLATFORM_DIR_MOTOR1_PIN;
	motor1->pwm_frequency = 10000;
	motor1->gpio_dir_port = PLATFORM_DIR_MOTOR1_PORT;
	motor1->gpio_enable_port = PLATFORM_ENABLE_MOTOR1_PORT;
	motor1->TIMx = PLATFORM_TIMER_MOTOR1;
	platform_motor1_init_io();

	enum AUSBEE_L298_DRIVER_ERROR error;
	error = ausbee_l298_init_chip(*motor1);
	if (error!=ENO_ERROR)
		printf("erreur motor1");
	//  platform_led_toggle(PLATFORM_LED2);
	ausbee_l298_enable_chip(*motor1, 1);
}

void platform_motor2_init(struct ausbee_l298_chip* motor2) {
	motor2->timer_channel = 2;
	motor2->gpio_enable_pin = PLATFORM_ENABLE_MOTOR2_PIN;
	motor2->gpio_dir_pin = PLATFORM_DIR_MOTOR2_PIN;
	motor2->pwm_frequency = 10000;
	motor2->gpio_dir_port = PLATFORM_DIR_MOTOR2_PORT;
	motor2->gpio_enable_port = PLATFORM_ENABLE_MOTOR2_PORT;
	motor2->TIMx = PLATFORM_TIMER_MOTOR2;
	platform_motor2_init_io();

	ausbee_l298_init_chip(*motor2);
	//if (error==ENO_ERROR)
	//  platform_led_toggle(PLATFORM_LED2);
	ausbee_l298_enable_chip(*motor2, 1);
}

void platform_motor1_init_io(void) {
	// Set clocks
	platform_enable_clock_timer(PLATFORM_TIMER_MOTOR1);
	platform_enable_clock_gpio(PLATFORM_ENABLE_MOTOR1_PORT);
	platform_enable_clock_gpio(PLATFORM_DIR_MOTOR1_PORT);
	platform_enable_clock_gpio(PLATFORM_PWM_MOTOR1_PORT);

	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);
	GPIOInitStruct.GPIO_Pin = PLATFORM_DIR_MOTOR1_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(PLATFORM_DIR_MOTOR1_PORT, &GPIOInitStruct);

	// Init ENABLE for L298
	GPIOInitStruct.GPIO_Pin = PLATFORM_ENABLE_MOTOR1_PIN;
	GPIO_Init(PLATFORM_ENABLE_MOTOR1_PORT, &GPIOInitStruct);

	// Init AF output
	GPIOInitStruct.GPIO_Pin = PLATFORM_PWM_MOTOR1_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(PLATFORM_PWM_MOTOR1_PORT, &GPIOInitStruct);

	GPIO_PinAFConfig(PLATFORM_PWM_MOTOR1_PORT, PLATFORM_PWM_MOTOR1_PIN_SOURCE, PLATFORM_PWM_MOTOR1_GPIO_AF);
	
	//useless?
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
}

void platform_motor2_init_io(void)
{
	// Set clocks
	platform_enable_clock_timer(PLATFORM_TIMER_MOTOR2);
	platform_enable_clock_gpio(PLATFORM_ENABLE_MOTOR2_PORT);
	platform_enable_clock_gpio(PLATFORM_DIR_MOTOR2_PORT);
	platform_enable_clock_gpio(PLATFORM_PWM_MOTOR2_PORT);


	// Init DIR signal for L298
	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);
	GPIOInitStruct.GPIO_Pin = PLATFORM_DIR_MOTOR2_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(PLATFORM_DIR_MOTOR2_PORT, &GPIOInitStruct);

	// Init ENABLE signal for l298
	GPIOInitStruct.GPIO_Pin = PLATFORM_ENABLE_MOTOR2_PIN;
	GPIO_Init(PLATFORM_ENABLE_MOTOR2_PORT, &GPIOInitStruct);

	// Init AF output
	GPIOInitStruct.GPIO_Pin = PLATFORM_PWM_MOTOR2_PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(PLATFORM_PWM_MOTOR2_PORT, &GPIOInitStruct);

	GPIO_PinAFConfig(PLATFORM_PWM_MOTOR2_PORT, PLATFORM_PWM_MOTOR2_PIN_SOURCE, PLATFORM_PWM_MOTOR2_GPIO_AF);
}*/

void platform_encoder_init(void) {

	// turn on the clocks for each of the ports needed
	platform_enable_clock_gpio(PLATFORM_ENC1A_PORT);
	platform_enable_clock_gpio(PLATFORM_ENC1B_PORT);
	platform_enable_clock_gpio(PLATFORM_ENC2A_PORT);
	platform_enable_clock_gpio(PLATFORM_ENC2B_PORT);

	// Configure the pins
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = PLATFORM_ENC1A_PIN;
	GPIO_Init(PLATFORM_ENC1A_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PLATFORM_ENC1B_PIN;
	GPIO_Init(PLATFORM_ENC1B_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PLATFORM_ENC2A_PIN;
	GPIO_Init(PLATFORM_ENC2A_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PLATFORM_ENC2B_PIN;
	GPIO_Init(PLATFORM_ENC2B_PORT, &GPIO_InitStructure);

	// Connect the pins to their Alternate Functions
	GPIO_PinAFConfig(PLATFORM_ENC1A_PORT, PLATFORM_ENC1A_PIN_SOURCE,
			PLATFORM_ENC1_GPIO_AF);
	GPIO_PinAFConfig(PLATFORM_ENC1B_PORT, PLATFORM_ENC1B_PIN_SOURCE,
			PLATFORM_ENC1_GPIO_AF);
	GPIO_PinAFConfig(PLATFORM_ENC2A_PORT, PLATFORM_ENC2A_PIN_SOURCE,
			PLATFORM_ENC2_GPIO_AF);
	GPIO_PinAFConfig(PLATFORM_ENC2B_PORT, PLATFORM_ENC2B_PIN_SOURCE,
			PLATFORM_ENC2_GPIO_AF);

	// Timer peripheral clock enable
	platform_enable_clock_timer(PLATFORM_ENC1_TIMER);
	platform_enable_clock_timer(PLATFORM_ENC2_TIMER);

	ausbee_encoder_init_timer(PLATFORM_ENC1_TIMER);
	ausbee_encoder_init_timer(PLATFORM_ENC2_TIMER);

}


int platform_can_init(CAN_TypeDef* CANx) {
	GPIO_InitTypeDef init_GPIO_CAN;

	GPIO_StructInit(&init_GPIO_CAN);

	if (CANx == CAN1) {

		//enable the clock to the module
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

		//configure alternate function for the GPIO
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); //CAN_RX
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); //CAN_TX

		//init the GPIO
		init_GPIO_CAN.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
		init_GPIO_CAN.GPIO_Mode = GPIO_Mode_AF;
		init_GPIO_CAN.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOA, &init_GPIO_CAN);
	} else {
		return -1;
	}

	return 0;
}


// enable the clock on the timer in order to be used
void platform_enable_clock_timer(TIM_TypeDef* TIMx) {
	if (TIMx == TIM1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	if (TIMx == TIM2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	if (TIMx == TIM3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	if (TIMx == TIM4)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	if (TIMx == TIM5)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	if (TIMx == TIM6)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	if (TIMx == TIM7)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	if (TIMx == TIM8)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	if (TIMx == TIM9)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	if (TIMx == TIM10)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	if (TIMx == TIM11)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	if (TIMx == TIM12)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	if (TIMx == TIM13)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	if (TIMx == TIM14)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
}

void platform_enable_clock_gpio(GPIO_TypeDef* GPIO_Port) {
	if (GPIO_Port == GPIOA)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	if (GPIO_Port == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	if (GPIO_Port == GPIOC)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	if (GPIO_Port == GPIOD)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	if (GPIO_Port == GPIOE)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	if (GPIO_Port == GPIOF)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	if (GPIO_Port == GPIOG)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	if (GPIO_Port == GPIOH)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	if (GPIO_Port == GPIOI)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

}
