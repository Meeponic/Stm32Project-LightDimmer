/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "lcd.h"


#define PWM_CUTOFF 8

typedef GPIO_TypeDef* Keys_PortTypeDef;
typedef uint32_t Keys_PinTypeDef;


typedef struct{

	Keys_PortTypeDef* KPIN;
	Keys_PinTypeDef* key_pin;
	uint8_t* bouncing;
	uint8_t* bouncing_is_pressed;
	uint8_t* flags_long_press;
	uint8_t* flags_long_press_is_pressed;
	uint8_t* long_press_active;

}TBUTTON;

typedef struct{

	uint8_t time_delay_turn_on;
	uint8_t time_delay_turn_off;
	uint16_t pwm_data;

}time_switch_t;


typedef struct{

	float out;
	float alpha;

}low_pass_filter_t;


typedef enum {
	LED_ON,
	LED_OFF

}led_state_t;


typedef enum {
	BULB_BUTTON_CONTROL,
	BULB_POTENTIOMETER_CONTROL,
	LED_BUTTON_CONTROL,
	LED_POTENTIOMETER_CONTROL

}control_state_t;


typedef void(*callback_t)(time_switch_t* time_pointer, const TBUTTON* btn);


/* Private function prototypes -----------------------------------------------*/
void GPIO_RS_E_PinsConfiguration(void);
void GPIO_4Bits_PinsConfiguration(void);
void Bulb_PinConfiguration(void);
void Led_PinConfiguration(void);

void Timer2Configuration(void);
void Timer3Configuration(void);
bool DelayMs(uint32_t* start_time, uint16_t delay);
void Usart2Configuration(void);
void Buttons_PinsConfiguration(void);
void ADC_PinsConfiguration(void);
void ADC_Configuration(void);
void ADC_Conversion(void);

void DMA_Potentiometers_Configuration(void);
uint16_t ADC_Potentiometer_Bulb_ReadData(void);
uint16_t ADC_Potentiometer_Led_ReadData(void);
void EXTI_PA12_Pins_CrossZeroConfiguration(void);
void EXTI_PA12_CrossZeroConfiguration(void);

void Bulb_ChangeDelayButtonAdd(time_switch_t* time_pointer, const TBUTTON* btn);
void Bulb_ChangeDelayButtonSubtract(time_switch_t* time_pointer, const TBUTTON* btn);
TBUTTON key_press_assign(Keys_PortTypeDef ports[], Keys_PinTypeDef pins[], uint8_t bouncing_flags[],
						uint8_t flags_long_press[], uint8_t long_press_active[]);

TBUTTON is_key_pressed_assign(Keys_PortTypeDef ports[], Keys_PinTypeDef pins[],uint8_t bouncing_is_pressed_flags[],
								uint8_t flags_long_press_is_pressed[]);

void key_press(TBUTTON* btn, uint8_t index, callback_t fun, time_switch_t* time_value);

uint8_t is_key_pressed(TBUTTON* btn, uint8_t index);

float LowPassFilter(low_pass_filter_t filters[], uint8_t index, uint16_t new_data);

void Bulb_LightDimmer(const time_switch_t time_value);
void Bulb_ChangeDelayPotentiometer(time_switch_t* time_pointer, low_pass_filter_t filters[], uint8_t index);
uint8_t Bulb_ChangeButtonPotentiometer(low_pass_filter_t filters[], uint8_t index, const uint8_t is_key_bulb_subtract_pressed,
										const uint8_t is_key_bulb_add_pressed);

void Timer1_PA11_PinConfiguration(void);
void Led_PWM_Timer1Configuration(void);
void Led_ChangePwmButtonAdd(time_switch_t* time_pointer, const TBUTTON* btn);
void Led_ChangePwmButtonSubtract(time_switch_t* time_pointer, const TBUTTON* btn);
void Led_ChangePwmPotentimeter(time_switch_t* time_pointer, low_pass_filter_t filters[], uint8_t index);
uint8_t Led_ChangeButtonPotentiometer(low_pass_filter_t filters[], uint8_t index, const uint8_t is_key_bulb_subtract_pressed,
										const uint8_t is_key_bulb_add_pressed, const uint8_t bulb_potentiometer_control);

void GenerateGammaTable(float gamma);

void SystemClock_Config(void);


uint16_t ADC_Buffer[2];
uint16_t gamma_table[4096];


volatile uint32_t timer2_count = 0;
volatile uint16_t timer2_count2_long_press = 0;
volatile uint8_t timer2_count3_add_subtract_bulb = 0;
volatile uint8_t timer2_count4_button_potentiometer = 0;
volatile uint16_t timer2_count5_long_press_is_pressed = 0;

volatile uint16_t timer3_count_sinus_minus = 0;
volatile uint16_t timer3_count2_sinus_plus = 0;
volatile uint16_t timer3_count3_delay_turn_on1 = 0;
volatile uint16_t timer3_count4_delay_turn_off1 = 0;
volatile uint16_t timer3_count5_delay_turn_on2 = 0;
volatile uint16_t timer3_count6_delay_turn_off2 = 0;

volatile uint16_t exti12_cross_zero = 0;



int main(void)
{

  HAL_Init();
  SystemClock_Config();

  GPIO_RS_E_PinsConfiguration();
  GPIO_4Bits_PinsConfiguration();
  Bulb_PinConfiguration();
  Led_PinConfiguration();
  Timer2Configuration();
  Timer3Configuration();
  Usart2Configuration();
  Buttons_PinsConfiguration();
  ADC_PinsConfiguration();
  ADC_Configuration();
  ADC_Conversion();
  DMA_Potentiometers_Configuration();
  EXTI_PA12_Pins_CrossZeroConfiguration();
  EXTI_PA12_CrossZeroConfiguration();

  Timer1_PA11_PinConfiguration();
  Led_PWM_Timer1Configuration();
  GenerateGammaTable(2.2);


  Lcd_PortType GPIO_Ports[] = {GPIOB,GPIOB,GPIOB,GPIOB};

  Lcd_PinType Pins_set[] = {GPIO_BSRR_BS15, GPIO_BSRR_BS14, GPIO_BSRR_BS13, GPIO_BSRR_BS12};

  Lcd_PinType Pins_reset[] = {GPIO_BSRR_BR15, GPIO_BSRR_BR14, GPIO_BSRR_BR13, GPIO_BSRR_BR12};

  Lcd_ControlTypeDef Control_pin_rs = {GPIOA, GPIO_BSRR_BS9, GPIO_BSRR_BR9};

  Lcd_ControlTypeDef Control_pin_e = {
		  .control_port = GPIOA,
		  .control_pin_set = GPIO_BSRR_BS8,
		  .control_pin_reset = GPIO_BSRR_BR8
  };

  Lcd_HandleTypeDef lcd;

  lcd = Lcd_create(GPIO_Ports, Pins_set, Pins_reset, Control_pin_rs, Control_pin_e, LCD_4_BIT_MODE);

  Keys_PortTypeDef ports[] = {GPIOA,GPIOA,GPIOB,GPIOB};
  Keys_PinTypeDef pins[] = {GPIO_IDR_IDR6, GPIO_IDR_IDR7,GPIO_IDR_IDR0,GPIO_IDR_IDR1};
  uint8_t bouncing_flags[] = {0,0,0,0};
  uint8_t flags_long_press[] = {0,0,0,0};

  uint8_t bouncing_is_pressed_flags[] = {0,0,0,0};
  uint8_t flags_long_press_is_pressed[] = {0,0,0,0};
  uint8_t long_press_active[] = {0,0,0,0};

  TBUTTON button;
  button = key_press_assign(ports, pins, bouncing_flags, flags_long_press, long_press_active);
  button = is_key_pressed_assign(ports, pins, bouncing_is_pressed_flags, flags_long_press_is_pressed);

	time_switch_t time_value = {

		.time_delay_turn_on = 90,
		.time_delay_turn_off = 0,
		.pwm_data = 0

	};

	low_pass_filter_t filters[4] = {

			{.out = 0.0f, .alpha = 0.01f},
			{.out = 0.0f, .alpha = 0.01f},
			{.out = 0.0f, .alpha = 0.01f},
			{.out = 0.0f, .alpha = 0.01f}

	};


  uint32_t time1 = 0;
//  uint32_t time2 = 0;

  uint8_t is_key_bulb_subtract_pressed = 0;
  uint8_t is_key_bulb_add_pressed = 0;
  uint8_t bulb_potentiometer_control = 0;
  control_state_t bulb_control_state = BULB_POTENTIOMETER_CONTROL;

  uint8_t is_key_led_subtract_pressed = 0;
  uint8_t is_key_led_add_pressed = 0;
  uint8_t led_potentiometer_control = 0;
  control_state_t led_control_state = LED_POTENTIOMETER_CONTROL;

  Lcd_cursor(&lcd, 0, 1);
  Lcd_string(&lcd, "led i zarowka");
  Lcd_cursor(&lcd, 1, 3);
  Lcd_string(&lcd, "sterowanie");

  while (1)
  {


	  Bulb_LightDimmer(time_value);

	  is_key_bulb_subtract_pressed = is_key_pressed(&button, 0);
	  is_key_bulb_add_pressed = is_key_pressed(&button, 1);

	  if (is_key_bulb_add_pressed || is_key_bulb_subtract_pressed )
		  bulb_control_state = BULB_BUTTON_CONTROL;


	  bulb_potentiometer_control = Bulb_ChangeButtonPotentiometer(filters ,0, is_key_bulb_subtract_pressed,
																  	  is_key_bulb_add_pressed);
	  if (bulb_potentiometer_control)
		  bulb_control_state = BULB_POTENTIOMETER_CONTROL;


	  if (bulb_control_state == BULB_BUTTON_CONTROL){

		  key_press(&button, 0, Bulb_ChangeDelayButtonSubtract, &time_value);
		  key_press(&button, 1, Bulb_ChangeDelayButtonAdd, &time_value);
	  }

	  if (bulb_control_state == BULB_POTENTIOMETER_CONTROL)
		  Bulb_ChangeDelayPotentiometer(&time_value, filters, 1);


	  Bulb_LightDimmer(time_value);


// Led control

	  is_key_led_subtract_pressed  = is_key_pressed(&button, 2);
	  is_key_led_add_pressed = is_key_pressed(&button, 3);

	  if(is_key_led_subtract_pressed || is_key_led_add_pressed )
		  led_control_state = LED_BUTTON_CONTROL;


	  led_potentiometer_control = Led_ChangeButtonPotentiometer(filters, 2, is_key_bulb_subtract_pressed,
			  	  	  	  	  	  	  	  	  	  	  	  	  	  is_key_bulb_add_pressed, bulb_potentiometer_control);

	  if(led_potentiometer_control)
		  led_control_state = LED_POTENTIOMETER_CONTROL;


	  if (led_control_state == LED_BUTTON_CONTROL){

		  key_press(&button, 2, Led_ChangePwmButtonSubtract, &time_value);
		  key_press(&button, 3, Led_ChangePwmButtonAdd, &time_value);
	  }


	  if (led_control_state == LED_POTENTIOMETER_CONTROL)
		  Led_ChangePwmPotentimeter(&time_value, filters, 3);

	  Bulb_LightDimmer(time_value);




	  if (DelayMs(&time1, 1000)){
//
//				printf("potentiometer_żarówka:= %hu\r\n",potentiometer_bulb_read_data );
//				printf("potentiometer_led:= %hu\r\n",potentiometer_led_read_data );
//				printf("timer_delay:%lu\r\n", timer3_count);
//				printf("exti12_cross_zero:=%hu\r\n", exti12_cross_zero);
//				printf("time_delay_turn_on:= %u\r\n", time_value.time_delay_turn_on);
//				printf("time_delay_turn_off:= %u\r\n", time_value.time_delay_turn_off);
//				printf("potentiometer_bulb_mean:= %hu \r\n", potentiometer_bulb_mean);
//				printf ("potentiometer_bulb_scale_delay_turn_on:= %.2f \r\n", potentiometer_bulb_scale_delay_turn_on);
//				printf ("potentiometer_bulb_scale_delay_turn_off %.2f \r\n", potentiometer_bulb_scale_delay_turn_off);
//				printf ("time_delay_turn_on:= %u \r\n", time_delay_turn_on);
//				printf ("time_delay_turn_off:= %u \r\n", time_delay_turn_off);
//				printf ("filter_out:= %.2f \r\n", filter_out);
//		  	 	printf("start_adc_data:= %u \r\n", start_adc_data);
//		  	  	printf("end_adc_data:= %u \r\n", end_adc_data);
//		  	  	  printf("increment:= %u \r\n", increment);
//		  	  	  printf("control_state:= %d \r\n", control_state);
//		  	  	 for(int i = 0; i < 5; i++){
//		  	  	  printf ("start_adc_array[%d]:= %hu \r\n", i, start_adc_array[i]);
//		  	  	 }

//		  	 	 printf("adc_read_data:= %hu \r\n", time_value.pwm_data);
//		  	 	 printf ("adc_read_gamma:= %hu \r\n", gamma_table[time_value.pwm_data]);

//
	  }


  }

}


void TIM2_IRQHandler(void){

	if (TIM2->SR & TIM_SR_UIF){

		TIM2->SR &= ~TIM_SR_UIF;
		timer2_count++;

		if (timer2_count2_long_press > 0)
			timer2_count2_long_press--;

		if (timer2_count3_add_subtract_bulb > 0)
			timer2_count3_add_subtract_bulb--;

		if (timer2_count4_button_potentiometer > 0)
			timer2_count4_button_potentiometer--;

		if (timer2_count5_long_press_is_pressed > 0)
			timer2_count5_long_press_is_pressed--;
	}

}


void TIM3_IRQHandler(void){

	if(TIM3->SR & TIM_SR_UIF){

		TIM3->SR &= ~TIM_SR_UIF;

		if (timer3_count_sinus_minus > 0)
			timer3_count_sinus_minus--;

		if (timer3_count2_sinus_plus > 0)
			timer3_count2_sinus_plus--;

		if (timer3_count3_delay_turn_on1 > 0)
			timer3_count3_delay_turn_on1--;

		if (timer3_count4_delay_turn_off1 > 0)
			timer3_count4_delay_turn_off1--;

		if (timer3_count5_delay_turn_on2 > 0)
			timer3_count5_delay_turn_on2--;

		if (timer3_count6_delay_turn_off2 > 0)
			timer3_count6_delay_turn_off2--;


	}

}


void EXTI15_10_IRQHandler(void){

	if(EXTI->PR & EXTI_PR_PR12){

		EXTI->PR |= EXTI_PR_PR12;

			exti12_cross_zero = 1;

	}

}


int __io_putchar(int ch) {

	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR = (ch & 0xFF);
	return ch;

}

void Timer2Configuration(void){

	//Enable clock for Timer2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->PSC = 799;
	TIM2->ARR = 9;
	// Enable update interrupt
	TIM2->DIER |= TIM_DIER_UIE;

	//Enable timer2 counter
	TIM2->CR1 |= TIM_CR1_CEN;

	//Set interrupt priority
	NVIC_SetPriority(TIM2_IRQn, 1);
	//Enable global NVIC interrupt
	NVIC_EnableIRQ(TIM2_IRQn);



}

void Timer3Configuration(void){

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Timer 10kHz, 0.0001
	TIM3->PSC = 79;
	TIM3->ARR = 9;

	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_CEN;

	NVIC_SetPriority(TIM3_IRQn, 0);
	NVIC_EnableIRQ(TIM3_IRQn);

}


bool DelayMs(uint32_t* start_time, uint16_t delay){

	if (*start_time == 0){

		*start_time = timer2_count;
	}

	if (timer2_count - *start_time >= delay){

		*start_time = timer2_count;
		return true;

	}

	return false;
}

void Usart2Configuration(void){

	//Enable clock for PA2
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	//Configure PA2 as USART2 TX, alternate function push-pull
	GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
	GPIOA->CRL |= GPIO_CRL_CNF2_1;
	//Configure output speed as 2MHZ
	GPIOA->CRL &= ~GPIO_CRL_MODE2_0;
	GPIOA->CRL |= GPIO_CRL_MODE2_1;

	//Enable clock for USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	//USART2 baud rate
	USART2->BRR = 0x341;
	//Word length 1 start bit, 8 data bit
	USART2->CR1 &= ~USART_CR1_M;
	//Disable parity control
	USART2->CR1 &= ~USART_CR1_PCE;
	//Stop bit 1
	USART2->CR2 &= ~USART_CR2_STOP;
	//Enable transmission
	USART2->CR1 |= USART_CR1_TE;
	//Enable USART
	USART2->CR1 |= USART_CR1_UE;

}


void GPIO_RS_E_PinsConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	//PA9 configuration Rs pin, as output push-pull
	GPIOA->CRH &= ~GPIO_CRH_CNF9;
	GPIOA->CRH &= ~GPIO_CRH_MODE9_0;
	GPIOA->CRH |= GPIO_CRH_MODE9_1;

	//PA8 configuration E pin, as output push-pull
	GPIOA->CRH &= ~GPIO_CRH_CNF8;
	GPIOA->CRH &= ~GPIO_CRH_MODE8_0;
	GPIOA->CRH |= GPIO_CRH_MODE8_1;


}

void GPIO_4Bits_PinsConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	//PB15 configuration D4 on LCD
	GPIOB->CRH &= ~GPIO_CRH_CNF15;
	GPIOB->CRH &= ~GPIO_CRH_MODE15_0;
	GPIOB->CRH |= GPIO_CRH_MODE15_1;

	//PB14 configuration D5 on LCD
	GPIOB->CRH &= ~GPIO_CRH_CNF14;
	GPIOB->CRH &= ~GPIO_CRH_MODE14_0;
	GPIOB->CRH |= GPIO_CRH_MODE14_0;

	//PB13 configuration D6 on LCD
	GPIOB->CRH &= ~GPIO_CRH_CNF13;
	GPIOB->CRH &= ~GPIO_CRH_MODE13_0;
	GPIOB->CRH |= GPIO_CRH_MODE13_1;

	//PB12 configuration D7 on LCD
	GPIOB->CRH &= ~GPIO_CRH_CNF12;
	GPIOB->CRH &= ~GPIO_CRH_MODE12_0;
	GPIOB->CRH |= GPIO_CRH_MODE12_1;

}

void Bulb_PinConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRH &= ~GPIO_CRH_CNF10;
	GPIOA->CRH &= ~GPIO_CRH_MODE10_0;
	GPIOA->CRH |= GPIO_CRH_MODE10_1;

}


void Led_PinConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRH &= ~GPIO_CRH_CNF11;
	GPIOA->CRH &= ~GPIO_CRH_MODE11_0;
	GPIOA->CRH |= GPIO_CRH_MODE11_1;

}
void Buttons_PinsConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	//Configure PB0 as input pull up
	GPIOB->CRL &= ~GPIO_CRL_CNF0_0;
	GPIOB->CRL |= GPIO_CRL_CNF0_1;
	GPIOB->CRL &= ~GPIO_CRL_MODE0;
	GPIOB->ODR |= GPIO_ODR_ODR0;

	//Configure PB1 as input pull up
	GPIOB->CRL &= ~GPIO_CRL_CNF1_0;
	GPIOB->CRL |= GPIO_CRL_CNF1_1;
	GPIOB->CRL &= ~GPIO_CRL_MODE1;
	GPIOB->ODR |= GPIO_ODR_ODR1;

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	//Configure PA6 as input pull up
	GPIOA->CRL &= ~GPIO_CRL_CNF6_0;
	GPIOA->CRL |= GPIO_CRL_CNF6_1;
	GPIOA->CRL &= ~GPIO_CRL_MODE6;
	GPIOA->ODR |= GPIO_ODR_ODR6;

	//Configure PA7 as input pull up
	GPIOA->CRL &= ~GPIO_CRL_CNF7_0;
	GPIOA->CRL |= GPIO_CRL_CNF7_1;
	GPIOA->CRL &= ~GPIO_CRL_MODE7;
	GPIOA->ODR |= GPIO_ODR_ODR7;

}

void ADC_PinsConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	//Configuration PA4 as input analog
	GPIOA->CRL &= ~GPIO_CRL_CNF4;
	GPIOA->CRL &= ~GPIO_CRL_MODE4;

	//Configuration PA5 as input analog
	GPIOA->CRL &= ~GPIO_CRL_CNF5;
	GPIOA->CRL &= ~GPIO_CRL_MODE5;

}

void ADC_Configuration(void){

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	//Enable AD converter 1st call just enable, second when ADON == 1 and we write again 1 will start converting
	ADC1->CR2 |= ADC_CR2_ADON;

	// Enable calibration and wait till completed
	ADC1->CR2 |= ADC_CR2_CAL;
	while (ADC1->CR2 & ADC_CR2_CAL);

	//Enable scan mode
	ADC1->CR1 |= ADC_CR1_SCAN;

	//Enable continuous mode
	ADC1->CR2 |= ADC_CR2_CONT;

	//Enable DMA
	ADC1->CR2 |= ADC_CR2_DMA;

	//Numbers of regular channels to be converted, 2 channels
	ADC1->SQR1 &= ~ADC_SQR1_L;
	ADC1->SQR1 |= ADC_SQR1_L_0;

	//Channel order sequence 1 channel 4, sequence 2 channel 5
	ADC1->SQR3 = (4 << 0) | (5 << 5);

	// Setting sampling time 111 to 239.5 cycles max, capacitors have time to charge
	ADC1->SMPR2 |= ADC_SMPR2_SMP4;
	ADC1->SMPR2 |= ADC_SMPR2_SMP5;

}

void ADC_Conversion(void){

	ADC1->CR2 |= ADC_CR2_ADON;

}


void DMA_Potentiometers_Configuration(void){

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	//Disable DMA for configuration
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;

	//Set peripheral address
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;

	//Set memory address
	DMA1_Channel1->CMAR = (uint32_t)ADC_Buffer;

	//Number of half-words to transfer (2 conversions), array[length] length = CNDTR
	DMA1_Channel1->CNDTR = 2;

	//Memory increment mode, increment n in array after each read
	DMA1_Channel1->CCR |= DMA_CCR_MINC;

	//16 bit peripheral size
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;

	//16 bit memory size
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;

	//Enable circular mode
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;

	//Enable DMA1_Channel1
	DMA1_Channel1->CCR |= DMA_CCR_EN;

}

uint16_t ADC_Potentiometer_Bulb_ReadData(void){

	return ADC_Buffer[0];

}
uint16_t ADC_Potentiometer_Led_ReadData(void){

	return ADC_Buffer[1];

}

void EXTI_PA12_Pins_CrossZeroConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRH &= ~GPIO_CRH_CNF12_0;
	GPIOA->CRH |= GPIO_CRH_CNF12_1;
	GPIOA->CRH &= ~GPIO_CRH_MODE12;
	GPIOA->ODR |= GPIO_ODR_ODR12;

	AFIO->EXTICR[3] &= ~AFIO_EXTICR4_EXTI12;

}

void EXTI_PA12_CrossZeroConfiguration(void){

	//Interrupt request not masked
	EXTI->IMR |= EXTI_IMR_MR12;

	//Rising edge trigger
	EXTI->RTSR |= EXTI_RTSR_TR12;

//	//Falling edge trigger
//	EXTI->FTSR |= EXTI_FTSR_TR12;

	NVIC_SetPriority(EXTI15_10_IRQn, 2);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}


void Bulb_ChangeDelayButtonAdd(time_switch_t* time_pointer, const TBUTTON* btn){


	if (btn->long_press_active[1] == 0){

		if (time_pointer->time_delay_turn_on > 0 && time_pointer->time_delay_turn_on <= 90)
			time_pointer->time_delay_turn_on -= 20;

		if (time_pointer->time_delay_turn_off >= 0 && time_pointer->time_delay_turn_off < 90)
			time_pointer->time_delay_turn_off += 20;

		if (time_pointer->time_delay_turn_on > 200)
			time_pointer->time_delay_turn_on = 0;

		if (time_pointer->time_delay_turn_off  >= 90)
			time_pointer->time_delay_turn_off = 90;

	}


	else if (btn->long_press_active[1] == 1){

		if (time_pointer->time_delay_turn_on > 0 && time_pointer->time_delay_turn_on <= 90)
			time_pointer->time_delay_turn_on -= 1;

		if (time_pointer->time_delay_turn_off >= 0 && time_pointer->time_delay_turn_off < 90)
			time_pointer->time_delay_turn_off += 1;

	}

}


void Bulb_ChangeDelayButtonSubtract(time_switch_t* time_pointer, const TBUTTON* btn){


	if (btn->long_press_active[0]== 0){

		if (time_pointer->time_delay_turn_on >= 0 && time_pointer->time_delay_turn_on < 90)
			time_pointer->time_delay_turn_on += 20;

		if (time_pointer->time_delay_turn_off > 0 && time_pointer->time_delay_turn_off <= 90)
			time_pointer->time_delay_turn_off-= 20;

		if (time_pointer->time_delay_turn_on >= 90)
			time_pointer->time_delay_turn_on = 90;

		if (time_pointer->time_delay_turn_off > 200)
			time_pointer->time_delay_turn_off = 0;

	}


	else if (btn->long_press_active[0] == 1){

		if (time_pointer->time_delay_turn_on >= 0 && time_pointer->time_delay_turn_on < 90)
			time_pointer->time_delay_turn_on += 1;

		if (time_pointer->time_delay_turn_off > 0 && time_pointer->time_delay_turn_off <= 90)
			time_pointer->time_delay_turn_off -= 1;

	}

}


TBUTTON key_press_assign(Keys_PortTypeDef ports[], Keys_PinTypeDef pins[], uint8_t bouncing_flags[],
							uint8_t flags_long_press[], uint8_t long_press_active[]){

	TBUTTON button;
	button.KPIN = ports;
	button.key_pin = pins;
	button.bouncing = bouncing_flags;
	button.flags_long_press = flags_long_press;
	button.long_press_active = long_press_active;
	return button;
}

TBUTTON is_key_pressed_assign(Keys_PortTypeDef ports[], Keys_PinTypeDef pins[],uint8_t bouncing_is_pressed_flags[],
								uint8_t flags_long_press_is_pressed[]){

	TBUTTON button;
	button.KPIN = ports;
	button.key_pin = pins;
	button.bouncing_is_pressed = bouncing_is_pressed_flags;
	button.flags_long_press_is_pressed = flags_long_press_is_pressed;
	return button;
}



void key_press(TBUTTON* btn, uint8_t index, callback_t fun, time_switch_t* time_value){

	Keys_PortTypeDef KPIN = btn->KPIN[index];
	Keys_PinTypeDef key_pin = btn->key_pin[index];

	uint8_t key_pressed = KPIN->IDR & key_pin;

	if (!btn->bouncing[index] && !key_pressed){

		btn->bouncing[index] = 1;
		btn->flags_long_press[index] = 1;
		btn->long_press_active[index] = 0;
		timer2_count2_long_press = 750;

		if (fun)
			(fun)(time_value, btn);
	}

	else if (btn->bouncing[index] && key_pressed){

		(btn->bouncing[index])++;

		if(!btn->bouncing[index]){
			btn->bouncing[index] = 0;
			btn->flags_long_press[index] = 0;
			btn->long_press_active[index] = 0;
			timer2_count2_long_press = 0;
		}

	}

	else if (!timer2_count2_long_press && btn->flags_long_press[index]){

		if (fun && !timer2_count3_add_subtract_bulb){

			btn->long_press_active[index] = 1;
			fun(time_value, btn);
			timer2_count3_add_subtract_bulb = 30;
		}
	}

}

uint8_t is_key_pressed(TBUTTON* btn, uint8_t index){

	Keys_PortTypeDef KPIN = btn->KPIN[index];
	Keys_PinTypeDef key_pin = btn->key_pin[index];
	uint8_t* bouncing_is_pressed =&btn->bouncing_is_pressed[index];

	uint8_t key_pressed = KPIN->IDR & key_pin;

	if(!*bouncing_is_pressed && !key_pressed){

		*bouncing_is_pressed  = 1;
		btn->flags_long_press_is_pressed[index] = 1;
		timer2_count5_long_press_is_pressed = 750;
		return 1;

	}

	else if (*bouncing_is_pressed && key_pressed){

		(*bouncing_is_pressed)++;

		if(!*bouncing_is_pressed){

			*bouncing_is_pressed = 0;
			btn->flags_long_press_is_pressed[index] = 0;
			timer2_count5_long_press_is_pressed = 0;
		}
	}

	else if(!timer2_count5_long_press_is_pressed && btn->flags_long_press_is_pressed[index]){

		return 1;
	}

	return 0;

}


float LowPassFilter(low_pass_filter_t filters[], uint8_t index, uint16_t new_data){

	filters[index].out = filters[index].alpha * new_data + (1.0 - filters[index].alpha) * filters[index].out;

	return filters[index].out;
}




void Bulb_LightDimmer(const time_switch_t time_value){

	 static uint8_t turn_on_bulb1 = 0;
	 static uint8_t turn_on_bulb2 = 0;
	 static uint8_t marker1_sinus_minus = 0;
	 static uint8_t marker2_turn_on1 = 0;
	 static uint8_t marker3_sinsu_plus = 0;
	 static uint8_t marker4_turn_on2 = 0;
	 static uint8_t set_cross_zero_sinus_minus = 0;
	 static uint8_t set_cross_zero_sinus_plus = 0;
	 static uint8_t enable_cross_zero = 0;


		  if ((time_value.time_delay_turn_on > 0 && time_value.time_delay_turn_on < 90) ||
			  (time_value.time_delay_turn_off > 0 && time_value.time_delay_turn_off < 90)){
			  enable_cross_zero = 1;
		  }

		  else if (time_value.time_delay_turn_on == 0 && time_value.time_delay_turn_off == 90){

			  enable_cross_zero = 0;
			  GPIOA->BSRR = GPIO_BSRR_BS10;
		  }


		  if (enable_cross_zero){

			  if (exti12_cross_zero){

				  exti12_cross_zero = 0;
				  timer3_count_sinus_minus = 3;
				  marker1_sinus_minus = 1;
			  }

			  if(!timer3_count_sinus_minus && marker1_sinus_minus){

				  set_cross_zero_sinus_minus = 1;
				  marker1_sinus_minus = 0;
			  }

			  if (set_cross_zero_sinus_minus){

				  set_cross_zero_sinus_minus = 0;
				  timer3_count2_sinus_plus = 100;
				  timer3_count3_delay_turn_on1 = time_value.time_delay_turn_on;
				  marker2_turn_on1 = 1;
				  marker3_sinsu_plus = 1;
			  }

			  //First turn on
			  if (!timer3_count3_delay_turn_on1 && marker2_turn_on1 ){

				  GPIOA->BSRR = GPIO_BSRR_BS10;
				  timer3_count4_delay_turn_off1 = time_value.time_delay_turn_off;
				  turn_on_bulb1 = 1;
				  marker2_turn_on1 = 0;
			  }

			  if (!timer3_count4_delay_turn_off1 && turn_on_bulb1){

				  GPIOA->BSRR = GPIO_BSRR_BR10;
				  turn_on_bulb1 = 0;
			  }


			  if (!timer3_count2_sinus_plus && marker3_sinsu_plus){

				  set_cross_zero_sinus_plus = 1;
				  marker3_sinsu_plus = 0;
			  }


			  if (set_cross_zero_sinus_plus){

				  set_cross_zero_sinus_plus = 0;
				  timer3_count5_delay_turn_on2 = time_value.time_delay_turn_on;
				  marker4_turn_on2 = 1;
			  }

			  // Second turn on
			  if (!timer3_count5_delay_turn_on2 && marker4_turn_on2){

				  GPIOA->BSRR = GPIO_BSRR_BS10;
				  timer3_count6_delay_turn_off2 = time_value.time_delay_turn_off;
				  turn_on_bulb2 = 1;
				  marker4_turn_on2 = 0;
			  }


			  if (!timer3_count6_delay_turn_off2 && turn_on_bulb2){

				  GPIOA->BSRR = GPIO_BSRR_BR10;
				  turn_on_bulb2 = 0;
				  }

		  }


}

void Bulb_ChangeDelayPotentiometer(time_switch_t* time_pointer, low_pass_filter_t filters[], uint8_t index){


		static uint16_t potentiometer_bulb_read_data = 0;
		static float potentiometer_bulb_scale_delay_turn_on = 0;
		static float filter_out = 0;


		potentiometer_bulb_read_data = ADC_Potentiometer_Bulb_ReadData();
		filter_out = LowPassFilter(filters, index, potentiometer_bulb_read_data);

		potentiometer_bulb_scale_delay_turn_on = (float) ((90.0 * filter_out)/4096.0);


		time_pointer->time_delay_turn_on = potentiometer_bulb_scale_delay_turn_on;
		time_pointer->time_delay_turn_off = (90 - time_pointer->time_delay_turn_on);


		if (time_pointer->time_delay_turn_off < 2 ){

			time_pointer->time_delay_turn_off = 0;
		}

		if ( time_pointer->time_delay_turn_on > 88){

			time_pointer->time_delay_turn_on = 90;
		}


}

uint8_t Bulb_ChangeButtonPotentiometer(low_pass_filter_t filters[], uint8_t index, const uint8_t is_key_bulb_subtract_pressed,
									const uint8_t is_key_bulb_add_pressed){


	static uint8_t initialize_array = 0;
	static uint8_t marker1_write_to_array = 0;
	static uint8_t can_calculate_mean = 0;

	static uint16_t ADC_Potentiometer_Bulb_Start_Data = 0;
	static float filter_start_out = 0;
	static uint16_t ADC_Potentiometer_Bulb_Start_Filtered = 0;
	static uint16_t start_adc_array[5];

	static uint16_t start_adc_data = 0;
	static uint16_t last_adc_data = 0;

	uint16_t sum_start_adc = 0;


	if (!initialize_array){

		for(int i = 0; i < 5; i++){
			start_adc_array[i] = 0;

		}
		initialize_array = 1;
	}

	if (!marker1_write_to_array){

		static uint8_t i = 0;

		ADC_Potentiometer_Bulb_Start_Data = ADC_Potentiometer_Bulb_ReadData();
		filter_start_out = LowPassFilter(filters, index, ADC_Potentiometer_Bulb_Start_Data);
		ADC_Potentiometer_Bulb_Start_Filtered = filter_start_out;
		start_adc_array[i] = ADC_Potentiometer_Bulb_Start_Filtered;

		if(i < 4)
			i++;
		else{
			i = 0;
			marker1_write_to_array = 1;
			can_calculate_mean = 1;
		}
	}

	if (can_calculate_mean){

		can_calculate_mean = 0;
		marker1_write_to_array = 0;
		sum_start_adc = 0;

		for (int i = 0; i < 5; i++){

			sum_start_adc += start_adc_array[i];
		}

		start_adc_data = sum_start_adc/5;
	}


	if (is_key_bulb_subtract_pressed || is_key_bulb_add_pressed)
		last_adc_data = start_adc_data;

	if (start_adc_data - last_adc_data > 120 || start_adc_data - last_adc_data < -120){
		last_adc_data = start_adc_data;
		return 1;
	}

	else
		return 0;

}


void Timer1_PA11_PinConfiguration(void){

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	//Alternate function push-pull
	GPIOA->CRH &= ~GPIO_CRH_CNF11_0;
	GPIOA->CRH |= GPIO_CRH_CNF11_1;

	// Output speed 2Mhz
	GPIOA->CRH &= ~GPIO_CRH_MODE11_0;
	GPIOA->CRH |= GPIO_CRH_MODE11_1;

}

void Led_PWM_Timer1Configuration(void){

	// Enable clock for timer 1 in RCC
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->PSC = 0;
	TIM1->ARR = 4095;

	// Channel as output
	TIM1->CCMR2 &= ~TIM_CCMR2_CC4S;

	// PWM mode 1 110
	TIM1->CCMR2 &= ~TIM_CCMR2_OC4M_0;
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

	// Preload register on TIMx_CCR1 enabled TIMx_CCR1 can be written only on event update
	TIM1->CCMR2 |= TIM_CCMR2_OC4PE;

	//	Direction 0, up counting
	TIM1->CR1 &= ~TIM_CR1_DIR;

	// Polarity active high
	TIM1->CCER &= ~TIM_CCER_CC4P;
	// Enable OC2 signal as output
	TIM1->CCER |= TIM_CCER_CC4E;

	// MOE main output enable needed for TIM1/TIM8 advanced timers
	TIM1->BDTR |= TIM_BDTR_MOE;

	// Reseting update interrupt flag
	TIM1->SR &= ~TIM_SR_UIF;

	// Tim1 counter enabled
	TIM1->CR1 |= TIM_CR1_CEN;

}


void Led_ChangePwmButtonAdd(time_switch_t* time_pointer, const TBUTTON* btn){


	if (btn->long_press_active[3] == 0){

		if (time_pointer->pwm_data < 4095){

			time_pointer->pwm_data += 820;
		}

		if (time_pointer->pwm_data > 4095){

			uint16_t store_subtraction = 0;
			store_subtraction = 4095 - time_pointer->pwm_data;
			time_pointer->pwm_data += store_subtraction;

		}

	}

	else if (btn->long_press_active[3] == 1){

		if (time_pointer->pwm_data < 4095){

			time_pointer->pwm_data += 40;
		}

		if (time_pointer->pwm_data > 4095){

			uint16_t store_subtraction = 0;
			store_subtraction = 4095 - time_pointer->pwm_data;
			time_pointer->pwm_data += store_subtraction;
		}

	}

	TIM1->CCR4 = gamma_table[time_pointer->pwm_data];

}


void Led_ChangePwmButtonSubtract(time_switch_t* time_pointer, const TBUTTON* btn){


	if (btn->long_press_active[2] == 0){

		int16_t store_subtraction_minus = 0;
		store_subtraction_minus = time_pointer->pwm_data  - 820;

		if (store_subtraction_minus < 0)
			time_pointer->pwm_data  = 0;

		else
			time_pointer->pwm_data -= 820;

		}

	else if (btn->long_press_active[2] == 1){

		int16_t store_subtraction_minus = 0;
		store_subtraction_minus = time_pointer->pwm_data  - 40;

		if (store_subtraction_minus < 0)
			time_pointer->pwm_data  = 0;

		else
			time_pointer->pwm_data -= 40;

	}

	TIM1->CCR4 = gamma_table[time_pointer->pwm_data];

}

void Led_ChangePwmPotentimeter(time_switch_t* time_pointer, low_pass_filter_t filters[], uint8_t index){


	static uint16_t potentiometer_led_read_data = 0;
	static float filter_out = 0;


	potentiometer_led_read_data = ADC_Potentiometer_Led_ReadData();
	filter_out = LowPassFilter(filters, index, potentiometer_led_read_data);

	time_pointer->pwm_data = filter_out;

	if (gamma_table[time_pointer->pwm_data] < PWM_CUTOFF)
		gamma_table[time_pointer->pwm_data] = 0;

	TIM1->CCR4 = gamma_table[time_pointer->pwm_data];

}


uint8_t Led_ChangeButtonPotentiometer(low_pass_filter_t filters[], uint8_t index, const uint8_t is_key_bulb_subtract_pressed,
										const uint8_t is_key_bulb_add_pressed, const uint8_t bulb_potentiometer_control){


	static uint8_t marker1_write_to_array = 0;
	static uint8_t can_calculate_mean = 0;

	static uint16_t ADC_Potentiometer_Led_Start_Data = 0;
	static float filter_start_out = 0;
	static uint16_t ADC_Potentiometer_Led_Start_Filtered = 0;
	static uint16_t start_adc_array[5] = {0};

	static uint16_t start_adc_data = 0;
	static uint16_t last_adc_data = 0;

	uint16_t sum_start_adc = 0;


	if(!marker1_write_to_array){

		static uint8_t i = 0;
		ADC_Potentiometer_Led_Start_Data = ADC_Potentiometer_Led_ReadData();
		filter_start_out = LowPassFilter(filters, index, ADC_Potentiometer_Led_Start_Data);
		ADC_Potentiometer_Led_Start_Filtered = filter_start_out;
		start_adc_array[i] = ADC_Potentiometer_Led_Start_Filtered;

		if (i < 4)
			i++;

		else {
			i = 0;
			marker1_write_to_array = 1;
			can_calculate_mean = 1;
		}
	}

	if(can_calculate_mean){

		can_calculate_mean = 0;
		marker1_write_to_array = 0;
		sum_start_adc = 0;

		for(int i = 0; i < 5; i++){
			sum_start_adc += start_adc_array[i];
		}

		start_adc_data = sum_start_adc/5;
	}


	if (is_key_bulb_subtract_pressed || is_key_bulb_add_pressed || bulb_potentiometer_control)
		last_adc_data = start_adc_data;

	if (start_adc_data - last_adc_data > 120 || start_adc_data - last_adc_data < -120){
		last_adc_data = start_adc_data;
		return 1;
	}

	else
		return 0;


}


void GenerateGammaTable(float gamma){

	for (int i = 0; i < 4096; i++){

		float normalized = (float) i / 4095.0f;
		gamma_table[i] = (uint16_t) (powf(normalized, gamma) * 4095.0f);

	}

}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
