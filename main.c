#include "stm32f10x.h"
#include "i2c_lcd.h"
#include <stdint.h>
#include <stdio.h>
#include "time.h"


#define SDA_0 GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define SDA_1 GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define SCL_0 GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define SCL_1 GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define SDA_VAL (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))


static uint32_t u32count = 2;
static uint32_t u32SetHum = 90;
static uint32_t u32SetTemp = 40;
static uint8_t u8SetMinute;
static uint8_t u8SetHour;
static uint8_t u8Second;
static uint8_t u8Minute;
static uint8_t u8Hour;
static uint8_t u8Day;
static uint8_t u8Month;
static uint8_t u8Year;


void Delay1Ms(void);
void Delay_Ms(uint32_t u32DelayInMs);
void delay_us(uint32_t delay);
void UART3_Puts(uint8_t *uartu8Data);
void Rtc_Configuration(void);
void RTC_IRQHandler(void);
void dht11_init(void);
uint8_t dht11_read(uint8_t *pu8Data);
void EXTI_Configuration(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);


void Rtc_Configuration(void){
	/* Set bit PWREN and BKPEN */
	RCC->APB1ENR |= (1 << 28) | (1 << 27);
	/*Set bit DBP */
	PWR->CR |= (1 << 8);
	/* Enable clock 32.768khz */
	RCC->BDCR |= (1 << 0);
	/* Wait clock */
	while (!(RCC->BDCR & (1 << 1))){
	}

	RCC->BDCR &= ~(3 << 8);
	RCC->BDCR |= (1 << 8);

	RCC->BDCR |= (1 << 15);

	/* Config RTC */
	while (!(RTC->CRL & (1 << 5))){
	}
	RTC->CRL |= (1 << 4);

        RTC->PRLH = 0x0000;
        RTC->PRLL = 0x7fff;

	RTC->CRL &= ~(1 << 4);
	while (!(RTC->CRL & (1 << 5))){
	}

	/* Config interrupt */
	 RTC->CRH |= (1 << 0);
	 NVIC_InitTypeDef   nvicInit;
	 nvicInit.NVIC_IRQChannel = RTC_IRQn;
	 nvicInit.NVIC_IRQChannelCmd = ENABLE;
	 nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
	 nvicInit.NVIC_IRQChannelSubPriority = 0;
	 NVIC_Init(&nvicInit);

}

void RTC_IRQHandler(void){
	if (RTC->CRL & (1 << 0)){
		u8Second++;
		RTC->CRL &= ~(1 << 0);
	}
}
void dht11_init(void)
{
	GPIO_InitTypeDef gpioInit;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	gpioInit.GPIO_Mode = GPIO_Mode_Out_OD;
	gpioInit.GPIO_Pin = GPIO_Pin_8;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInit);

	GPIO_SetBits(GPIOB, GPIO_Pin_8);
}

uint8_t dht11_read(uint8_t *pu8Data)
{
	uint16_t u16Tim;
	uint8_t u8Buff[5];
	uint8_t u8CheckSum;
	uint8_t i;

	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	Delay_Ms(20);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);

	/* Wait for pin PB12 to go high */
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 10) {
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
			break;
		}
	}
	u16Tim = TIM_GetCounter(TIM2);
	if (u16Tim >= 10) {
		return 0;
	}

	/* Wait for pin PB12 to go low */
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 45) {
		if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
			break;
		}
	}
	u16Tim = TIM_GetCounter(TIM2);
	if ((u16Tim >= 45) || (u16Tim <= 5)) {
		return 0;
	}

	/* Wait for pin PB12 to go high */
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 90) {
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
			break;
		}
	}
	u16Tim = TIM_GetCounter(TIM2);
	if ((u16Tim >= 90) || (u16Tim <= 70)) {
		return 0;
	}

	/* Wait for pin PB12 to go low */
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 95) {
		if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
			break;
		}
	}
	u16Tim = TIM_GetCounter(TIM2);
	if ((u16Tim >= 95) || (u16Tim <= 75)) {
		return 0;
	}

	/*Receive the first byte  */
	for (i = 0; i < 8; ++i) {
		/*  Wait for pin PB12 to go high */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 65) {
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 65) || (u16Tim <= 45)) {
			return 0;
		}

		/*  Wait for pin PB12 to go low */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 80) {
			if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 80) || (u16Tim <= 10)) {
			return 0;
		}
		u8Buff[0] <<= 1;
		if (u16Tim > 45) {
			/* Bit 1 */
			u8Buff[0] |= 1;
		} else {
			/* Bit 0*/
			u8Buff[0] &= ~1;
		}
	}

	/* Receive the second byte  */
	for (i = 0; i < 8; ++i) {
		/*  Wait for pin PB12 to go high */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 65) {
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 65) || (u16Tim <= 45)) {
			return 0;
		}

		/*  Wait for pin PB12 to go low  */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 80) {
			if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 80) || (u16Tim <= 10)) {
			return 0;
		}
		u8Buff[1] <<= 1;
		if (u16Tim > 45) {
			/*  Bit 1 */
			u8Buff[1] |= 1;
		} else {
			/* Bit 0 */
			u8Buff[1] &= ~1;
		}
	}

	/*Receive the third byte */
	for (i = 0; i < 8; ++i) {
		/*  Wait for pin PB12 to go high */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 65) {
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 65) || (u16Tim <= 45)) {
			return 0;
		}

		/* Wait for pin PB12 to go low  */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 80) {
			if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 80) || (u16Tim <= 10)) {
			return 0;
		}
		u8Buff[2] <<= 1;
		if (u16Tim > 45) {
			/*  Bit 1*/
			u8Buff[2] |= 1;
		} else {
			/* Bit 0 */
			u8Buff[2] &= ~1;
		}
	}

	/*Receive fourth byte  */
	for (i = 0; i < 8; ++i) {
		/*  Wait for pin PB12 to go high */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 65) {
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 65) || (u16Tim <= 45)) {
			return 0;
		}

		/*  Wait for pin PB12 to go low  */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 80) {
			if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 80) || (u16Tim <= 10)) {
			return 0;
		}
		u8Buff[3] <<= 1;
		if (u16Tim > 45) {
			/*  Bit 1 */
			u8Buff[3] |= 1;
		} else {
			/* Bit 0 */
			u8Buff[3] &= ~1;
		}
	}

	/*Receive checksum byte   */
	for (i = 0; i < 8; ++i) {
		/*  Wait for pin PB12 to go high */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 65) {
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 65) || (u16Tim <= 45)) {
			return 0;
		}

		/*  Wait for pin PB12 to go low  */
		TIM_SetCounter(TIM2, 0);
		while (TIM_GetCounter(TIM2) < 80) {
			if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)) {
				break;
			}
		}
		u16Tim = TIM_GetCounter(TIM2);
		if ((u16Tim >= 80) || (u16Tim <= 10)) {
			return 0;
		}
		u8Buff[4] <<= 1;
		if (u16Tim > 45) {
			/*  Bit 1 */
			u8Buff[4] |= 1;
		} else {
			/* Bit 0 */
			u8Buff[4] &= ~1;
		}
	}

	u8CheckSum = u8Buff[0] + u8Buff[1] + u8Buff[2] + u8Buff[3];
	if (u8CheckSum != u8Buff[4]) {
		return 0;
	}

	for (i = 0; i < 4; ++i) {
		pu8Data[i] = u8Buff[i];
	}

	return 1;
}

void UART3_Puts(uint8_t *uartu8Data){
	uint8_t i;
	for(i = 0 ; i < strlen(uartu8Data) ; i++){
		USART_SendData(USART3, *uartu8Data);
		uartu8Data++;
		/* Check bit flag */
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){
		}
}
}


void EXTI_Configuration(void){
	EXTI_InitTypeDef   extiInit;
	NVIC_InitTypeDef   nvicInit;

	/*Enable clock for AFIOEN */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* External interrupt configuration register 1 (AFIO_EXT1CR1)*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
	/* Clear the EXTI line interrupt pending bit */
	EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4);

	/* EXTI line Configuration */
	extiInit.EXTI_Mode= EXTI_Mode_Interrupt;
	extiInit.EXTI_Line= EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4;
	extiInit.EXTI_Trigger = EXTI_Trigger_Falling;
	extiInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&extiInit);

	/* NVIC Configuration */
	nvicInit.NVIC_IRQChannel = EXTI0_IRQn | EXTI1_IRQn | EXTI2_IRQn | EXTI3_IRQn | EXTI4_IRQn ;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&nvicInit);


}

void EXTI0_IRQHandler(void){
	if (EXTI_GetFlagStatus(EXTI_Line0) == SET) {
		/* Clear the ITPending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		u32count++;


	}

}

void EXTI1_IRQHandler(void){
	if (EXTI_GetFlagStatus(EXTI_Line1) == SET) {
		/* Clear the ITPending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
		uint8_t i = 0;
		if (i == 0U) u32SetHum--;
    if (u32SetHum <= 15 ) i = 1U;
		if (u32SetHum >= 90 ) i =0U;
		if (i == 1U) u32SetHum++;
	}

}

void EXTI2_IRQHandler(void){
	if (EXTI_GetFlagStatus(EXTI_Line2) == SET) {
		/* Clear the ITPending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);
		uint8_t i = 0;
		if (i == 0U) u32SetTemp--;
        if (u32SetTemp <= 5 ) i = 1U;
		if (u32SetTemp >= 50 ) i =0U;
		if (i == 1U) u32SetTemp++;

	}

}

void EXTI3_IRQHandler(void){
	if (EXTI_GetFlagStatus(EXTI_Line3) == SET) {
		/* Clear the ITPending bit */
		EXTI_ClearITPendingBit(EXTI_Line3);
		uint8_t i = 0;
		if (i == 0U) u8SetHour--;
        if (u8SetHour == 24 ) i = 0U;
		if (u8SetHour == 0 ) i =1U;
		if (i == 1U) u32SetTemp++;

	}

}

void EXTI4_IRQHandler(void){
	if (EXTI_GetFlagStatus(EXTI_Line4) == SET) {
			/* Clear the ITPending bit */
			EXTI_ClearITPendingBit(EXTI_Line4);
			uint8_t i = 0;
			if (i == 0U) u8SetMinute--;
	        if (u8SetMinute == 60 ) i = 0U;
			if (u8SetMinute == 0 ) i =1U;
			if (i == 1U) u32SetTemp++;

		}


	}



void Delay1Ms(void)
{
	
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < 1000) {
	}
}

void delay_us(uint32_t delay)
{
	
	TIM_SetCounter(TIM2, 0);
	while (TIM_GetCounter(TIM2) < delay) {
	}
}

void Delay_Ms(uint32_t u32DelayInMs)
{
	
	while (u32DelayInMs) {
		Delay1Ms();
		--u32DelayInMs;
	}
}

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t u8Data);
uint8_t i2c_read(uint8_t u8Ack);

void i2c_init(void)
{
	GPIO_InitTypeDef gpioInit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	gpioInit.GPIO_Mode = GPIO_Mode_Out_OD;
	gpioInit.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &gpioInit);
	
	SDA_1;
	SCL_1;
}

void i2c_start(void)
{
	
	SCL_1;
	delay_us(3);
	SDA_1;
	delay_us(3);
	SDA_0;
	delay_us(3);
	SCL_0;
	delay_us(3);
}

void i2c_stop(void)
{
	
	SDA_0;
	delay_us(3);
	SCL_1;
	delay_us(3);
	SDA_1;
	delay_us(3);
}

uint8_t i2c_write(uint8_t u8Data)
{
	uint8_t i;
	uint8_t u8Ret;
	/* Write Data */
	/* Send MSB bit first */
	for (i = 0; i < 8; ++i) {
		if (u8Data & 0x80) {
			SDA_1;
		} else {
			SDA_0;
		}
		delay_us(3);
		SCL_1;
		delay_us(5);
		SCL_0;
		delay_us(2);
		u8Data <<= 1;
	}
	
	SDA_1;
	delay_us(3);
	SCL_1;
	delay_us(3);
	/* Check the last bit */
	if (SDA_VAL) {
		u8Ret = 0;
	} else {
		u8Ret = 1;
	}
	delay_us(2);
	SCL_0;
	delay_us(5);
	
	return u8Ret;
}

uint8_t i2c_read(uint8_t u8Ack)
{
	uint8_t i;
	uint8_t u8Ret;
	
	SDA_1;
	delay_us(3);
	/* Read Data */
	for (i = 0; i < 8; ++i) {
		u8Ret <<= 1;
		SCL_1;
		delay_us(3);
		if (SDA_VAL) {
			u8Ret |= 0x01;
		}
		delay_us(2);
		SCL_0;
		delay_us(5);
	}
	
	if (u8Ack) {
		SDA_0;
	} else {
		SDA_1;
	}
	delay_us(3);
	
	SCL_1;
	delay_us(5);
	SCL_0;
	delay_us(5);
	
	return u8Ret;
}


int main(void)
{
	GPIO_InitTypeDef gpioInit;
	TIM_TimeBaseInitTypeDef timerInit;
    USART_InitTypeDef uartInit;

	/* Enable Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/*Configure port C */
	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpioInit);
	
	/*Configure port B */
	gpioInit.GPIO_Mode = GPIO_Mode_IPU;
	gpioInit.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInit);


	/*Configure Timer 2 */
	timerInit.TIM_CounterMode = TIM_CounterMode_Up;
	timerInit.TIM_Period = 0xFFFF;
	timerInit.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInit(TIM2, &timerInit);
	
	TIM_Cmd(TIM2, ENABLE);
	
	/*Configure UART */
	uartInit.USART_BaudRate = 9600;
	uartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        uartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        uartInit.USART_Parity = USART_Parity_No;
        uartInit.USART_StopBits = USART_StopBits_1;
        uartInit.USART_WordLength = USART_WordLength_8b;
        USART_Init(USART3, &uartInit);

        USART_Cmd(USART3, ENABLE);

	/* Init functions */
	Rtc_Configuration();
	EXTI_Configuration();
	i2c_init();
	dht11_init();
	I2C_LCD_Init();
	I2C_LCD_Clear();
	I2C_LCD_BackLight(1);
	uint8_t DisplayScreen[17];
    uint8_t u8Buff[4];
	
	while (1) {
		dht11_read(u8Buff);
		/* Check alarm */
		if ((u8Minute == u8SetMinute) && (u8Hour == u8SetHour)){
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
		}
		/*Check warning humidity and Temperature */
		if ((u32SetHum == u8Buff[0]) || (u32SetTemp == u8Buff[2])){
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
		}

	  /* Show Humidity and Temperature  */
		if (u32count % 4 == 0U){
			if(dht11_read(u8Buff)){
				 sprintf(DisplayScreen,"Humidity : %d    ",u8Buff[0]);
				 I2C_LCD_Puts(DisplayScreen);
				 UART3_Puts(DisplayScreen);
				 I2C_LCD_NewLine();
				 sprintf(DisplayScreen,"Temperature : %d    ",u8Buff[2]);
				 I2C_LCD_Puts(DisplayScreen);
				 UART3_Puts(DisplayScreen);

	  }
			else {
				I2C_LCD_Puts("ERROR   ");
				I2C_LCD_NewLine();
				I2C_LCD_Puts("Please RESET");
			}
		}
		/* Setting Warning Hum and Temp */
		else if (u32count % 4 == 1U){
			I2C_LCD_Puts("Current Setting   ");
			I2C_LCD_NewLine();
			sprintf(DisplayScreen,"Hum:%d,Temp:%d",u32SetHum,u32SetTemp);
			I2C_LCD_Puts(DisplayScreen);
			Delay_Ms(2500);
			I2C_LCD_Clear();
			I2C_LCD_BackLight(1);
			I2C_LCD_Puts("Please setting   ");
			Delay_Ms(1500);
            while (u32count % 4 == 1U){
            	I2C_LCD_Clear();
            	I2C_LCD_BackLight(1);
            	sprintf(DisplayScreen,"SetHumidity = %d", u32SetHum);
            	I2C_LCD_Puts(DisplayScreen);
            	I2C_LCD_NewLine();
            	sprintf(DisplayScreen,"SetTemp = %d", u32SetTemp);
            	I2C_LCD_Puts(DisplayScreen);
            }

		}
		/* Show Time */
	 else if (u32count % 4 == 2U) {
		 uint8_t DisplayTime[17];
		 Time(u8Second, u8Minute, u8Hour, u8Day, u8Month, u8Year);
		 sprintf(DisplayTime,"   %d: %d: %d ", u8Second, u8Minute, u8Hour);
		 I2C_LCD_Puts(DisplayTime);
		 I2C_LCD_NewLine();
		 sprintf(DisplayTime,"   %d: %d: %d ", u8Day, u8Month, u8Year);
		 I2C_LCD_Puts(DisplayTime);


	 }
	 /* Setting Time */
	 else if (u32count % 4 == 3U) {
		 I2C_LCD_Puts("Current Setting   ");
		 I2C_LCD_NewLine();
		 sprintf(DisplayScreen,"  %d: %d ", u8SetMinute, u8SetHour);
		 I2C_LCD_Puts(DisplayScreen);
		 Delay_Ms(2500);
		 I2C_LCD_Clear();
		 I2C_LCD_BackLight(1);
		 I2C_LCD_Puts("Please setting   ");
		 Delay_Ms(1500);
		 while (u32count % 4 == 3U){
		             	I2C_LCD_Clear();
		             	I2C_LCD_BackLight(1);
		             	sprintf(DisplayScreen," %d: %d ", u8SetMinute, u8SetHour);
		             	I2C_LCD_Puts(DisplayScreen);


	 }


}
}
}
