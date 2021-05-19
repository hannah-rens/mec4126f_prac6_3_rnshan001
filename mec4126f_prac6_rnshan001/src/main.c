// Description----------------------------------------------------------------|
/*
 * Initialises a struct with Name and Age data. Displays results on LEDs and
 * LCD.
 */
// DEFINES AND INCLUDES-------------------------------------------------------|

#define STM32F051
//>>> Uncomment line 10 if using System Workbench (SW4STM32) or STM32CubeIDE
//#define SW4STM32

#ifndef SW4STM32
	#define TRUESTUDIO
#endif

#include "stm32f0xx.h"
#include "lcd_stm32f0-1.h"
// GLOBAL VARIABLES ----------------------------------------------------------|
uint8_t = ADC_value; //adc value after conversation
unsigned char buffer[16];

// FUNCTION DECLARATIONS -----------------------------------------------------|
void init_ADC(void);
void display_on_LCD();
void init_LEDS(void);
void display_on_LEDs();

void main(void);                                                   //COMPULSORY

#ifdef TRUESTUDIO												   //COMPULSORY
	void reset_clock_to_48Mhz(void);							   //COMPULSORY
#endif															   //COMPULSORY



// MAIN FUNCTION -------------------------------------------------------------|

void main(void)
{
#ifdef TRUESTUDIO  											 	   //COMPULSORY
	reset_clock_to_48Mhz();										   //COMPULSORY
#endif															   //COMPULSORY
	void init_ADC();
	void display_on_LCD();
	void init_LEDS();
	void display_on_LEDs();
	while(1)
	{
		ADC_value = ADC1 -> DR;
	}
}

// OTHER FUNCTIONS -----------------------------------------------------------|
void init_ADC(void)
{
    RCC -> APB2ENR |= RCC_APB2ENR_ADCEN; // enable clock for ADC
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN; // enable clock PORT A
    GPIOA -> MODER |= GPIO_MODER_MODER5; // set PA5, analogue mode
    ADC1 -> CFGR1 |= ADC_CFGR1_RES_1; // 8 bit resolution
    ADC1->CFGR1 |= ADC_CFGR1_CONT; //continuous conversion mode
    ADC1->CR |= ADC_CR_ADEN; //set ADEN=1
    while((ADC1->ISR & ADC_ISR_ADRDY)==0); //wait until ADRDY==1
    ADC1->CR |= ADC_CR_ADSTART; //start the ADC
}
void display_on_LCD()
{
    sprintf(buffer,"%f",ADC_value);
    init_LCD();
    lcd_command(CLEAR);
    lcd_putstring(buffer);
}

void init_LEDS(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock
    GPIOB->MODER |= GPIO_MODER_MODER0_0;
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOB->MODER |= GPIO_MODER_MODER2_0;
    GPIOB->MODER |= GPIO_MODER_MODER3_0;
    GPIOB->MODER |= GPIO_MODER_MODER4_0;
    GPIOB->MODER |= GPIO_MODER_MODER5_0;
    GPIOB->MODER |= GPIO_MODER_MODER6_0;
    GPIOB->MODER |= GPIO_MODER_MODER7_0;
}
void display_on_LEDs(void)
{
    GPIOB -> ODR = ADC_value;
}


#ifdef TRUESTUDIO												   //COMPULSORY
/* Description:
 * This function resets the STM32 Clocks to 48 MHz
 */
void reset_clock_to_48Mhz(void)									   //COMPULSORY
{																   //COMPULSORY
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)			   //COMPULSORY
	{															   //COMPULSORY
		RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);					   //COMPULSORY
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);	   //COMPULSORY
	}															   //COMPULSORY

	RCC->CR &= (uint32_t)(~RCC_CR_PLLON);						   //COMPULSORY
	while ((RCC->CR & RCC_CR_PLLRDY) != 0);						   //COMPULSORY
	RCC->CFGR = ((RCC->CFGR & (~0x003C0000)) | 0x00280000);		   //COMPULSORY
	RCC->CR |= RCC_CR_PLLON;									   //COMPULSORY
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);						   //COMPULSORY
	RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);					   //COMPULSORY
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);		   //COMPULSORY
}																   //COMPULSORY
#endif															   //COMPULSORY






// INTERRUPT HANDLERS --------------------------------------------------------|




