/* Embedded Systems - Exercise 5 */

#include <tinyprintf.h>
#include <stm32f4/rcc.h>
#include <stm32f4/gpio.h>
#include <stm32f4/nvic.h>
#include <stm32f4/exti.h>
#include <stm32f4/syscfg.h>
#include <stm32f4/tim.h>
#include <stm32f4/adc.h>


// GPIOD
//#define X P
#define C1 X                            //TODO
#define C2 X                            //TODO
#define C3 X                            //TODO
#define C4 X                            //TODO
#define C5 X                            //TODO
#define C6 X                            //TODO
#define C7 X                            //TODO
#define C8 X                            //TODO


// GPIODA

void init(){

	

}

int lire_valeur(int x)
{

    return 0;
}

int main() {
	printf("\nStarting...\n");

	// RCC init
	RCC_AHB1ENR |= RCC_GPIOAEN;
	RCC_AHB1ENR |= RCC_GPIODEN;
	RCC_APB1ENR |= RCC_TIM4EN;
	RCC_APB2ENR |= RCC_ADC1EN;

	// initialization
	init();
	// main loop
	printf("Endless loop!\n");
	while(1) {

	}__asm("nop");

}
 
