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

#define WAIT_PSC 1
#define WAIT_DELAY (APB1_CLK / WAIT_PSC)
#define HALF_PERIOD (WAIT_DELAY/2)

volatile int un_sur_deux = 0;
volatile int TIM4_triggered = 0;
volatile enum{ E0 , E1 , E2 , E3 , E4 , E5 , E6 } state=CHARGE;

// calcul du temps entre de le moment ou la ligne d'un capteur est montee a 1 et le moment ou elle retombe a 0
// passer la ligne du capteur en outpout avec pull up
// attendre 10 microseconde pour que la ligne soit a 1
// passer la ligne en inpout -> la tension de la ligne va diminuer jusqu'a etre nulle 
// calculer le temps de descente a 0



void handle_TIM4() {
	TIM4_ARR = HALF_PERIOD;
    if(un_sur_deux!=1){    
		TIM4_triggered = 1;
        
        un_sur_deux+=1;
    }
    else{
        un_sur_deux=0;
    }
	
	TIM4_SR &= ~TIM_UIF;
}

void init_TIM4(){
	DISABLE_IRQS;

	NVIC_ICER(TIM4_IRQ >> 5) |= 1 << (TIM4_IRQ & 0X1f);
	NVIC_IRQ(TIM4_IRQ) = (uint32_t)handle_TIM4;
	NVIC_IPR(TIM4_IRQ) = 0;

	NVIC_ICPR(TIM4_IRQ >> 5) |= 1 << (TIM4_IRQ & 0X1f);
	NVIC_ISER(TIM4_IRQ >> 5) |= 1 << (TIM4_IRQ & 0X1f);

	TIM4_CR1 = 0;
	TIM4_PSC = WAIT_PSC;
	TIM4_ARR = HALF_PERIOD;
	TIM4_EGR = TIM_UG;
	TIM4_SR = 0;
	TIM4_CR1 = TIM_ARPE;
	TIM4_SR &= ~TIM_UIF;
	TIM4_DIER = TIM_UIE;

	ENABLE_IRQS;
	TIM4_CR1 |= TIM_CEN;
}


void init(){
    
	

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
    init_TIM4();
	// main loop
	printf("Endless loop!\n");
    int x=0;
    int vc1, vc2, vc3, vc4, vc5, vc6, vc7, vc8;
	while(1) {
        if(TIM4_triggered){
			TIM4_triggered = 0;
			switch(state){
			case E0 :

                if (x == 10)
                {
                    x=0;

                }
				break;
			
			/*case E1 :

				break;

			case E2 :

				break;

			case E3 :

				break;

			case E4 :

				break;

			case E5 :

				break;*/
		}
	}__asm("nop");

}
 
