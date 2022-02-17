#include <tinyprintf.h>
#include <stm32f4/rcc.h>
#include <stm32f4/gpio.h>
#include <stm32f4/nvic.h>
#include <stm32f4/exti.h>
#include <stm32f4/syscfg.h>
#include <stm32f4/tim.h>
#include <stm32f4/adc.h>

#define SM 4

#define TIM3_DIV 27
#define TIM3_PERIOD	59999

void init_SM(){
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*SM, 2, 0b10);
    GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*SM, 4, 2);
    GPIOB_OTYPER &= ~(1 << SM);
}

void init_TIM3(){
	// TIM3
    //TIM3_PSC = 26;
    //TIM3_PSC = 84;
	//TIM3_ARR = 59999;
    //TIM3_ARR = 59973;
    //TIM3_ARR = 70000;
    
    //TIM3_PSC = 3750;
    //TIM3_ARR = 255;    

    //TIM3_PSC = 1000;
    //TIM3_ARR = 100;

    TIM3_PSC = 9;
    TIM3_ARR = 21000;
	
    //TIM3_ARR = 0xE0F;
    //TIM3_ARR = 9916;
    //TIM3_ARR = 10000;
    //TIM3_PSC = 0;
    //TIM3_ARR = 49999;

	TIM3_CCMR1 = TIM_CCS1S_OUT | TIM_OC1M_PWM1;
	TIM3_CCER = TIM_CC1E;
	TIM3_CCR1 = 0;
	TIM3_CR1 = TIM_CEN | TIM_ARPE;  
}

void start_SM() {
	TIM3_CCR1 = TIM3_ARR;
}

void stop_SM() {
	TIM3_CCR1 = 0;
}

void set_SM(int pulse) {
    TIM3_CCR1 = (pulse * TIM3_ARR / 100);
}

void depose_caisse(){
    set_SM(50);
    for(int i = 0;i<40000000;i++)__asm("nop");
    stop_SM();
    for(int i = 0;i<40000000;i++)__asm("nop");
    set_SM(10);
    for(int i = 0;i<40000000;i++)__asm("nop");
    stop_SM();
    for(int i = 0;i<40000000;i++)__asm("nop");
}

int main(){
    RCC_AHB1ENR |= RCC_GPIOBEN;
    RCC_APB1ENR |= RCC_TIM3EN;

    init_SM();
    init_TIM3();
    
    depose_caisse();

    while(1){        
    }__asm("nop");

    return 0;
}