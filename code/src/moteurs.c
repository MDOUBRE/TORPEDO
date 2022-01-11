#include <tinyprintf.h>
#include <stm32f4/rcc.h>
#include <stm32f4/gpio.h>
#include <stm32f4/nvic.h>
#include <stm32f4/exti.h>
#include <stm32f4/syscfg.h>
#include <stm32f4/tim.h>
#include <stm32f4/adc.h>

// les 4 pins pour les moteurs + pin MODE
// sur GPIOB
#define M1P1 4
#define M1P2 5
#define M2P1 6
#define M2P2 7
#define MODE 8

#define TIM3_DIV	7
#define TIM3_PERIOD	60000

#define KP_M1
#define KI_M1
#define KD_M1
#define KP_M2
#define KI_M2
#define KD_M2

init_moteurs(){
    // init des deux pins pour le moteur 1
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M1P1, 2, 0b10);
    GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*M1P1, 4, 2);
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M1P2, 2, 0b01);
	GPIOB_OTYPER &= ~(1 << M1P2);
	GPIOB_PUPDR = SET_BITS(GPIOB_PUPDR, 2*M1P2, 2, 0b01);

    // init des deux pins pour le moteur 2
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M2P1, 2, 0b10);
    GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*M2P1, 4, 2);
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M2P2, 2, 0b01);
	GPIOB_OTYPER &= ~(1 << M2P2);
	GPIOB_PUPDR = SET_BITS(GPIOB_PUPDR, 2*M2P2, 2, 0b01);

    // init de la pin pour le mode
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*MODE, 2, 0b01);
	GPIOB_OTYPER &= ~(1 << MODE);
	GPIOB_PUPDR = SET_BITS(GPIOB_PUPDR, 2*MODE, 2, 0b01);

    GPIOB_BSRR = 1 << M1P2;
    GPIOB_BSRR = 1 << M2P2;
    GPIOB_BSRR = 1 << MODE;
}

void init_TIM3() {
	// TIM3
	TIM3_CCR1 = 0;
	TIM3_ARR = TIM3_PERIOD;
	TIM3_PSC = TIM3_DIV - 1;
	TIM3_CCMR1 = TIM_OC1S_OUT | TIM_OC1M_PWM1
			   | TIM_OC2S_OUT | TIM_OC2M_PWM1;
	TIM3_CCER = TIM_CC1E | TIM_CC2E;
	TIM3_CC1R = 0;
	TIM3_CC2R = 0;
	TIM3_CCR1 = TIM_CEN | CIM_ARPE;
}

void start_M1() {
	TIM3_CC1R = TIM3_PERIOD;
}

void stop_M1() {
	TIM3_CC1R = 0;
}

void start_M2() {
	TIM3_CC2R = PUMP_PERIOD;
}

void stop_M2() {
	TIM3_CC2R = 0;
}

void set_M1(int n) {
	TIM3_CC1R = n * TIM3_PERIOD / 100;
}

void set_M2(int n) {
	TIM3_CC2R = n * TIM3_PERIOD / 100;
}

int main(){
    init_moteurs();
    init_TIM3();

    set_M1(20);
    for(int i = 0;i<30000000;i++)__asm("nop");
    set_M1(50);
    for(int i = 0;i<30000000;i++)__asm("nop");
    set_M1(80);
    for(int i = 0;i<30000000;i++)__asm("nop");
    stop_M1();  

    while(1){

    }__asm("nop");


    return 0;
}