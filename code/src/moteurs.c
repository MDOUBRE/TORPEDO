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

#define TIM3_DIV 8
#define TIM3_PERIOD	60000

#define KP_M1
#define KI_M1
#define KD_M1
#define KP_M2
#define KI_M2
#define KD_M2

int puiss_M1 = 0;
int puiss_M2 = 0;
int sens_M1 = 0;
int sens_M2 = 0;

void init_moteurs(){
    //init des deux pins pour le moteur 1
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M1P1, 2, 0b10);
    GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*M1P1, 4, 2);
    GPIOB_OTYPER &= ~(1 << M1P1);
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M1P2, 2, 0b01);
	GPIOB_OTYPER &= ~(1 << M1P2);
	GPIOB_PUPDR = SET_BITS(GPIOB_PUPDR, 2*M1P2, 2, 0b01);

    // init des deux pins pour le moteur 2
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M2P1, 2, 0b10);
    GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*M2P1, 4, 2);
    GPIOB_OTYPER &= ~(1 << M2P1);
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*M2P2, 2, 0b01);
	GPIOB_OTYPER &= ~(1 << M2P2);
	GPIOB_PUPDR = SET_BITS(GPIOB_PUPDR, 2*M2P2, 2, 0b01);

    // init de la pin pour le mode
    GPIOD_MODER = SET_BITS(GPIOD_MODER, 2*MODE, 2, 0b01);
	GPIOD_OTYPER &= ~(1 << MODE);
	GPIOD_PUPDR = SET_BITS(GPIOD_PUPDR, 2*MODE, 2, 0b01);

    GPIOD_BSRR = 1 << MODE;
}

// pour le moment 20KHz
void init_TIM3() {
	// TIM3
    TIM3_PSC = 0;
	//TIM3_CCR1 = 0;
	TIM3_ARR = 0xE0F;
	//TIM3_PSC = TIM3_DIV - 1;
	TIM3_CCMR1 = TIM_CCS1S_OUT | TIM_OC1M_PWM1
			   | TIM_CCS2S_OUT | TIM_OC2M_PWM1;
	TIM3_CCER = TIM_CC1E | TIM_CC2E;
	TIM3_CCR1 = 0;
	TIM3_CCR2 = 0;
	TIM3_CR1 = TIM_CEN | TIM_ARPE;
}

void start_M1() {
	TIM3_CCR1 = TIM3_ARR;
}

void stop_M1() {
	TIM3_CCR1 = 0;
}

void set_M1(int pulse) {
    TIM3_CCR1 = (pulse * TIM3_ARR / 100);
}

void inverseM1(){
    if(GPIOB_ODR & (1<<M1P2)){
        GPIOB_BSRR = 1 << (M1P2 + 16);
    }
    else{
        GPIOB_BSRR = 1 << M1P2;
    }
}

void start_M2() {
	TIM3_CCR2 = TIM3_ARR;
}

void stop_M2() {
	TIM3_CCR2 = 0;
}

void set_M2(int pulse) {
	TIM3_CCR2 = (pulse * TIM3_ARR / 100);
}

void inverseM2(){
    if(GPIOB_ODR & (1<<M1P2)){
        GPIOB_BSRR = 1 << (M1P2 + 16);
    }
    else{
        GPIOB_BSRR = 1 << M1P2;
    }
}

int main(){
    RCC_AHB1ENR |= RCC_GPIOBEN;
	RCC_AHB1ENR |= RCC_GPIODEN;
	RCC_APB1ENR |= RCC_TIM3EN;

    init_moteurs();
    init_TIM3();
    printf("les inits sont fait\n");

    printf("Stop des moteurs\n");
    stop_M1();

    printf("Allumage des moteurs\n");
    start_M1();

    printf("Stop des moteurs\n");
    stop_M1();
        
    printf("Allumage des moteurs\n");
    start_M1();
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("M1 à 20\n");
    set_M1(20);    
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("M1 à 50\n");
    set_M1(50);    
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("M1 à 80\n");
    set_M1(80);
    for(int i = 0;i<90000000;i++)__asm("nop");

    printf("Stop des moteurs\n");   
    stop_M1();
    for(int i = 0;i<90000000;i++)__asm("nop");

    start_M1();
    for(int i = 0;i<90000000;i++)__asm("nop");

    inverseM1();
    for(int i = 0;i<90000000;i++)__asm("nop");

    inverseM1();
    for(int i = 0;i<90000000;i++)__asm("nop");
    
    stop_M1();
    while(1){

    }__asm("nop");


    return 0;
}