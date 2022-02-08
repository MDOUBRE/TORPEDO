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
#define MD1 4
#define MD2 6
#define MG1 5
#define MG2 7
#define MODE 8

#define TIM3_DIV 8
#define TIM3_PERIOD	60000

#define KP_MD
#define KI_MD
#define KD_MD
#define KP_MG
#define KI_MG
#define KD_MG

int puiss_MD = 0;
int puiss_MG = 0;
int sens_MD = 0;
int sens_MG = 0;

/****************************************/
/*            BRANCHEMENTS              */
/*                                      */
/*  VCC ==> 3V, GND ==> GND             */
/*  B4 ==> EN A                         */
/*  B6 ==> PH A                         */
/*  B5 ==> EN B                         */
/*  B7 ==> PH B                         */
/*  VCC => 6/9V, GND ==> GND de la pile */
/*  MD : rouge sur O1, noire sur O2     */
/*  MG : rouge sur O1, noire sur O2     */ 
/****************************************/

void init_moteurs(){
    //init des deux pins pour le moteur 1
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*MD1, 2, 0b10);
    GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*MD1, 4, 2);
    GPIOB_OTYPER &= ~(1 << MD1);
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*MD2, 2, 0b01);
	GPIOB_OTYPER &= ~(1 << MD2);
	GPIOB_PUPDR = SET_BITS(GPIOB_PUPDR, 2*MD2, 2, 0b01);

    // init des deux pins pour le moteur 2
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*MG1, 2, 0b10);
    GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*MG1, 4, 2);
    GPIOB_OTYPER &= ~(1 << MG1);
    GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*MG2, 2, 0b01);
	GPIOB_OTYPER &= ~(1 << MG2);
	GPIOB_PUPDR = SET_BITS(GPIOB_PUPDR, 2*MG2, 2, 0b01);

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
	TIM3_ARR = 0xE0F;
	TIM3_CCMR1 = TIM_CCS1S_OUT | TIM_OC1M_PWM1 | TIM_CCS2S_OUT | TIM_OC2M_PWM1;
    //TIM3_CCMR2 = TIM_CCS2S_OUT | TIM_OC2M_PWM1;
	TIM3_CCER = TIM_CC1E | TIM_CC2E;
	TIM3_CCR1 = 0;
	TIM3_CCR2 = 0;
	TIM3_CR1 = TIM_CEN | TIM_ARPE;
    //TIM3_CR2 = TIM_CEN | TIM_ARPE;
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
    if(GPIOB_ODR & (1<<MD2)){
        GPIOB_BSRR = 1 << (MD2 + 16);
    }
    else{
        GPIOB_BSRR = 1 << MD2;

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
    if(GPIOB_ODR & (1<<MG2)){
        GPIOB_BSRR = 1 << (MG2 + 16);
    }
    else{
        GPIOB_BSRR = 1 << MG2;
    }
}

void test_moteurs(){
    /*
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("MD à 20\n");
    set_M1(30);    
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("MD à 50\n");
    set_M1(50);    
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("MD à 80\n");
    set_M1(80);
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("Stop MD\n");   
    stop_M1();
        
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("MG à 20\n");
    set_M2(30);    
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("MG à 50\n");
    set_M2(50);    
    for(int i = 0;i<90000000;i++)__asm("nop");
    printf("MG à 80\n");
    set_M2(80);
    for(int i = 0;i<90000000;i++)__asm("nop");   
    printf("Stop MG\n");   
    stop_M2();
    */

    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M1(38);
    set_M2(30);
    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M1(60);
    set_M2(50);
    /*
    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M1(80);
    set_M2(80);
    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M1(100);
    set_M2(100);
    
    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    stop_M2();

    for(int i = 0;i<90000000;i++)__asm("nop");
    inverseM2();
    set_M1(50);
    set_M2(50);

    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    stop_M2();

    for(int i = 0;i<90000000;i++)__asm("nop");
    inverseM1();
    inverseM2();
    set_M1(50);
    set_M2(50);

    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    stop_M2();

    for(int i = 0;i<90000000;i++)__asm("nop");
    inverseM1();
    set_M1(30);
    set_M2(30);

    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    stop_M2();

    for(int i = 0;i<90000000;i++)__asm("nop");
    inverseM1();
    inverseM2();
    set_M1(30);
    set_M2(30);

    for(int i = 0;i<90000000;i++)__asm("nop");
    inverseM1();
    inverseM2();
    set_M1(30);
    set_M2(30);
    */
    for(int i = 0;i<90000000;i++)__asm("nop");
    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    stop_M2();
}

int main(){
    RCC_AHB1ENR |= RCC_GPIOBEN;
	RCC_AHB1ENR |= RCC_GPIODEN;
	RCC_APB1ENR |= RCC_TIM3EN;

    init_moteurs();
    init_TIM3();
        
    if(GPIOB_ODR & (1<<MD2)){
        sens_MD = 0;
    }
    else{
        sens_MD = 1;
    }
    if(GPIOB_ODR & (1<<MD2)){
        sens_MD = 0;
    }
    else{
        sens_MD = 1;
    }
    //²printf("les inits sont fait\n");
    
    test_moteurs();
    /*
    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M2(30);
    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M2();
    inverseM2();
    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M2(30);
    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M2();

    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M1(30);
    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    inverseM1();
    for(int i = 0;i<90000000;i++)__asm("nop");
    set_M1(30);
    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    
    set_M1(30);
    for(int i = 0;i<90000000;i++)__asm("nop");
    stop_M1();
    */
    while(1){

    }__asm("nop");


    return 0;
}