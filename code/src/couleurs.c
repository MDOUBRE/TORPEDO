#include <tinyprintf.h>
#include <stm32f4/rcc.h>
#include <stm32f4/gpio.h>
#include <stm32f4/nvic.h>
#include <stm32f4/exti.h>
#include <stm32f4/syscfg.h>
#include <stm32f4/tim.h>
#include <stm32f4/adc.h>

#define R 4
#define G 5
#define B 6
#define ADC 3

#define WAIT_PSC 1000
#define WAIT_DELAY (APB1_CLK / WAIT_PSC)
#define DELAY_1000 (WAIT_DELAY)
#define DELAY_500 (WAIT_DELAY/2)
#define DELAY_100 (WAIT_DELAY/10)

enum{ETEINT, ROUGE, VERT, BLEU, CAPTE} couleur = ETEINT;
int prochain = 0; //0 = rouge, 1 = vert, 2 = bleu 
int val_rouge = 0;
int val_vert = 0;
int val_bleu = 0;

void init(){
	// LEDs
    for(int i=4;i<=6;i++){
        GPIOD_MODER = SET_BITS(GPIOD_MODER, 2*i, 2, 0b01);
        GPIOD_OTYPER &= ~(1 << i);
        GPIOD_PUPDR = SET_BITS(GPIOD_PUPDR, 2*i, 2, 0b01);
    }

    GPIOA_MODER = SET_BITS(GPIOA_MODER, ADC*2, 2, 0b11);
	GPIOA_PUPDR = SET_BITS(GPIOA_PUPDR, ADC*2, 2, 0b01);
	ADC1_SQR3 = 3;
	ADC1_CR1 = 0;
	ADC1_CR2 = ADC_ADON;
}

void init_TIM4(){
	TIM4_CR1 = 0;
	TIM4_PSC = WAIT_PSC;
	TIM4_ARR = DELAY_500;
	TIM4_EGR = TIM_UG;
	TIM4_SR = 0;
	TIM4_CR1 = TIM_ARPE;
}

void handle_adc(int couleur){
    int val = 0;
    ADC1_CR2 |= ADC_SWSTART;
    while((ADC1_SR & ADC_EOC) == 0)__asm("nop");
    val = ADC1_DR;
    if(couleur==0){
        val_rouge = val;
    }
    else if(couleur==1){
        val_vert = val;
    }
    else{
        val_bleu = val;
    }
}

int main(){
    RCC_AHB1ENR |= RCC_GPIODEN;
    RCC_AHB1ENR |= RCC_GPIOAEN;
    RCC_APB1ENR |= RCC_TIM4EN;
    RCC_APB2ENR |= RCC_ADC1EN;

    init();
    init_TIM4();

    TIM4_CR1 = TIM4_CR1 | TIM_CEN;
    
    int cas = 0;
    int allume = 0;
    printf("coucou\n");
    while(1){
        for (int i=0;i<6;i++){
            if((TIM4_SR & TIM_UIF)!=0){
                switch(couleur){
                case ETEINT:
                    GPIOD_BSRR = 1 << (R+16);
                    GPIOD_BSRR = 1 << (G+16);
                    GPIOD_BSRR = 1 << (B+16);
                    if(prochain==0){
                        couleur = ROUGE;
                        prochain = (prochain+1)%4;
                    }
                    else if(prochain==1){
                        couleur = VERT;
                        prochain = (prochain+1)%4;
                    }
                    else if(prochain==2){
                        couleur = BLEU;
                        prochain = (prochain+1)%4;
                    }
                    else{
                        couleur = CAPTE;
                        prochain = (prochain+1)%4;
                    }
                    break;
                case ROUGE:
                    GPIOD_BSRR = 1 << R;
                    handle_adc(0);
                    couleur = ETEINT;
                    break;
                case VERT:
                    GPIOD_BSRR = 1 << G;
                    handle_adc(1);
                    couleur = ETEINT;
                    break;
                case BLEU:
                    GPIOD_BSRR = 1 << B;
                    handle_adc(2);
                    couleur = ETEINT;
                    break;
                case CAPTE:
                    if((val_rouge > val_vert) && (val_rouge > val_bleu)){
                        printf("valRouge = %d", val_rouge);
                        printf("valVert = %d", val_vert);
                        printf("valBleu = %d", val_bleu);
                        printf("rouge capte\n");
                        GPIOD_BSRR = 1 << R;
                    }
                    else if((val_vert > val_rouge) && (val_vert > val_bleu)){
                        printf("valRouge = %d", val_rouge);
                        printf("valVert = %d", val_vert);
                        printf("valBleu = %d", val_bleu);
                        printf("vert capte\n");
                        GPIOD_BSRR = 1 << G;
                    }
                    else if((val_bleu > val_rouge) && (val_bleu > val_vert)){
                        printf("valRouge = %d", val_rouge);
                        printf("valVert = %d", val_vert);
                        printf("valBleu = %d", val_bleu);
                        printf("bleu capte\n");
                        GPIOD_BSRR = 1 << B;
                    }
                    couleur = ETEINT;
                default:
                    break;
                }
                TIM4_SR = 0;
        
            }
        }
    }__asm("nop");    

    return 0;
}