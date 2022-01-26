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
#define C1 0                            
#define C2 1                            
#define C3 2                            
#define C4 3                            
#define C5 4                            
#define C6 5                           
#define C7 6                            
#define C8 7                            


// GPIODA

#define WAIT_PSC 100
#define WAIT_DELAY (APB1_CLK / WAIT_PSC)
#define HALF_PERIOD (WAIT_DELAY/2)

#define SEUIL_BLANC	4
#define SEUIL_NOIR	8

volatile int un_sur_deux = 0;
volatile int TIM4_triggered = 0;
volatile enum{ D0 , R1 , R2 , R3 , L1 , L2 , L3 , SR} state=D0;

#define SEUIL_BLANC	3
#define SEUIL_NOIR	7

//E0  1278 -> B ; 45 -> N
//R1  5678 -> B ; 23 -> N
//R2  4567 -> B ; 12 -> N
//L1  1234 -> B ; 67 -> N
//L2  2345 -> B ; 78 -> N

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


void CHARGE()
{
	for (short int i=0; i < 8; i++)                              // pin 0 à 7
	{
		GPIOA_MODER = SET_BITS(GPIOA_MODER, i*2, 2, 0b01); //set GPIO A ou D
		GPIOA_OTYPER = GPIOA_OTYPER & ~(1 << i);
		GPIOA_PUPDR = SET_BITS(GPIOA_PUPDR, i*2, 2, 0b01);
	}
}


void CAPTURE_START()
{
	for (short int i=0; i < 8; i++)                              // pin 0 à 7
	{
		GPIOA_MODER = SET_BITS(GPIOA_MODER, i*2, 2, 0b00); //set GPIO A ou D
		GPIOA_OTYPER = GPIOA_OTYPER & ~(1 << i);
		GPIOA_PUPDR = SET_BITS(GPIOA_PUPDR, i*2, 2, 0b01);
	}
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
    short int x=0;
    CHARGE();
    short int vc[8] = { 12 , 12 , 12 , 12 , 12 , 12 , 12 , 12 };
	while(1) {
        if(TIM4_triggered){
			TIM4_triggered = 0;
			switch(state){
			case D0 :
			
			   // mettre les roues à la même vitesse
			   //accélérer les roues jusqu'à atteindre la vitesse max 
			    
				for (int i=0; i < 8; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))!= 0))
					{
						vc[i] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
                    CHARGE();
                    if( vc[3] < SEUIL_NOIR)
                    {
                    	state = R1;
                    	printf("R1\n");
                    }
                    if( vc[4] < SEUIL_NOIR)
                    {
                    	state = L1;
                    	printf("R1\n");
                    }		
                }
                else
                {
                	if (x == 1)
                	{
                		CAPTURE_START();
                		vc[0] = 12;
                		vc[1] = 12;
                		vc[2] = 12;
                		vc[3] = 12;
                		vc[4] = 12;
                		vc[5] = 12;
                		vc[6] = 12;
                		vc[7] = 12;
                	}
                	x = x + 1;
                }
				break;
				
			case R1 :
			    //ralentir la roue gauche légèrement
			    for (int i=0; i < 8; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))!= 0))
					{
						vc[i] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
                    CHARGE();
                    if( vc[3] < SEUIL_BLANC)
                    {
                    	state = R2;
                    	printf("R2\n");
                    }
                    if( vc[3] > SEUIL_NOIR)
                    {
                    	state = D0;
                    	printf("D0\n");
                    }
                    if( vc[4] < SEUIL_NOIR)
                    {
                    	state = L1;
                    	printf("L1\n");
                    }
                }
                else
                {
                	if (x == 1)
                	{
                		CAPTURE_START();
                		vc[0] = 12;
                		vc[1] = 12;
                		vc[2] = 12;
                		vc[3] = 12;
                		vc[4] = 12;
                		vc[5] = 12;
                		vc[6] = 12;
                		vc[7] = 12;
                	}
                	x = x + 1;
                }
				break;
			case R2 :
			    //ralentir la roue gauche modérément
			    //accélérer la roue droite légèrement si possible
			    for (int i=0; i < 8; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))!= 0))
					{
						vc[i] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
                    CHARGE();
                    if( vc[3] > SEUIL_BLANC)
                    {
                    	state = R1;
                    	printf("R1\n");
                    }
                    if( vc[1] > SEUIL_NOIR || vc[0] > SEUIL_NOIR )
                    {
                    	state = R3;
                    	printf("R3\n");
                    }
                }
                else
                {
                	if (x == 1)
                	{
                		CAPTURE_START();
                		vc[0] = 12;
                		vc[1] = 12;
                		vc[2] = 12;
                		vc[3] = 12;
                		vc[4] = 12;
                		vc[5] = 12;
                		vc[6] = 12;
                		vc[7] = 12;
                	}
                	x = x + 1;
                }
				break;
			case R3 :
    			//ralentir la roue gauche fortement
			    //accélérer la roue droite fortement si possible
			    for (int i=0; i < 8; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))!= 0))
					{
						vc[i] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
                    CHARGE();
                    if( vc[0] > SEUIL_BLANC && vc[1] > SEUIL_BLANC )
                    {
                    	state = SR;
                    	printf("SR\n");
                    }
                    if( vc[2] > SEUIL_NOIR || vc[3] > SEUIL_NOIR )
                    {
                    	state = R2;
                    	printf("R2\n");
                    }
                }
                else
                {
                	if (x == 1)
                	{
                		CAPTURE_START();
                		vc[0] = 12;
                		vc[1] = 12;
                		vc[2] = 12;
                		vc[3] = 12;
                		vc[4] = 12;
                		vc[5] = 12;
                		vc[6] = 12;
                		vc[7] = 12;
                	}
                	x = x + 1;
                }
				break;
				
			case L1 :
			    //ralentir la roue droite légèrement			
                for (int i=0; i < 8; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))!= 0))
					{
						vc[i] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
                    CHARGE();
                    if( vc[4] < SEUIL_BLANC)
                    {
                    	state = L2;
                    	printf("L2\n");
                    }
                    if( vc[4] > SEUIL_NOIR)
                    {
                    	state = D0;
                    	printf("D0\n");
                    }
                    if( vc[3] < SEUIL_NOIR)
                    {
                    	state = R1;
                    	printf("R1\n");
                    }
                }
                else
                {
                	if (x == 1)
                	{
                		CAPTURE_START();
                		vc[0] = 12;
                		vc[1] = 12;
                		vc[2] = 12;
                		vc[3] = 12;
                		vc[4] = 12;
                		vc[5] = 12;
                		vc[6] = 12;
                		vc[7] = 12;
                	}
                	x = x + 1;
                }		
				break;
			case L2 :
			    //ralentir la roue droite modérément
			    //accélérer la roue gauche légèrement si possible
			    for (int i=0; i < 8; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))!= 0))
					{
						vc[i] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
                    CHARGE();
                    if( vc[4] > SEUIL_BLANC)
                    {
                    	state = L1;
                    	printf("L1\n");
                    }
                    if( vc[6] > SEUIL_NOIR || vc[7] > SEUIL_NOIR )
                    {
                    	state = L3;
                    	printf("L3\n");
                    }
                }
                else
                {
                	if (x == 1)
                	{
                		CAPTURE_START();
                		vc[0] = 12;
                		vc[1] = 12;
                		vc[2] = 12;
                		vc[3] = 12;
                		vc[4] = 12;
                		vc[5] = 12;
                		vc[6] = 12;
                		vc[7] = 12;
                	}
                	x = x + 1;
                }
				break;
			case L3 :
			    //ralentir la roue droite fortement
			    //accélérer la roue gauche fortement si possible
			    for (int i=0; i < 8; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))!= 0))
					{
						vc[i] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
                    CHARGE();
                    if( vc[7] > SEUIL_BLANC && vc[6] > SEUIL_BLANC )
                    {
                    	state = SR;
                    	printf("SR\n");
                    }
                    if( vc[5] > SEUIL_NOIR || vc[4] > SEUIL_NOIR )
                    {
                    	state = L2;
                    	printf("L2\n");
                    }
                }
                else
                {
                	if (x == 1)
                	{
                		CAPTURE_START();
                		vc[0] = 12;
                		vc[1] = 12;
                		vc[2] = 12;
                		vc[3] = 12;
                		vc[4] = 12;
                		vc[5] = 12;
                		vc[6] = 12;
                		vc[7] = 12;
                	}
                	x = x + 1;
                }
				break;
			case SR :
			    //sortie de route
			
				break;
		}
	}__asm("nop");

}
 
