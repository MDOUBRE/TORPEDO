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

#define WAIT_PSC 7
#define WAIT_DELAY (APB1_CLK / WAIT_PSC)
#define HALF_PERIOD 60000

#define SEUIL_BLANC	4
#define SEUIL_NOIR	8

volatile int puiss_MD = 0; //[0-100] puissance moteur droit
volatile int puiss_MG = 0; //[0-100] puissance moteur gauche
int sens_MD = 0; //[0,1] sens moteur droit
int sens_MG = 0; //[0,1] sens moteur gauche

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

volatile int un_sur_deux = 0;
volatile int TIM4_triggered = 0;
volatile enum{ D0 , R1 , R2 , R3 , L1 , L2 , L3 , SR , CLR , CLL , CT , CTR , CTL , CX} state = D0;
volatile int vitesse_m1 = 30;
volatile int vitesse_m2 = 30;
int KP = 3; //Constante Proportionnelle
volatile int P = 0 ; // Error
int KI = 0; //Constante Integrale
volatile float I = 0; //I = I + error (P)
volatile int KD = 0; //Constante Derivée
float D = 0; //D= error - previousError 

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
	for (short int i=0; i < 4; i++)                              
	{
		GPIOA_MODER = SET_BITS(GPIOA_MODER, i*2, 2, 0b01); //set GPIO A 
		GPIOA_OTYPER = GPIOA_OTYPER & ~(1 << i);
		GPIOA_PUPDR = SET_BITS(GPIOA_PUPDR, i*2, 2, 0b01);

		GPIOD_MODER = SET_BITS(GPIOD_MODER, i*2, 2, 0b01); //set GPIO D
		GPIOD_OTYPER = GPIOD_OTYPER & ~(1 << i);
		GPIOD_PUPDR = SET_BITS(GPIOD_PUPDR, i*2, 2, 0b01);
	}
}


void CAPTURE_START()
{
	for (short int i=0; i < 4; i++)                             
	{
		GPIOA_MODER = SET_BITS(GPIOA_MODER, i*2, 2, 0b00); //set GPIO A 
		GPIOA_OTYPER = GPIOA_OTYPER & ~(1 << i);
		GPIOA_PUPDR = SET_BITS(GPIOA_PUPDR, i*2, 2, 0b01);

		GPIOD_MODER = SET_BITS(GPIOD_MODER, i*2, 2, 0b00); //set GPIO D
		GPIOD_OTYPER = GPIOD_OTYPER & ~(1 << i);
		GPIOD_PUPDR = SET_BITS(GPIOD_PUPDR, i*2, 2, 0b01);
	}
	
}

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

void start_MD() {
	TIM3_CCR1 = TIM3_ARR;
}

void stop_MD() {
	TIM3_CCR1 = 0;
}

void set_MD(int pulse) {

	puiss_MD = pulse;
    TIM3_CCR1 = (pulse * TIM3_ARR / 100);
}

void inverseMD(){
    if(GPIOB_ODR & (1<<MD2)){
        GPIOB_BSRR = 1 << (MD2 + 16);
    }
    else{
        GPIOB_BSRR = 1 << MD2;

    }
}

void start_MG() {
	TIM3_CCR2 = TIM3_ARR;
}

void stop_MG() {
	TIM3_CCR2 = 0;
}

void set_MG(int pulse) {

	puiss_MG = pulse;
	TIM3_CCR2 = (pulse * TIM3_ARR / 100);
}

void inverseMG(){
    if(GPIOB_ODR & (1<<MG2)){
        GPIOB_BSRR = 1 << (MG2 + 16);
    }
    else{
        GPIOB_BSRR = 1 << MG2;
    }
}

void VITESSE_PID()
{
	printf("In PID\n");
	vitesse_m1 = vitesse_m1 -P;
	vitesse_m2 = vitesse_m2 +P;
	
	printf("Vitesse M1: %d, Vitesse M2 %d\n", vitesse_m1, vitesse_m2);
}


int main() {
	printf("\nStarting...\n");

	// RCC init
	RCC_AHB1ENR |= RCC_GPIOAEN;
	RCC_AHB1ENR |= RCC_GPIODEN;
	RCC_APB1ENR |= RCC_TIM4EN;
	RCC_APB2ENR |= RCC_ADC1EN;
	RCC_AHB1ENR |= RCC_GPIOBEN;
	RCC_APB1ENR |= RCC_TIM3EN;



	// initialization
	init();
    init_TIM4();
	init_moteurs();
    init_TIM3();

    if(GPIOB_ODR & (1<<MD2)){
        sens_MD = 0;
    }
    else{
        sens_MD = 1;
    }
    if(GPIOB_ODR & (1<<MG2)){
        sens_MG = 0;
    }
    else{
        sens_MG = 1;
    }

	// main loop
	printf("Endless loop!\n");
    short int x=0;
    CHARGE();
    short int vc[8] = { 12 , 12 , 12 , 12 , 12 , 12 , 12 , 12 };
	while(1) {
        if(TIM4_triggered){
			TIM4_triggered = 0;
			switch(state){
			//cas D0: le véhicule est aligné avec la ligne
			case D0 :
				    //set_MD(30);
    				//set_MG(30);
			   // mettre les roues à la même vitesse
			   
			   //accélérer les roues jusqu'à atteindre la vitesse max 
			    
				for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
					if(puiss_MD != puiss_MG)
					{
						if(puiss_MD < puiss_MG)
						{
							set_MG(puiss_MD);		
						}
						else
						{
							set_MD(puiss_MG);
						}
					}
					else
					{
						if(puiss_MG < 30)
						{
							set_MG(puiss_MG + 1);
							set_MD(puiss_MD + 1);
						}
					}

                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
                    CHARGE();

					if (vc[3] > SEUIL_NOIR && vc[4]< SEUIL_BLANC){
						P= -1;
					}else if (vc[4] > SEUIL_NOIR && vc[3]< SEUIL_BLANC){
						P= 1;
					}else if (vc[4] > SEUIL_NOIR && vc[5]> SEUIL_NOIR){
						P= 2;
					}else if (vc[3] > SEUIL_NOIR && vc[2]> SEUIL_NOIR){
						P = -2;
					}
					//VITESSE_PID();




					
                    if( vc[4] < SEUIL_NOIR)
                    {
                    	state = R1;
                    	printf("R1\n");
                    }
                    if( vc[3] < SEUIL_NOIR)
                    {
                    	state = L1;
                    	printf("L1\n");
                    }
					if(vc[3] < SEUIL_BLANC && vc[4] < SEUIL_BLANC)
					{
						if(vc[2] > SEUIL_NOIR || vc[1] > SEUIL_NOIR || vc[0] > SEUIL_NOIR)
						{
							state = R2;
                    		printf("R2\n");
						}
						if(vc[5] > SEUIL_NOIR || vc[6] > SEUIL_NOIR || vc[7] > SEUIL_NOIR)
						{
							state = L2;
                    		printf("L2\n");
						}
						if(vc[0] < SEUIL_BLANC && vc[1] < SEUIL_BLANC && vc[2] < SEUIL_BLANC && vc[5] < SEUIL_BLANC && vc[6] < SEUIL_BLANC && vc[7] < SEUIL_BLANC)
						{
							state = SR;
                    		printf("SR\n");
						}
					}

					/*if(vc[7] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CLR;
						printf("CR\n");
					}

					if(vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CLL;
						printf("CL\n");
					}

					if(vc[7] > SEUIL_NOIR && vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CT;
						printf("CT\n");
					}*/

					

					//passage à R2 R3 L2 L3	SR CLR CLL CT CTR CTL CX


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
			//cas R1: le véhicule est légèrement à droite de la ligne	
			case R1 :
			    //ralentir la roue gauche légèrement
			    for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
					if(puiss_MG > 0)
					{
						set_MG(puiss_MG - 1);
					}
                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
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
					/*if(vc[7] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CLR;
						printf("CR\n");
					}

					if(vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CLL;
						printf("CL\n");
					}

					if(vc[7] > SEUIL_NOIR && vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CT;
						printf("CT\n");
					}*/
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
			//cas R2: le véhicule est clairement décalé à droite de la ligne	
			case R2 :
			    //ralentir la roue gauche modérément
			    //accélérer la roue droite légèrement si possible
			    for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
					if(puiss_MG > 0)
					{
						set_MD(puiss_MD + 1);
						set_MG(puiss_MG - 1 );
					}
                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
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
			//cas R3: le véhicule est pas loin de sortir de la route car trop à droite de la ligne
			case R3 :
    			//ralentir la roue gauche fortement
			    //accélérer la roue droite fortement si possible
			    for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
					if(puiss_MG > 0)
					{
						set_MD(puiss_MD + 1);
						set_MG(0);
					}
                    x = 0;
                    CHARGE();
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
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
			//cas L1: le véhicule est légèrement à gauche de la ligne		
			case L1 :
			    //ralentir la roue droite légèrement			
                for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
					if(puiss_MD > 0)
					{
						set_MD(puiss_MD - 1);
					}
                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
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
					/*if(vc[7] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CLR;
						printf("CR\n");
					}

					if(vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CLL;
						printf("CL\n");
					}

					if(vc[7] > SEUIL_NOIR && vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CT;
						printf("CT\n");
					}*/
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
			//cas L2: le véhicule est clairement décalé à gauche de la ligne
			case L2 :
			    //ralentir la roue droite modérément
			    //accélérer la roue gauche légèrement si possible
			    for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
					if(puiss_MD > 0)
					{
						set_MG(puiss_MG + 1);
						set_MD(puiss_MD - 1);
					}
                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
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
			//cas R3: le véhicule est pas loin de sortir de la route car trop à gauche de la ligne
			case L3 :
			    //ralentir la roue droite fortement
			    //accélérer la roue gauche fortement si possible
			    for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
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
					if(puiss_MD > 0)
					{
						set_MG(puiss_MG + 1);
						set_MD(0);
					}
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
			//cas SR: sortie de route, le véhicule ne capte plus de ligne
			case SR :
			    //sortie de route
                state = D0;								//TODO
				break;
			
			//cas CLR: le véhicule détecte un croisement en L menant à droite
			case CLR :
			    //ralentir jusqu'à arret
				for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
                    CHARGE();
					if(vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
						{
							state = CT;
							printf("CT\n");
						}

					if(vc[7] < SEUIL_BLANC && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CTR;
						printf("CTR\n");
					}
					if( puiss_MD == 0 && puiss_MG == 0)
					{
						//interrogez mémoire sur direction à prendre
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
			
			//cas CLL: le véhicule détecte un croisement en L menant à gauche
			case CLL :
			    //ralentir jusqu'à arret
				for (int i=0; i < 4; i++)                              
					{
						if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
						{
							vc[i] = x;
						}
						if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
						{
							vc[i+4] = x;
						}
					}
					if (x == 11)
					{
						x = 0;
						printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
						CHARGE();
						if(vc[7] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
							{
								state = CT;
								printf("CT\n");
							}

						if(vc[0] < SEUIL_BLANC && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
						{
							state = CTR;
							printf("CTR\n");
						}
						if( puiss_MD == 0 && puiss_MG == 0)
						{
							//interrogez mémoire sur direction à prendre
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
			
			//cas CT: le véhicule détecte un croisement en T menant à gauche et à droite
			case CT :
			    //ralentir jusqu'à arret
				for (int i=0; i < 4; i++)                              
				{
					if((vc[i] > x) & ((GPIOA_IDR & (1 << i ))== 0))
					{
						vc[i] = x;
					}
					if((vc[i+4] > x) & ((GPIOD_IDR & (1 << i ))== 0))
					{
						vc[i+4] = x;
					}
				}
                if (x == 11)
                {
                    x = 0;
					printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",vc[0],vc[1],vc[2],vc[3],vc[4],vc[5],vc[6],vc[7]);
                    CHARGE();
					if(vc[7] < SEUIL_BLANC && vc[0] < SEUIL_BLANC && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
					{
						state = CX;
						printf("CX\n");
					}
					if( puiss_MD == 0 && puiss_MG == 0)
					{
						//interrogez mémoire sur direction à prendre
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
			
			//cas CTR: le véhicule détecte un croisement en T menant en face et à droite
			case CTR :
			    //ralentir jusqu'à arret
                if( puiss_MD == 0 && puiss_MG == 0)
				{
					//interrogez mémoire sur direction à prendre
				}
				break;
			
			//cas CTL: le véhicule détecte un croisement en T menant en face et à gauche
			case CTL :
			    //ralentir jusqu'à arret
                if( puiss_MD == 0 && puiss_MG == 0)
				{
					//interrogez mémoire sur direction à prendre
				}
				break;
			
			//cas CX: le véhicule détecte un croisement en X 
			case CX :
			    //ralentir jusqu'à arret
                if( puiss_MD == 0 && puiss_MG == 0)
				{
					//interrogez mémoire sur direction à prendre
				}
				break;
			}
		}
	}__asm("nop");
	return 0;
}
 
