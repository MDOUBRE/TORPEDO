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

#define WAIT_PSC 4  //T * 42MHz / 2¹⁶
#define WAIT_DELAY (APB1_CLK / WAIT_PSC)
#define HALF_PERIOD 52500

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
//volatile enum{ D0 , SR , CLR , CLL , CT , CTR , CTL , CX , TL1 , TL2 , TL3 , TR1 , TR2 , TR3} state = D0;
volatile int KP = 144; //Constante Proportionnelle
volatile int P = 0 ; // Error
volatile int KI = 1; //Constante Integrale
volatile int I = 0; //I = I + error (P)
volatile int KD = 108; //Constante Derivée
volatile int D = 0; //D= error - previousError 
volatile int PAvant = 0; // Previous error
volatile int PID = 0;
volatile enum{ OUT,LINE,INTERSECTION} etat = OUT;
volatile enum{ T,LL,LR,X,TL,TR,TR1,TR2,TR3,TL1,TL2,TL3} type = T;

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

int stringCompare(char * first, char * second){
	for (int i = 0; i < 8; i++) {
		if(first[i]!=second[i]){
			return 0;
		}
	}
	return 1;
}
int compare_line(short int * array){
    char string[8];
	for (short int i =0 ; i < 8; i++){
		if (array[i]< 6){
			string[i] = '0';
		}else{
			string[i] = '1';
		}
	}
    if (stringCompare(string,"00011000")){
		return 1;
	}else{
        return 0;
    }
}



int get_error(short int * array){
	char string[8];
	for (short int i =0 ; i < 8; i++){
		if (array[i]< 6){
			string[i] = '0';
		}else{
			string[i] = '1';
		}
	}
	if (stringCompare(string,"00011000")){
		return 0;
	}else if(stringCompare(string,"00001000")){
		return 1;
	}else if(stringCompare(string,"00001100")){
		return 2;
	}else if(stringCompare(string,"00000100")){
		return 3;
	}else if(stringCompare(string,"00000110")){
		return 4;
	}else if(stringCompare(string,"00000010")){
		return 5;
	}else if(stringCompare(string,"00000011")){
		return 6;
	}else if(stringCompare(string,"00000001")){
		return 7;
	}else if(stringCompare(string,"00010000")){
		return -1;
	}else if(stringCompare(string,"00110000")){
		return -2;
	}else if(stringCompare(string,"00100000")){
		return -3;
	}else if(stringCompare(string,"01100000")){
		return -4;
	}else if(stringCompare(string,"01000000")){
		return -5;
	}else if(stringCompare(string,"11000000")){
		return -6;
	}else if(stringCompare(string,"10000000")){
		return -7;
	}else if(stringCompare(string,"00000000")) {
		return 10;
	}else if(stringCompare(string,"11111111")  || stringCompare(string,"10111111") || stringCompare(string,"10011111")  || 
    stringCompare(string,"11111101") || stringCompare(string,"11111001") || stringCompare(string,"11011111") || stringCompare(string,"11111011") ||
    stringCompare(string,"10111101") || stringCompare(string,"10011101") || stringCompare(string,"10111001") || stringCompare(string,"11011011")) {
		return 11; //croisement en T
    }else if(stringCompare(string,"11111000") || stringCompare(string,"11110000")) {
		return 12; //croisement en L gauche
    }else if(stringCompare(string,"00011111") || stringCompare(string,"00001111")) {
		return 13; //croisement en L droite
	}else{
        return 0;
    }
	
}

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
    TIM3_CCR1 =pulse;
	//TIM3_CCR1 = 0;
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
	TIM3_CCR2 = pulse;//TIM_ARR = 20k, TIM_CCR2 va de 0 a 20k
	//TIM3_CCR2 = 0;
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
	puiss_MD = 1080 -PID; //30 % de 3600
	if (puiss_MD <0){
		puiss_MD = 0;
	}
	puiss_MG = 1980 - puiss_MD;
	set_MD(puiss_MD);
	set_MG(puiss_MG);
	
	printf("Vitesse M1: %d, Vitesse M2 %d\n", puiss_MD, puiss_MG);
}

void SMOOTHSTOP()
{
	if(puiss_MD >= 100)
	{
		set_MD(puiss_MD - 100);
	}
	else
	{
		set_MD(0);
	}
	if(puiss_MG >= 100)
	{
		set_MG(puiss_MG - 100);
	}
	else
	{
		set_MG(0);
	}
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
    int error = 0;
    CHARGE();
    short int vc[8] = { 12 , 12 , 12 , 12 , 12 , 12 , 12 , 12 };
	set_MG(30);
	set_MD(30);
	while(1) {
        if(TIM4_triggered){
			TIM4_triggered = 0;
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
                switch(etat){
                //cas D0: le véhicule est "aligné" avec la ligne
                case OUT :
					printf("CASE OUT");
                    if (compare_line(vc)){
                        etat = LINE;
						PAvant = 0;
						I = 0;
                        break;
                    }
                    stop_MD();
                    stop_MG();
                    break;
                case LINE:
					printf("CASE LINE");
                    error = get_error(vc);
                    switch(error){
                        case 10:
                            etat = OUT;
                            stop_MD();
                            stop_MG();
                            break;
                        case 11:
                            etat = INTERSECTION;
                            type = T;
                            stop_MD();
                            stop_MG();
                            break;
                        case 12:
                            etat = INTERSECTION;
                            type = LL;
                            stop_MD();
                            stop_MG();
                            break;
                        case 13:
                            etat = INTERSECTION;
                            type = LR;
                            stop_MD();
                            stop_MG();
                            break;
                        default:
                            P = error;
                            I = I + P;
                            D = P - PAvant;
                            PID = (KP*P) +(KI*I)+ (KD*D);
                            PAvant = P;
                            printf("P: %d, result get_error: %d\n",P,get_error(vc));
                            VITESSE_PID();
                            break;
                    }
                    break;
                case INTERSECTION:
                    printf("CASE INTERSECTION\n");
                    
                    switch(type){
                        case T:
                            printf("CASE T\n");
                            SMOOTHSTOP();
                            if(vc[7] < SEUIL_BLANC && vc[0] < SEUIL_BLANC && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
                            {
                                type = X;
                                //printf("CX\n");
                            }
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                //interrogez mémoire sur direction à prendre
                                type = TR1;
                            }
                            break;
                        case LL:
                            printf("CASE LL\n");
                            SMOOTHSTOP();
                            if(vc[7] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
                            {
                                type = T;
                                //printf("CT\n");
                            }

                            if(vc[0] < SEUIL_BLANC && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
                            {
                                type = TL;
                                //printf("CTL\n");
                            }
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                //interrogez mémoire sur direction à prendre
                                type = TL1;
                            }
                            break;
                        case LR:
                            printf("CASE LR\n");
                            SMOOTHSTOP();
                            if(vc[0] > SEUIL_NOIR && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
                            {
                                type = T;
                                //printf("CT\n");
                            }

                            if(vc[7] < SEUIL_BLANC && (vc[3] > SEUIL_BLANC || vc[4] > SEUIL_BLANC))
                            {
                                type = TR;
                                //printf("CTR\n");
                            }
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                //interrogez mémoire sur direction à prendre
                                type = TR1;
                            }
                            break;
                        case X:
                            printf("CASE X\n");
                            SMOOTHSTOP();
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                //interrogez mémoire sur direction à prendre
                                type = TL1;
                            }
                            break;
                        case TL:
                            printf("CASE TL\n");
                            SMOOTHSTOP();
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                //interrogez mémoire sur direction à prendre
                                type = TL1;
                                //printf("TL1\n");
                            }
                            break;
                        case TR:
                            printf("CASE TR\n");
                            SMOOTHSTOP();
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                type = TR1;
                                //interrogez mémoire sur direction à prendre
                            }
                            break;
                        case TR1:
                            printf("CASE TR1\n");
                            inverseMD();
                            set_MD(1000);
                            set_MG(850);
                            type = TR2;
                            break;
                        case TR2:
                            printf("CASE TR2\n");
                            if(vc[3] < SEUIL_BLANC && vc[4] < SEUIL_BLANC)
                            {
                                type = TR3;
                                //printf("TR3\n");
                            }
                            break;
                        case TR3:
                            printf("CASE TR3\n");
                            if(vc[3] > SEUIL_NOIR || vc[4] > SEUIL_NOIR )
                            {
                                inverseMD();
                                set_MD(0);
                                set_MG(0);
                                etat = LINE;
                                PAvant = 0;
                                I= 0;
                                //printf("D0\n");
                            }
                            break;
                        case TL1:
                            printf("CASE TL1\n");
                            inverseMG();
                            set_MD(850);
                            set_MG(1000);
                            type = TL2;
                            break;
                        case TL2:
                            printf("CASE TL2\n");
                            if(vc[3] < SEUIL_BLANC && vc[4] < SEUIL_BLANC)
                            {
                                //interrogez mémoire sur direction à prendre
                                type = TL3;
                                //printf("TL3\n");
                            }
                            break;
                        case TL3:
                            printf("CASE TL3\n");
                            if(vc[3] > SEUIL_NOIR || vc[4] > SEUIL_NOIR )
                            {
                                inverseMG();
                                set_MD(0);
                                set_MG(0);
                                etat = LINE;
                                PAvant = 0;
                                I= 0;
                                //printf("D0\n");
                            }
                            break;
                        
                    }
                    break;
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
		}
	}__asm("nop");
	return 0;
}
 
