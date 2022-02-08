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

#define WAIT_PSC 1  //T * 42MHz / 2¹⁶
#define WAIT_DELAY (APB1_CLK / WAIT_PSC)
#define HALF_PERIOD 42000

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

#define TAILLE_GRILLE

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
volatile enum{ N, E, S ,O} direction = E;
volatile int d = 0;
volatile short int demitour = 0;
volatile short int chemin[20] = { -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 };
volatile short int checkpoint[10] = {9 , 2  , 7 ,9 ,7 ,2 ,0 ,1 ,-1 , -1};


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

struct croisement
{
    int nord;
    int est;
    int ouest;
    int sud;
};
typedef struct croisement croisement;

int get_error(short int * array){

	unsigned char TB = 0b00000000;
	unsigned char TBT;
	for (short int i =0 ; i < 8; i++){
		TB = TB << 1;
		printf("%d\n", array[i]);
		if (array[i]> 6){
			TB = TB | 0b00000001;
		}
		
	}
	printf("%u\n",TB);
	if (!(TB ^ 0b00011000))
	{
		return 0;
	}else if(!(TB ^ 0b00001000))
	{
		return 1;
	}else if(!(TB ^ 0b00001100))
	{
		return 2;
	}else if(!(TB ^ 0b00000100))
	{
		return 3;
	}else if(!(TB ^ 0b00000110))
	{
		return 4;
	}else if(!(TB ^ 0b00000010))
	{
		return 5;
	}else if(!(TB ^ 0b00000011))
	{
		return 6;
	}else if(!(TB ^ 0b00000001))
	{
		return 7;
	}else if(!(TB ^ 0b00010000))
	{
		return -1;
	}else if(!(TB ^ 0b00110000))
	{
		return -2;
	}else if(!(TB ^ 0b00100000))
	{
		return -3;
	}else if(!(TB ^ 0b01100000))
	{
		return -4;
	}else if(!(TB ^ 0b01000000))
	{
		return -5;
	}else if(!(TB ^ 0b11000000))
	{
		return -6;
	}else if(!(TB ^ 0b10000000))
	{
		return -7;
	}else if(!(TB ^ 0b00000000))
	{
		return 10;
	}else
	{
		TBT = (TB & 0b11111000);
		if(!(TBT ^ 0b11111000) || !(TBT ^ 0b11110000) || !(TBT ^ 0b11101000) || !(TBT ^ 0b11011000) || !(TBT ^ 0b10111000) || !(TBT ^ 0b01111000)) 
		{
			TBT = (TB & 0b00011111);
			if(!(TBT ^ 0b00011111) || !(TBT ^ 0b00011110) || !(TBT ^ 0b00011101) || !(TBT ^ 0b00011011) || !(TBT ^ 0b00010111) || !(TBT ^ 0b000001111)) 
			{
				return 11;
			}
			return 12;
		}else 
		{
			TBT = (TB & 0b00011111);
			if(!(TBT ^ 0b00011111) || !(TBT ^ 0b00011110) || !(TBT ^ 0b00011101) || !(TBT ^ 0b00011011) || !(TBT ^ 0b00010111) || !(TBT ^ 0b000001111)) 
			{
				return 13;
			}
		}
	}
	return 0;
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
	if(puiss_MD >= 25)
	{
		set_MD(puiss_MD - 25);
	}
	else
	{
		set_MD(0);
	}
	if(puiss_MG >= 25)
	{
		set_MG(puiss_MG - 25);
	}
	else
	{
		set_MG(0);
	}
}

int calculer_chemin(short int start, short int end);

int choix_direction(int x, short int* Tab)
{
    if(x == 19)
    {
        d = d + 1;
        if(d == 10 || checkpoint[d+1] == -1) 
        {
            set_MD(0);
            set_MG(0);
            DISABLE_IRQS;
            return 0;
        }
        else
        {
            DISABLE_IRQS;
            x = calculer_chemin(checkpoint[d], checkpoint[d+1]);
            ENABLE_IRQS;
        }
    } 
    switch(direction)
        {
            case N:
                if(Tab[x] == 1)
                {
                    set_MD(0);
                    set_MG(0);
                    etat = LINE;
                    PAvant = 0;
                    I= 0;
                    direction = N;
                }
                else if(Tab[x] == 2)
                {
                    type = TR1;
                    direction = E;
                }
                else if(Tab[x] == 3)
                {
                    switch(type){
                        case X:
                            demitour = 1;
                            break;
                        case LR:
                            demitour = 1;
                            break;
                        case LL:
                            demitour = 0;
                            break;
                        case TR:
                            demitour = 1;
                            break;
                        case TL:
                            demitour = 0;
                            break;
                        case T:
                            demitour = 1;
                            break;
                    }
                    type = TR1;
                    direction = S;
                }
                else if(Tab[x] == 4)
                {
                    type = TL1;
                    direction = O;
                }
                break;
            case E:
                if(Tab[x] == 1)
                {
                    type = TL1;
                    direction = N;
                }
                else if(Tab[x] == 2)
                {
                    set_MD(0);
                    set_MG(0);
                    etat = LINE;
                    PAvant = 0;
                    I= 0;
                    direction = E;
                }
                else if(Tab[x] == 3)
                {
                    type = TR1;
                    direction = S;
                }
                else if(Tab[x] == 4)
                {
                    switch(type){
                        case X:
                            demitour = 1;
                            break;
                        case LR:
                            demitour = 1;
                            break;
                        case LL:
                            demitour = 0;
                            break;
                        case TR:
                            demitour = 1;
                            break;
                        case TL:
                            demitour = 0;
                            break;
                        case T:
                            demitour = 1;
                            break;
                    }
                    type = TR1;
                    direction = O;
                }
                break;
            case S:
                if(Tab[x] == 1)
                {
                    switch(type){
                        case X:
                            demitour = 1;
                            break;
                        case LR:
                            demitour = 1;
                            break;
                        case LL:
                            demitour = 0;
                            break;
                        case TR:
                            demitour = 1;
                            break;
                        case TL:
                            demitour = 0;
                            break;
                        case T:
                            demitour = 1;
                            break;
                    }
                    type = TR1;
                    direction = N;
                }
                else if(Tab[x] == 2)
                {
                    type = TL1;
                    direction = E;
                }
                else if(Tab[x] == 3)
                {
                    set_MD(0);
                    set_MG(0);
                    etat = LINE;
                    PAvant = 0;
                    I= 0;
                    direction = S;
                }
                else if(Tab[x] == 4)
                {
                    type = TR1;
                    direction = O;
                }
                break;
            case O:
                if(Tab[x] == 1)
                {
                    type = TR1;
                    direction = N;
                }
                else if(Tab[x] == 2)
                {
                    switch(type){
                        case X:
                            demitour = 1;
                            break;
                        case LR:
                            demitour = 1;
                            break;
                        case LL:
                            demitour = 0;
                            break;
                        case TR:
                            demitour = 1;
                            break;
                        case TL:
                            demitour = 0;
                            break;
                        case T:
                            demitour = 1;
                            break;
                    }
                    type = TR1;
                    direction = E;
                }
                else if(Tab[x] == 3)
                {
                    type = TL1;
                    direction = S;
                }
                else if(Tab[x] == 4)
                {
                    set_MD(0);
                    set_MG(0);
                    etat = LINE;
                    PAvant = 0;
                    I= 0;
                    direction = O;
                }
                break;
        }
    
    return x+1;
}

int calculer_chemin(short int start, short int end)
{
    croisement grille[20];
    short int cost[20] = { 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 , 20 };
    short int pred[20] = { -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 , -1 };

    cost[start] = 0;
    for (int i = 0; i<20 ;i++){
        chemin[i] = -1;
    }
    chemin[19] = end;
    
    int c = 19;
        // remplir

    grille[0].nord = 3;
    grille[0].est = 1;
    grille[0].sud = -1;
    grille[0].ouest = -1;
    grille[1].nord = 4;
    grille[1].est = -1;
    grille[1].sud = 17;
    grille[1].ouest = 0;
    grille[2].nord = 7;
    grille[2].est = 3;
    grille[2].sud = -1;
    grille[2].ouest = -1;
    grille[3].nord = 5;
    grille[3].est = 4;
    grille[3].sud = 0;
    grille[3].ouest = 2;
    grille[4].nord = 6;
    grille[4].est = -1;
    grille[4].sud = 1;
    grille[4].ouest = 3;
    grille[5].nord = 8;
    grille[5].est = 6;
    grille[5].sud = 3;
    grille[5].ouest = -1;
    grille[6].nord = 9;
    grille[6].est = -1;
    grille[6].sud = 4;
    grille[6].ouest = 5;
    grille[7].nord = -1;
    grille[7].est = 8;
    grille[7].sud = 2;
    grille[7].ouest = -1;
    grille[8].nord = -1;
    grille[8].est = 9;
    grille[8].sud = 5;
    grille[8].ouest = 7;
    grille[9].nord = 12;
    grille[9].est = 10;
    grille[9].sud = 6;
    grille[9].ouest = 8;
    grille[10].nord = 13;
    grille[10].est = 11;
    grille[10].sud = -1;
    grille[10].ouest = 9;
    grille[11].nord = 14;
    grille[11].est = -1;
    grille[11].sud = -1;
    grille[11].ouest = 10;
    grille[12].nord = 16;
    grille[12].est = 13;
    grille[12].sud = 9;
    grille[12].ouest = -1;
    grille[13].nord = -1;
    grille[13].est = 14;
    grille[13].sud = 10;
    grille[13].ouest = 12;
    grille[14].nord = -1;
    grille[14].est = 15;
    grille[14].sud = 11;
    grille[14].ouest = 13;
    grille[15].nord = -1;
    grille[15].est = -1;
    grille[15].sud = -1;
    grille[15].ouest = 14;
    grille[16].nord = -1;
    grille[16].est = -1;
    grille[16].sud = 12;
    grille[16].ouest = -1;
    grille[17].nord = 1;
    grille[17].est = -1;
    grille[17].sud = -1;
    grille[17].ouest = -1;
    grille[18].nord = -1;
    grille[18].est = -1;
    grille[18].sud = -1;
    grille[18].ouest = -1;
    grille[19].nord = -1;
    grille[19].est = -1;
    grille[19].sud = -1;
    grille[19].ouest = -1;


    for (int i=0; i < 20; i++) 
    {
        for (int j=0; j < 20; j++)
        {
            if(grille[j].nord != -1)
            {
                if(cost[grille[j].nord] + 1 < cost[j])
                {
                    cost[j] = cost[grille[j].nord] + 1; 
                    pred[j] = grille[j].nord;
                }
            }
            if(grille[j].est != -1)
            {
                if(cost[grille[j].est] + 1 < cost[j])
                {
                    cost[j] = cost[grille[j].est] + 1; 
                    pred[j] = grille[j].est;
                }
            }
            if(grille[j].sud != -1)
            {
                if(cost[grille[j].sud] + 1 < cost[j])
                {
                    cost[j] = cost[grille[j].sud] + 1; 
                    pred[j] = grille[j].sud;
                }
            }
            if(grille[j].ouest != -1)
            {
                if(cost[grille[j].ouest] + 1 < cost[j])
                {
                    cost[j] = cost[grille[j].ouest] + 1; 
                    pred[j] = grille[j].ouest;
                }
            }
        }
    }


    while(chemin[c] != start)
    {
        chemin[c-1] = pred[chemin[c]];
        printf("%d\n",chemin[c-1]);
        c = c-1;
    }

     for (int i=c; i < 19; i++) 
    {
        if(grille[chemin[i]].nord == chemin[i+1])
        {
            chemin[i] = 1;
        }
        if(grille[chemin[i]].est == chemin[i+1])
        {
            chemin[i] = 2;
        }
        if(grille[chemin[i]].sud == chemin[i+1])
        {
            chemin[i] = 3;
        }
        if(grille[chemin[i]].ouest == chemin[i+1])
        {
            chemin[i] = 4;
        }
    }
    printf("%d ; %d ; %d ; %d ; %d ; %d ; %d ; %d\n",chemin[12],chemin[13],chemin[14],chemin[15],chemin[16],chemin[17],chemin[18],chemin[19]);
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

    // calcul chemin
    
    short int start = checkpoint[d];
    short int end = checkpoint[d+1];

    short int c = calculer_chemin(start,end);
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
                    if (vc[3] > SEUIL_NOIR || vc[4] > SEUIL_NOIR ){
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
                                c = choix_direction(c, chemin);
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
                                c = choix_direction(c, chemin);
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
                                c = choix_direction(c, chemin);
                            }
                            break;
                        case X:
                            printf("CASE X\n");
                            SMOOTHSTOP();
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                c = choix_direction(c, chemin);
                            }
                            break;
                        case TL:
                            printf("CASE TL\n");
                            SMOOTHSTOP();
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                c = choix_direction(c, chemin);
                            }
                            break;
                        case TR:
                            printf("CASE TR\n");
                            SMOOTHSTOP();
                            if( puiss_MD == 0 && puiss_MG == 0)
                            {
                                c = choix_direction(c, chemin);
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
                            if(vc[3] < SEUIL_BLANC && vc[4] < SEUIL_BLANC && vc[2] < SEUIL_BLANC && vc[5] < SEUIL_BLANC )
                            {
                                type = TR3;
                                //printf("TR3\n");
                            }
                            break;
                        case TR3:
                            printf("CASE TR3\n");
                            if(vc[3] > SEUIL_NOIR || vc[4] > SEUIL_NOIR )
                            {
                                if (demitour){
                                    demitour= 0;
                                    inverseMD();
                                    type = TR1;
                                }else{
                                    inverseMD();
                                    set_MD(0);
                                    set_MG(0);
                                    etat = LINE;
                                    PAvant = 0;
                                    I= 0;
                                }
                                
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
                            if(vc[3] < SEUIL_BLANC && vc[4] < SEUIL_BLANC && vc[2] < SEUIL_BLANC && vc[5] < SEUIL_BLANC)
                            {
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
 
