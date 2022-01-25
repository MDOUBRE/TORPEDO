
// LED on GPIOD
#define ONOFF_LED		12
#define STARTSTOP_LED	13

// button on GPIOA
#define ONOFF_BUT		PA3
#define STARTSTOP_BUT	P4		

void init_ui() {

	// LEDs
	GPIOD_MODER = SET_BITS(GPIOD_MODER, 2*ONOFF_LED, 2, GPIOD_MODER_OUT);
	GPIOD_OTYPER ~= 1 << ONOFF_LED;
	GPIOD_PUPDR = SET_BITS(GPIOD_PUPDR, 2*ONOFF_LED, 2, GPIOD_PUPDR_PU);
	GPIOD_BSSR = (1 << (16 + ONOFF_LED));
	GPIOD_MODER = SET_BITS(GPIOD_MODER, 2*STARTSTOP_LED, 2, GPIOD_MODER_OUT);
	GPIOD_OTYPER ~= 1 << STARTSTOP_LED;
	GPIOD_PUPDR = SET_BITS(GPIOD_PUPDR, 2*STARTSTOP_LED, 2, GPIOD_PUPDR_PU);
	GPIOD_BSSR = (1 << (16 + STARTSTOP_LED));

	// buttons
	GPIOA_MODER = SET_BITS(GPIOA_MODER, 2*ONOFF_BUT, 2, GPIOD_MODER_IN);
	GPIOA_PUPDR = SET_BITS(GPIOA_PUPDR, 2*ONOFF_BUT, 2, GPIOD_PUPDR_PU);
	GPIOA_MODER = SET_BITS(GPIOA_MODER, 2*STARTSTOP_BUT, 2, GPIOD_MODER_IN);
	GPIOA_PUPDR = SET_BITS(GPIOA_PUPDR, 2*STARTSTOP_BUT, 2, GPIOD_PUPDR_PU);
}

void set_onoff_led(int s) {
	if(s == 1)
		GPIOD_BSRR = 1 << ONOFF_LED;
	else
		GPIOD_BSRR = 1 << (16 + ONOFF_LED);
}

void set_startstop_led(int s) {
	if(s == 1)
		GPIOD_BSRR = 1 << STARTSTOP_LED;
	else
		GPIOD_BSRR = 1 << (16 + STARTSTOP_LED);	
}

int test_onoff_button() {
	static int pushed = 0;
	if((GPIOA_IDR & (1 << ONOFF_BUT)) == 1)
		pushed = 1;
	else if(pushed) {
		pushed = 0;
		return 1;
	}
	return 0;
}

int test_startstop_button() {
	static int pushed = 0;
	if((GPIOA_IDR & (1 << STARTSTOP_BUT)) == 1)
		pushed = 1;
	else if(pushed) {
		pushed = 0;
		return 1;
	}
	return 0;
}


// Themo-resistror and pump (GPIOB)
// TIM3_CH1
#define THERM_PIN	4
// TIM3_CH2
#define PUMP_PIN	5
#define TIM3_DIV	7
#define TIM3_PERIOD	60000

// period 10ms
void init_TIM3() {

	// GPIOB
	GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*THERM_PIN, 2, GPIOD_MODER_ALT);
	GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*THERM_PIN, 4, 2);
	GPIOB_MODER = SET_BITS(GPIOB_MODER, 2*PUMP_PIN, 2, GPIOD_MODER_ALT);
	GPIOB_AFRL = SET_BITS(GPIOB_AFRL, 4*PUMP_PIN, 4, 2);

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

void start_heater() {
	TIM3_CC1R = TIM3_PERIOD;
}

void stop_heater() {
	TIM3_CC1R = 0;
}

void start_pump() {
	TIM3_CC2R = PUMP_PERIOD;
}

void stop_pump() {
	TIM3_CC2R = 0;
}

void set_heater(int n) {
	TIM3_CC1R = n * TIM3_PERIOD / 100;
}

void set_pump(int n) {
	TIM3_CC2R = n * TIM3_PERIOD / 100;
}


// ADC (GPIOA) (IM8)
// therm sensor - ADC1_IN1
#define THERM_SENS_PIN	1
// pressure sensor - ADC2_IN2
#define PRESS_SENS_PIN	2

#define KP_THERM
#define KI_THERM
#define KD_THERM
#define KP_PRESS
#define KI_PRESS
#define KD_PRESS

int ready = 0;

void init_sensors() {

	// GPIOA
	GPIOA_MODER = SET_BITS(GPIOA_MODER, THERM_SENS_PIN*2, 2, GPIO_MODER_ANA);
	GPIOA_MODER = SET_BITS(GPIOA_MODER, PRESS_SENS_PIN*2, 2, GPIO_MODER_ANA);

	// ADC
	ADC1_SR = 0;
	ADC1_CR1 = ADC_EOCIE;
	//ADC1_CR2 = ADC_T8TRGO;
	ADC1_SQR3 = 1;
	ADC2_SR = 0;
	ADC2_CR1 = ADC_EOCIE;
	//ADC2_CR2 = ADC_T8TRGO;
	ADC1_SQR3 = 2;
	
	// TIM4
	TIM4_CR1 = 0;
	TIM4_PSC = 1000;
	TIM4_ARR = APB1_CLK/1000/10;
	TIM4_CR1 = ADC_CEN;

	// NVIC
	NVIC_ICER(ID_ADC & 0x1f) =  ID_ADC >> 5;
	NVIC_IPR(ADC_IRQ) = 0;
	NVIC_IRQ(ADC_IRQ) = handle_adc;
	NVIC_ISER(ID_ADC & 0x1f) =  ID_ADC >> 5; 
}

void start_therm_pid() {
	ADC1_CR2 = ADC_T8TRGO;
}

void stop_therm_pid() {
	ADC1_CR2 = 0;
}

void start_pump_pid() {
	ADC2_CR2 = ADC_T8TRGO;
}

void stop_pump_pid() {
	ADC2_CR2 = 0;
}

void handle_adc() {

	if((ADC1_SR & ADC_EOC) != 0) {
		static int sum = 0, ep = 0;
		int e = ADC1_DC - THERM_REF;
		int u = KP_THERM * e + KI_THERM * sum + KD_THERM * (e - ep);
		if(u < 0)
			u = 0;
		if(u > 100)
			u = 100;
		set_heater(u);
		if(e <= 0)
			ready = 1;
	}

	if((ADC2_SR & ADC_EOC) != 0) {
		static int sum = 0, ep = 0;
		int e = ADC2_DC - PRESS_REF;
		int u = KP_PRESS * e + KI_PRESS * sum + KD_PRESS * (e - ep);
		if(u < 0)
			u = 0;
		if(u > 100)
			u = 100;
		set_pump(u); 		
	}
}


int main() {
	typedef enum {
		OFF,
		HEATING,
		READY,
		BREWING
	} state = OFF;

	/* initialization */
	init_ui();
	init_sensors();
	...

	/* endless loop */
	while(1) {
		switch(state) {

		case OFF:
			if(test_onoff_button() == 1) {
				state = HEATING;
				set_onoff_led(1);
				start_therm_pid();
				start_therm();
			}
			break;

		case HEATING:
			if(ready == 1)
				state = READY;
			if(test_onoff_button() == 1) {
				state = OFF;
				set_onoff_led(0);
				stop_therm_pid();
				stop_therm();				
			}
			break;

		case READY:
			if(test_onoff_button() == 1) {
				state = OFF;
				stop_therm_pid();
				stop_therm();
				ready = 0;
			}
			else if(test_startstop_button() == 1) {
				set_startstop_led_led(1);
				state =  BREWING;
				start_pump();
				start_pump_pid();
				...
			}
			break;

		case BREWING:
			if(test_startstop_button() == 1) {
				set_startstop_led_led(0);
				stop_pump();
				stop_pump_pid();
				state = READY;
			}
			break;
		}
	}
}
