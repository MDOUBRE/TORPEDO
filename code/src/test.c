#include <tinyprintf.h>
#include <stm32f4/rcc.h>
#include <stm32f4/gpio.h>
#include <stm32f4/nvic.h>
#include <stm32f4/exti.h>
#include <stm32f4/syscfg.h>
#include <stm32f4/tim.h>
#include <stm32f4/adc.h>

#define M1P1 4
#define M1P2 5
#define M2P1 6
#define M2P2 7
#define MODE 8

int main(){
    GPIOD_MODER = SET_BITS(GPIOD_MODER, 2*MODE, 2, 0b01);
    GPIOD_OTYPER &= ~(1 << MODE);
    GPIOD_PUPDR = SET_BITS(GPIOD_PUPDR, 2*MODE, 2, 0b01);

    GPIOD_BSRR = 1 << MODE;
    while(1){

    }__asm("nop");
    return 0;
}