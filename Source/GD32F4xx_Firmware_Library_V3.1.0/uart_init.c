#include "gd32f4xx_libopt.h"

void uart0_init_gd(void){
    usart_baudrate_set(USART0, 115200U);
}