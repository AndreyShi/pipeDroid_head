#include "../GD32F4xx_Firmware_Library_V3.1.0/gd32f4xx_libopt.h"
#include "mux.h"

/*configurate spi and gpio for mux control*/
void spi_mux_config(void)
{
    rcu_periph_clock_enable(RCU_SPI2);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOB);

    mux_set_sync(0);
    mux_reset(1);
    /* configure SPI2 GPIO */
    gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_10 | GPIO_PIN_12 );
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10 | GPIO_PIN_12 );
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_12 );
    /* set MUX SYNC as GPIO*/
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    /* set MUX RESET as GPIO*/
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    spi_parameter_struct spi_init_struct;

    /* configure SPI2 parameter */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_BDTRANSMIT;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_16;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;

    spi_init(SPI2, &spi_init_struct);
    spi_enable(SPI2);
    mux_reset(0);
	
}
/*
change level on SYNC pin ADG714
char on - 1 - set low level (data reg is ready), 0 - set high level
*/
void mux_set_sync(char on)
{
    if(on)
        { 
            gpio_bit_reset(GPIOC,GPIO_PIN_11);
            __asm("nop");
            __asm("nop");
            __asm("nop");
        }
    else
        { 
            wait_spi();
            __asm("nop");
            __asm("nop");
            __asm("nop");
            gpio_bit_set(GPIOC,GPIO_PIN_11);
        }
}
/*
change level on RESET pin ADG714
char on - 1 - set low level (reset data reg to 0), 0 - set high level
*/
void mux_reset(char on)
{
    if(on)
        { gpio_bit_reset(GPIOB,GPIO_PIN_10);}
    else
        { gpio_bit_set(GPIOB,GPIO_PIN_10);}
}

void spi_mux_send(unsigned char byte)
{   
    spi_i2s_data_transmit(SPI2, byte);
    //while(!spi_i2s_flag_get(SPI2, SPI_FLAG_TBE));    // only empty buffer
    while(spi_i2s_flag_get(SPI2, SPI_FLAG_TRANS)); // full transfer
}

char wait_spi(void)
{
    while(spi_i2s_flag_get(SPI2, SPI_FLAG_TRANS)); // full transfer
    return 1;
}