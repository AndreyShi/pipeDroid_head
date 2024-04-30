#include "../GD32F4xx_Firmware_Library_V3.1.0/gd32f4xx_libopt.h"
#include "adc.h"
#define V_REF_EXT_21_PIN 2.505F
#define ADC_RESOLUTION12BIT 4096
uint16_t data_adc;
float volt_adc;
void adc_init(){
    
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    adc_clock_config(ADC_ADCCK_PCLK2_DIV4); //HCLK 200 Mhz/2 (APB2presc) = PCLK2 100Mhz/4 = 25 Mhz

    /* config the GPIO as analog mode */
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0);// ADC01 PB0 AOUT1
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_1);// ADC01 PB1 AOUT2
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_5);// ADC01 PA5 AOUT3
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_6);// ADC01 PA6 AOUT4

    /* reset ADC */
    adc_deinit();
    /* ADC mode config */
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    /* ADC contineous function disable */
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC continous function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC_EXTTRIG_ROUTINE_T0_CH0);
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);

    /* ADC temperature and Vref enable */
    //adc_channel_16_to_18(ADC_TEMP_VREF_CHANNEL_SWITCH,ENABLE);

    /* enable ADC interface */
    adc_enable(ADC0);
    /* wait for ADC stability */
    Delay_ms(2);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    //volt_adc = adc_channel_sample(ADC_CHANNEL_8) * V_REF_EXT_21_PIN / ADC_RESOLUTION12BIT;
}

/*!
    \brief      ADC channel sample
    \param[in]  
    ADC_CHANNEL_8 (PB0 AOUT1),
    ADC_CHANNEL_9 (PB1 AOUT2),
    ADC_CHANNEL_5 (PA5 AOUT3),
    ADC_CHANNEL_6 (PA6 AOUT4)
    \retval     uint16_t adc_data
*/
uint16_t adc_channel_sample(uint8_t channel)
{
    /* ADC routine channel config */
    adc_routine_channel_config(ADC0, 0U, channel, ADC_SAMPLETIME_3);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC0, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_routine_data_read(ADC0));
}

/*!
    \brief      ADC channel sample
    \param[in]  
    ADC_CHANNEL_8 (PB0 AOUT1),
    ADC_CHANNEL_9 (PB1 AOUT2),
    ADC_CHANNEL_5 (PA5 AOUT3),
    ADC_CHANNEL_6 (PA6 AOUT4)
    \retval     float adc_volts
*/
float adc_channel_sample_f(uint8_t channel)
{
    return adc_channel_sample(channel) * V_REF_EXT_21_PIN / ADC_RESOLUTION12BIT;
}