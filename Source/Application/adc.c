#include "../GD32F4xx_Firmware_Library_V3.1.0/gd32f4xx_libopt.h"
#include "adc.h"

void adc_init(){
    /* ADC mode config */
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    /* ADC continous function enable */
    //adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
   // adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 4);
    /* ADC routine channel config */
    adc_routine_channel_config(ADC0, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_15);
   // adc_routine_channel_config(ADC0, 1, ADC_CHANNEL_5, ADC_SAMPLETIME_15);
   //adc_routine_channel_config(ADC0, 2, ADC_CHANNEL_6, ADC_SAMPLETIME_15);
    //adc_routine_channel_config(ADC0, 3, ADC_CHANNEL_7, ADC_SAMPLETIME_15);
    /* ADC trigger config */
    //adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC_EXTTRIG_ROUTINE_EXTI_11);
    //adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    /* ADC discontinuous mode */
    //adc_discontinuous_mode_config(ADC0, ADC_ROUTINE_CHANNEL, 1);

    /* enable ADC interface */
    adc_enable(ADC0);
    /* wait for ADC stability */
    Delay_ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
    while(ADC_STAT(ADC0) & ADC_STAT_EOC){};    //waitEOC
    adc_routine_data_read(ADC0);
}