#ifndef ADC_H
#define ADC_H

#define AOUT1 ADC_CHANNEL_8 
#define AOUT2 ADC_CHANNEL_9 
#define AOUT3 ADC_CHANNEL_5 
#define AOUT4 ADC_CHANNEL_6 

void adc_init(void);
uint16_t adc_channel_sample(uint8_t channel);
float adc_channel_sample_f(uint8_t channel);
void dma_config(void);
#endif