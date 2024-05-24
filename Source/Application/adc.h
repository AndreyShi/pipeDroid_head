#ifndef ADC_H
#define ADC_H

#define AOUT1 ADC_CHANNEL_8 
#define AOUT2 ADC_CHANNEL_9 
#define AOUT3 ADC_CHANNEL_5 
#define AOUT4 ADC_CHANNEL_6 

extern uint16_t adc_buff[24][360];
extern int iab;

void adc_init(void);
uint16_t adc_channel_sample(uint8_t channel);
uint16_t adc_wait_result(void);
float adc_wait_result_f(void);
float adc_channel_sample_f(uint8_t channel);
void dma_config(uint32_t adc,uint8_t ADCchannel,uint32_t dma,uint8_t dma_ch,uint16_t* buff,uint8_t dma_it);
void dma_reconfig(uint32_t adc,uint8_t ADCchannel,uint32_t dma,uint8_t dma_ch,uint16_t* buff);
void adc_main_algorithm(void);
#endif