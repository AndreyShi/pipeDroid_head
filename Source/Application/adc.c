#include "../GD32F4xx_Firmware_Library_V3.1.0/gd32f4xx_libopt.h"
#include "adc.h"
#include "mux.h"

#ifdef __BSD_VISIBLE
#undef __BSD_VISIBLE
#define __BSD_VISIBLE 1
#endif

#include "math.h"
//#include "stm32f4xx.h" при подключении stm файла не работает adc

#define V_REF_EXT_21_PIN 2.505F
#define ADC_RESOLUTION12BIT 4096
uint16_t data_adc;
float volt_adc;

uint16_t adc_buff[24][360];
int iab;
char flag_dma_finish;
float sin_t[91];
float ampl[24][2]; // [AIN0-23][0 - 300Hz 1 - 600Hz]

 static void init_sin(void);
 static float sin_fast(int a);
 static float cos_fast(int a);
 static float CalcHarm(uint16_t* signal, int size); 

void adc_init(){
    
    for(int i = 0; i < 24;i++)
    {
        for(int j = 0; j < 360;j++)
        {
             adc_buff[i][j] = 0xffff;
        }
    }
    init_sin();
    
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    // автоматически устанавливается AHB или APB2
    //HCLK 200 Mhz/2 (APB2presc) = PCLK2 100Mhz/4 = 25 Mhz
    //HCLK 200 Mhz/2 (APB2presc) = PCLK2 100Mhz/8 = 12,5 Mhz
    adc_clock_config(ADC_ADCCK_PCLK2_DIV8); 

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
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
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
        /* ADC DMA function enable */
    adc_dma_request_after_last_enable(ADC0);
    adc_dma_mode_enable(ADC0);

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

    return adc_wait_result();
}

uint16_t adc_wait_result(void)
{
    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC0, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_routine_data_read(ADC0));
}

float adc_wait_result_f(void)
{    
    return adc_wait_result() * V_REF_EXT_21_PIN / ADC_RESOLUTION12BIT;
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
/*
uint32_t adc       - переферия ADC0, ADC1
uint8_t ADCchannel - канал ADC_CHANNEL_8 (PB0 AOUT1),ADC_CHANNEL_9 (PB1 AOUT2),ADC_CHANNEL_5 (PA5 AOUT3),ADC_CHANNEL_6 (PA6 AOUT4)
uint32_t dma       - переферия DMA0, DMA1
uint8_t dma_ch     - канал DMA_CH0, DMA_CH1
uint16_t* buff     - буфер для приема
uint8_t dma_it     - (1)активировать прерывание по окончанию пересылки в буфер или нет(0)
*/
void dma_config(uint32_t adc,uint8_t ADCchannel,uint32_t dma,uint8_t dma_ch,uint16_t* buff,uint8_t dma_it)
{
        /* enable DMA clock */
    if(dma == DMA0)
        { rcu_periph_clock_enable(RCU_DMA0);}
    else if(dma == DMA1)
        { rcu_periph_clock_enable(RCU_DMA1);}
    /* ADC_DMA_channel configuration */
    dma_single_data_parameter_struct dma_single_data_parameter;

    /* ADC DMA_channel configuration */
    dma_deinit(dma, dma_ch);

    /* initialize DMA single data mode */
    dma_single_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(adc));
    dma_single_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_single_data_parameter.memory0_addr = (uint32_t)(buff);
    dma_single_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_single_data_parameter.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_single_data_parameter.direction = DMA_PERIPH_TO_MEMORY;
    dma_single_data_parameter.number = 360;
    dma_single_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(dma, dma_ch, &dma_single_data_parameter);
    dma_channel_subperipheral_select(dma, dma_ch, DMA_SUBPERI0);

    if(dma_it == 1)
    {
        /* Enable the DMA Interrupt */
        dma_interrupt_enable(dma, dma_ch, DMA_CHXCTL_FTFIE);    
        //nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
        nvic_irq_enable(DMA1_Channel0_IRQn, 0, 0);  // запускаются прерывания в которых нужно очищать флаг (в STM файле это DMA2_Stream0_IRQHandler)
    }
    /* disable DMA circulation mode */
    dma_circulation_disable(dma, dma_ch);
    //dma_circulation_enable(dma, dma_ch);
    /* enable DMA channel */
    dma_channel_enable(dma, dma_ch);
    //adc 12.5 MHZ * ADC_SAMPLETIME_112 = 0.00000896 Sec = 111607 Hz for signal 300 Hz
    adc_routine_channel_config(adc, 0U, ADCchannel, ADC_SAMPLETIME_112); 
    /* ADC software trigger enable */
    adc_software_trigger_enable(adc, ADC_ROUTINE_CHANNEL);
}

void dma_reconfig(uint32_t adc,uint8_t ADCchannel,uint32_t dma,uint8_t dma_ch,uint16_t* buff,uint32_t sam_t)
{
    dma_flag_clear(dma, dma_ch, DMA_FLAG_FTF);
    dma_flag_clear(dma, dma_ch, DMA_FLAG_HTF);
    dma_channel_disable(dma, dma_ch);

    adc_special_function_config(adc, ADC_CONTINUOUS_MODE, DISABLE);
    adc_dma_mode_disable(adc); 
    adc_dma_mode_enable(adc);           
    adc_flag_clear(adc, ADC_FLAG_EOC | ADC_FLAG_STRC | ADC_FLAG_ROVF);
    adc_special_function_config(adc, ADC_CONTINUOUS_MODE, ENABLE);
        //adc 12.5 MHZ * ADC_SAMPLETIME_112 = 0.00000896 Sec = 111607 Hz for signal 300 Hz
        //adc 12.5 MHZ * ADC_SAMPLETIME_56  = 0.00000448 Sec = 223214 Hz for signal 600 Hz
    adc_routine_channel_config(adc, 0U, ADCchannel, sam_t); 
    
    dma_periph_address_config(dma, dma_ch, (uint32_t)(&ADC_RDATA(adc)));
    dma_memory_address_config(dma, dma_ch, dma == DMA_CH1 ? 1 : 0, (uint32_t)buff);
    dma_transfer_number_config(dma, dma_ch, 360);
    dma_channel_enable(dma, dma_ch);
    adc_flag_clear(adc, ADC_FLAG_EOC | ADC_FLAG_STRC | ADC_FLAG_ROVF);
    /* ADC software trigger enable */
    adc_software_trigger_enable(adc, ADC_ROUTINE_CHANNEL);
}
//за счет этой последовательности чередуются переключатели тем самым дается пауза между переключениями одного канала 
//3 * 3,2 = 9,6 мс для 300 Hz преобразования
//3 * 1,6 = 4,8 мс для 600 Hz преобразования
int adc_sec[24] = {0,4,12,18, 1,5,13,19, 2,8,14,20, 3,9,15,21, 6,10,16,22, 7,11,17,23};
int freq300_600 = 0;// 0 - 300 Hz  1 - 600 Hz
uint32_t sam_t = ADC_SAMPLETIME_112;

void adc_main_algorithm(void)
{
	if(dma_flag_get(DMA1, DMA_CH0, DMA_FLAG_FTF) == SET || flag_dma_finish == SET)
	{  
        flag_dma_finish = RESET;
        iab++;
        if(iab >= 24)
            {
                set_muxes("\x00\x00\x00\x00\x00\x00");//set all reset
                iab = 0;
                __asm("nop");
                for(int i = 0 ; i < 24;i++)
                    { ampl[i][freq300_600] = CalcHarm(adc_buff[i], 360);}
                if(freq300_600 == 0)
                {
                    sam_t = ADC_SAMPLETIME_56;
                    freq300_600 = 1;
                }else{
                    sam_t = ADC_SAMPLETIME_112;
                    freq300_600 = 0;
                }
            }

        int iabs = adc_sec[iab];
        
        if(iabs == 0)
        {
            set_muxes("\x00\x00\x00\x00\x00\x30");// only on AIN0 for AOUT1
	        dma_reconfig(ADC0,AOUT1,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 1){
            set_muxes("\x00\x00\x00\x00\x00\xC0");// only on AIN1 for AOUT1
            dma_reconfig(ADC0,AOUT1,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 2){
            set_muxes("\x00\x00\x00\x00\x00\x0C");// only on AIN2 for AOUT1
            dma_reconfig(ADC0,AOUT1,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 3){
            set_muxes("\x00\x00\x00\x00\x00\x03");// only on AIN3 for AOUT1
            dma_reconfig(ADC0,AOUT1,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 4){
            set_muxes("\x00\x00\x00\x00\xC0\x00");// only on AIN4 for AOUT2
            dma_reconfig(ADC0,AOUT2,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 5){
            set_muxes("\x00\x00\x00\x00\x30\x00");// only on AIN5 for AOUT2
            dma_reconfig(ADC0,AOUT2,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 6){
            set_muxes("\x00\x00\x00\x00\x0C\x00");// only on AIN6 for AOUT1
            dma_reconfig(ADC0,AOUT1,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 7){
            set_muxes("\x00\x00\x00\x00\x03\x00");// only on AIN7 for AOUT1
            dma_reconfig(ADC0,AOUT1,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 8){
            set_muxes("\x00\x00\x00\x0C\x00\x00");// only on AIN8 for AOUT2
            dma_reconfig(ADC0,AOUT2,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 9){
            set_muxes("\x00\x00\x00\x03\x00\x00");// only on AIN9 for AOUT2
            dma_reconfig(ADC0,AOUT2,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 10){
            set_muxes("\x00\x00\x00\xC0\x00\x00");// only on AIN10 for AOUT2
            dma_reconfig(ADC0,AOUT2,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 11){
            set_muxes("\x00\x00\x00\x30\x00\x00");// only on AIN11 for AOUT2
            dma_reconfig(ADC0,AOUT2,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 12){
            set_muxes("\x30\x00\x00\x00\x00\x00");// only on AIN12 for AOUT4
            dma_reconfig(ADC0,AOUT4,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 13){
            set_muxes("\xC0\x00\x00\x00\x00\x00");// only on AIN13 for AOUT4
            dma_reconfig(ADC0,AOUT4,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 14){
            set_muxes("\x0C\x00\x00\x00\x00\x00");// only on AIN14 for AOUT4
            dma_reconfig(ADC0,AOUT4,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 15){
            set_muxes("\x03\x00\x00\x00\x00\x00");// only on AIN15 for AOUT4
            dma_reconfig(ADC0,AOUT4,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 16){
            set_muxes("\x00\xC0\x00\x00\x00\x00");// only on AIN16 for AOUT4
            dma_reconfig(ADC0,AOUT4,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 17){
            set_muxes("\x00\x30\x00\x00\x00\x00");// only on AIN17 for AOUT4
            dma_reconfig(ADC0,AOUT4,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 18){
            set_muxes("\x00\x0C\x00\x00\x00\x00");// only on AIN18 for AOUT3
            dma_reconfig(ADC0,AOUT3,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 19){
            set_muxes("\x00\x03\x00\x00\x00\x00");// only on AIN19 for AOUT3
            dma_reconfig(ADC0,AOUT3,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 20){
            set_muxes("\x00\x00\xC0\x00\x00\x00");// only on AIN20 for AOUT3
            dma_reconfig(ADC0,AOUT3,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 21){
            set_muxes("\x00\x00\x30\x00\x00\x00");// only on AIN21 for AOUT3
            dma_reconfig(ADC0,AOUT3,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 22){
            set_muxes("\x00\x00\x03\x00\x00\x00");// only on AIN22 for AOUT3
            dma_reconfig(ADC0,AOUT3,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }else if(iabs == 23){
            set_muxes("\x00\x00\x0C\x00\x00\x00");// only on AIN23 for AOUT3
            dma_reconfig(ADC0,AOUT3,DMA1,DMA_CH0,adc_buff[iabs],sam_t);
        }
    }  
	//set_muxes("\x00\x00\x0C\x00\x00\x00");// only on AIN23 for AOUT3 ok

	//set_muxes("\xC0\x00\x00\x00\x00\x00");// only on AIN13 for AOUT4 ok
	//  set_muxes("\xC0\x00\x0C\x00\x30\x03");
	//подготовить функции для включения комбинаций датчиков:

    //set_muxes("\x30\x00\xC0\x0C\x00\x30");//1(AOUT1)  9(AOUT2) 13(AOUT4) 21(AOUT3)   //1 итерация
    //set_muxes("\xC0\x00\x30\x03\x00\xC0");//2(AOUT1) 10(AOUT2) 14(AOUT4) 22(AOUT3)   //2 итерация
	//set_muxes("\x0C\x00\x03\xC0\x00\x0C");//3(AOUT1) 11(AOUT2) 15(AOUT4) 23(AOUT3)   //3
	//set_muxes("\x03\x00\x0C\x30\x00\x03");//4(AOUT1) 12(AOUT2) 16(AOUT4) 24(AOUT3)   //4
	//set_muxes("\x00\xC0\x00\x00\xC0\x00");//5(AOUT2) 17(AOUT4)                       //5
	//set_muxes("\x00\x30\x00\x00\x30\x00");//6(AOUT2) 18(AOUT4)                       //6
	//set_muxes("\x00\x0C\x00\x00\x0C\x00");//7(AOUT1) 19(AOUT3)                       //7
	//set_muxes("\x00\x03\x00\x00\x03\x00");//8(AOUT1) 20(AOUT3)                       //8
}
//GD версия обработчика
void DMA1_Channel0_IRQHandler(void)
{
    __asm("nop");
}
//STM версия обработчика
void DMA2_Stream0_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA1, DMA_CH0, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA1, DMA_CH0, DMA_INT_FLAG_FTF);
        //отключаем ADC  преобразования
        adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
        flag_dma_finish = SET;
    }
}

void init_sin(void) {
	for (int i = 0; i <= 90; i++)
		sin_t[i] = sin(2 * M_PI * i / 360);
}

float sin_fast(int a) {
	int sign = 1;
	while (a > 359) {
		a = a - 360;
	}
	if (a > 179) {
		a = a - 180;
		sign = -1;
	}
	if (a > 89) {
		return sign * sin_t[180 - a];
	} else {
		return sign * sin_t[a];
	}

}

float cos_fast(int a) {
	return sin_fast(90 + a);
}

/**
 * @brief  Рассчет амплитуды сигнала на одной частоте (F=частота дескритизации/360)
 * @param  signal: сигнал с АЦП 1 точка- это 1 градус (1/360 периода)
 * @param  size:	размер буфера
 * @retval Амплитуда гармоники
 */
float CalcHarm(uint16_t* signal, int size) {
	float sum1 = 0;
	float sum2 = 0;
	float a, b, c;
	float calc_amp;
	for (int i = 0; i < size; i++) {
		sum1 = sum1 + signal[i] * cos_fast(i);
		sum2 = sum2 + signal[i] * sin_fast(i);
	}
	a = 2 * sum1 / size;
	b = 2 * sum2 / size;
	c = a * a + b * b;
	if (c > 0)
		calc_amp = sqrt(c);
	else
		calc_amp = 0;
	return calc_amp;
}