#include "adc.h"

void Init_ADC()
{
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    adc2_config_channel_atten(ADC2_CHANNEL_4,ADC_ATTEN_11db);
    adc2_config_channel_atten(ADC2_CHANNEL_5,ADC_ATTEN_11db);
}
void ADC_Pro()
{
    uint32_t adc_reading_0 = 0;
        uint32_t adc_reading_3 = 0;
        uint32_t adc_reading_4 = 0;
        uint32_t adc_reading_5 = 0;
        uint32_t adc_reading_6 = 0;
        uint32_t adc_reading_7 = 0;
        uint32_t adc2_reading_5 = 0;
        uint32_t adc2_reading_4 = 0;
        int adc2_read_raw = 0;
        int adc_read_raw = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) 
        {   
           adc_reading_0 += adc1_get_raw(ADC1_CHANNEL_0);
           adc_reading_3 += adc1_get_raw(ADC1_CHANNEL_3);
           adc_reading_4 += adc1_get_raw(ADC1_CHANNEL_4);
           adc_reading_5 += adc1_get_raw(ADC1_CHANNEL_5);
           adc_reading_6 += adc1_get_raw(ADC1_CHANNEL_6);
           adc_reading_7 += adc1_get_raw(ADC1_CHANNEL_7);
           esp_err_t err = adc2_get_raw(ADC2_CHANNEL_5,ADC_WIDTH_12Bit,&adc2_read_raw); 
            if(err == ESP_OK)
                adc2_reading_5 += adc2_read_raw;
           esp_err_t err1 = adc2_get_raw(ADC2_CHANNEL_4,ADC_WIDTH_12Bit,&adc_read_raw); 
           if(err1 == ESP_OK)
                adc2_reading_4 += adc_read_raw;
        }
        adc_reading_0 /= NO_OF_SAMPLES;
        adc_reading_3 /= NO_OF_SAMPLES;
        adc_reading_4 /= NO_OF_SAMPLES;
        adc_reading_5 /= NO_OF_SAMPLES;
        adc_reading_6 /= NO_OF_SAMPLES;
        adc_reading_7 /= NO_OF_SAMPLES;
        adc2_reading_5 /= NO_OF_SAMPLES;
        adc2_reading_4 /= NO_OF_SAMPLES;

        printf("GPIO36_ADC0: %d\n", adc_reading_0);
        printf("GPIO39_ADC3: %d\n", adc_reading_3);
        printf("GPIO32_ADC4: %d\n", adc_reading_4);
        printf("GPIO33_ADC5: %d\n", adc_reading_5);
        printf("GPIO34_ADC6: %d\n", adc_reading_6);
        printf("GPIO35_ADC7: %d\n", adc_reading_7);
        printf("GPIO12_ADC15: %d\n", adc2_reading_5);
        printf("GPIO13_ADC14: %d\n", adc2_reading_4);
        printf("\n");
}
