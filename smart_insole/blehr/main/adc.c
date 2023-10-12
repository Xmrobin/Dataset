#include "adc.h"

void Init_ADC()
{
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    adc2_config_channel_atten(ADC2_CHANNEL_8,ADC_ATTEN_11db);
    adc2_config_channel_atten(ADC2_CHANNEL_9,ADC_ATTEN_11db);
}
