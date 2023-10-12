#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"

#define NO_OF_SAMPLES   64          //Multisampling

static const adc_channel_t channel = ADC_CHANNEL_0|ADC_CHANNEL_3|ADC_CHANNEL_4|ADC_CHANNEL_5|ADC_CHANNEL_6|ADC_CHANNEL_7;   
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

void Init_ADC();
