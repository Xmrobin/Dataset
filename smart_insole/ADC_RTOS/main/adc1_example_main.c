#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "batt.h"

uint32_t adc_reading_0 = 0;
uint32_t adc_reading_3 = 0;
uint32_t adc_reading_4 = 0;
uint32_t adc_reading_5 = 0;
uint32_t adc_reading_6 = 0;
uint32_t adc_reading_7 = 0;
int adc2_9_read_raw = 0;
int adc2_8_read_raw = 0;
static const adc_channel_t channel = ADC_CHANNEL_0|ADC_CHANNEL_3|ADC_CHANNEL_4|ADC_CHANNEL_5|ADC_CHANNEL_6|ADC_CHANNEL_7;   
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
void Init_ADC()
{
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    adc2_config_channel_atten(ADC2_CHANNEL_8,ADC_ATTEN_11db);
    adc2_config_channel_atten(ADC2_CHANNEL_9,ADC_ATTEN_11db);
}
static void ADC__task(void *pvParameters)
{
    while (1)
    {
        adc_reading_0 = adc1_get_raw(ADC1_CHANNEL_0);
        adc_reading_3 = adc1_get_raw(ADC1_CHANNEL_3);
        adc_reading_4 = adc1_get_raw(ADC1_CHANNEL_4);
        adc_reading_5 = adc1_get_raw(ADC1_CHANNEL_5);
        adc_reading_6 = adc1_get_raw(ADC1_CHANNEL_6);
        adc_reading_7 = adc1_get_raw(ADC1_CHANNEL_7);
        adc2_get_raw(ADC2_CHANNEL_9,ADC_WIDTH_12Bit,&adc2_9_read_raw);
        adc2_get_raw(ADC2_CHANNEL_8,ADC_WIDTH_12Bit,&adc2_8_read_raw);
        vTaskDelay(50 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    Init_ADC();
    xTaskCreate(ADC__task,"adc_Task",4096,NULL,1,NULL);
    //xTaskCreate(batt_monitor,"batt_monitor",2048,NULL,2,NULL);
     while (1) {
         printf("GPIO36_ADC0: %d\n", adc_reading_0);
         printf("GPIO39_ADC3: %d\n", adc_reading_3);
         printf("GPIO32_ADC4: %d\n", adc_reading_4);
         printf("GPIO33_ADC5: %d\n", adc_reading_5);
         printf("GPIO34_ADC6: %d\n", adc_reading_6);
         printf("GPIO35_ADC7: %d\n", adc_reading_7);
         printf("GPIO25_ADC8: %d\n", adc2_8_read_raw);
         printf("GPIO26_ADC9: %d\n", adc2_9_read_raw);
         printf("\n");
         vTaskDelay(pdMS_TO_TICKS(500));
     }
}
