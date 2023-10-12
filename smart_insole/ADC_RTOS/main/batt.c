#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// ADC Attenuation
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_CHAN ADC2_CHANNEL_5

// ADC Calibration

#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_VREF


int32_t adc_batt_raw;
int32_t adc_batt_v;
static esp_adc_cal_characteristics_t adc2_chars;

static const char *TAG = "BAT";

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED)
    {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    }
    else if (ret == ESP_ERR_INVALID_VERSION)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else if (ret == ESP_OK)
    {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN, ADC_WIDTH, 0, &adc2_chars);
        ESP_LOGI(TAG, "ADC alibration enabled");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}
void batt_monitor(void *pvParameters)
{
    bool cali_enable = adc_calibration_init();

    // adc
    
    adc2_config_channel_atten(ADC_CHAN, ADC_ATTEN);

    while (1)
    {
        adc2_get_raw(ADC_CHAN,ADC_WIDTH,&adc_batt_raw);       
        if (cali_enable)
        {
            adc_batt_v = esp_adc_cal_raw_to_voltage(adc_batt_raw, &adc2_chars) * 2;
        }

        ESP_LOGI(TAG, "BATT RAW: %d, V: %d", adc_batt_raw, adc_batt_v);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }

    vTaskDelete(NULL);
}
