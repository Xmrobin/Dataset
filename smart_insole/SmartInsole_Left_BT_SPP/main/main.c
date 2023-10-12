/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "SmartInsole_Left"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

static void BT_SPP_task(void *arg);

/*---------- FSR_ADC -------------- */

#include "adc.h"
#include "string.h"
static void ADC__task(void *arg);

uint32_t adc_reading_0 = 0;
uint32_t adc_reading_3 = 0;
uint32_t adc_reading_4 = 0;
uint32_t adc_reading_5 = 0;
uint32_t adc_reading_6 = 0;
uint32_t adc_reading_7 = 0;
uint32_t adc2_reading_9 = 0;
uint32_t adc2_reading_8 = 0;
int adc2_9_read_raw = 0;
int adc2_8_read_raw = 0;
char ble_data[100]={0};
/*---------- FSR_ADC -------------- */

/*---------- MPU6050 -------------- */
#include <stdio.h>
#include <stdlib.h>
#include "mpu_dmp_driver.h"
#include "driver/gpio.h"

#define GPIO_MPU_INTR 4
extern float pitch, roll, yaw;   
static xQueueHandle gpio_evt_queue = NULL; // 用于接收 GPIO 中断的 queue

short pitch_ble,roll_ble,yaw_ble;
uint16_t MPU6050_Pitch = 0;
uint16_t MPU6050_Roll = 0;
uint16_t MPU6050_Yaw = 0;
// 初始化所有需要使用的GPIO 
void gpio_init(void);
void gpio_task(void* arg);  
void gpio_intr_handle(void* arg);
/*---------- MPU6050 -------------- */

uint16_t bt_handle;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        bt_handle = 0;
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        /*
         * We only show the data in which the data length is less than 128 here. If you want to print the data and
         * the data rate is high, it is strongly recommended to process them in other lower priority application task
         * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
         * stack and also have a effect on the throughput!
         */
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d",
                 param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 128) {
            esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
        }
#else
        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }
#endif
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        gettimeofday(&time_old, NULL);
        bt_handle = param->cong.handle;
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void app_main(void)
{
    char bda_str[18] = {0};

    /* 初始化ADC */
    Init_ADC();
    xTaskCreate(ADC__task,"adc_Task",2048,NULL,2,NULL);
    /* mpu6050初始化及配置 */
    mpu_dmp_init();
    gpio_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));//用于在GPIO中断和gpio_task任务之间发送消息  
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);//完成姿态数据的获取
    gpio_install_isr_service(0);//安装GPIO中断驱动服务
    gpio_isr_handler_add(GPIO_MPU_INTR, gpio_intr_handle, (void*) GPIO_MPU_INTR);//为相应的GPIO管脚添加ISR处理程序
    /* 蓝牙串口传输数据任务 */
    xTaskCreate(BT_SPP_task,"BLE_data_task",4096,NULL,5,NULL);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}
static void BT_SPP_task(void *arg)
{
    for(;;){
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(bt_handle){
           ESP_LOGI(SPP_TAG,"字节数=%d 字符串的长度=%d",sizeof(ble_data),strlen(ble_data));
           esp_spp_write(bt_handle, strlen(ble_data), (uint8_t *)ble_data);
        }
    }
    vTaskDelete(NULL);
}
static void ADC__task(void *arg)
{ 
    while (1)
    {
        vTaskDelay(20 / portTICK_RATE_MS);
        adc_reading_0 = 0;
        adc_reading_3 = 0;
        adc_reading_4 = 0;
        adc_reading_5 = 0;
        adc_reading_6 = 0;
        adc_reading_7 = 0;
        adc2_reading_9 = 0;
        adc2_reading_8 = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) 
        {   
           adc_reading_0 += adc1_get_raw(ADC1_CHANNEL_0);
           adc_reading_3 += adc1_get_raw(ADC1_CHANNEL_3);
           adc_reading_4 += adc1_get_raw(ADC1_CHANNEL_4);
           adc_reading_5 += adc1_get_raw(ADC1_CHANNEL_5);
           adc_reading_6 += adc1_get_raw(ADC1_CHANNEL_6);
           adc_reading_7 += adc1_get_raw(ADC1_CHANNEL_7);
           esp_err_t err = adc2_get_raw(ADC2_CHANNEL_9,ADC_WIDTH_12Bit,&adc2_9_read_raw); 
            if(err == ESP_OK)
                adc2_reading_9 += adc2_9_read_raw;
           esp_err_t err1 = adc2_get_raw(ADC2_CHANNEL_8,ADC_WIDTH_12Bit,&adc2_8_read_raw); 
           if(err1 == ESP_OK)
                adc2_reading_8 += adc2_8_read_raw;

        }
        adc_reading_0 /= NO_OF_SAMPLES;
        adc_reading_3 /= NO_OF_SAMPLES;
        adc_reading_4 /= NO_OF_SAMPLES;
        adc_reading_5 /= NO_OF_SAMPLES;
        adc_reading_6 /= NO_OF_SAMPLES;
        adc_reading_7 /= NO_OF_SAMPLES;
        adc2_reading_9 /= NO_OF_SAMPLES;
        adc2_reading_8 /= NO_OF_SAMPLES;
        //ESP_LOGI(GATTS_FSR_TAG,"%d-%d-%d-%d-%d-%d-%d-%d\n",adc_reading_0,adc_reading_3,adc_reading_4,adc_reading_5,adc_reading_6,adc_reading_7,adc2_reading_9,adc2_reading_8);
        sprintf(ble_data,"%d-%d-%d-%d-%d-%d-%d-%d %d %d %d\n",adc_reading_0,adc_reading_3,adc_reading_4,adc_reading_5,adc_reading_6,adc_reading_7,adc2_reading_9,adc2_reading_8,pitch_ble,roll_ble,yaw_ble);
    }
    vTaskDelete(NULL);
}
/* MPU6050 相关GPIO函数配置 */
void gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_MPU_INTR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,//上升沿触发
    };
    gpio_config(&io_conf);

}
void gpio_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            gyro_data_ready_cb();
            dmp_get_data();
            //printf("pitch:%f,roll:%f,yaw:%f\n",pitch, roll, yaw);
            pitch_ble = (short)pitch;
            roll_ble = (short)roll;
            yaw_ble = (short)yaw;
        }
    }
}
/* DMP是通过中断通知MPU姿态角数据准备好了,需要将MPU6050模块的INT引脚接到 ESP32 的GPIO4上 */ 
void gpio_intr_handle(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}