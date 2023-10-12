/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "blehr_sens.h"

#include "adc.h"
#include "driver/gpio.h"

/* MPU6050 */
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

/* FSR */
static void ADC__task(void *pvParameters);

uint32_t adc_reading_0 = 0;
uint32_t adc_reading_3 = 0;
uint32_t adc_reading_4 = 0;
uint32_t adc_reading_5 = 0;
uint32_t adc_reading_6 = 0;
uint32_t adc_reading_7 = 0;
uint32_t adc2_reading_5 = 0;
uint32_t adc2_reading_4 = 0;
int adc2_5_read_raw = 0;
int adc2_4_read_raw = 0;

static const char *tag = "NimBLE_BLE_HeartRate";

static xTimerHandle blehr_tx_timer;

static bool notify_state;

static uint16_t conn_handle;

static const char *device_name = "blehr_sensor_1.0";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;

/**
 * Utility function to log an array of bytes.
 */
void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}


/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void
blehr_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blehr_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void
blehr_tx_hrate_stop(void)
{
    xTimerStop( blehr_tx_timer, 1000 / portTICK_PERIOD_MS );
}

/* Reset heart rate measurement */
static void
blehr_tx_hrate_reset(void)
{
    int rc;

    if (xTimerReset(blehr_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
        rc = 0;
    } else {
        rc = 1;
    }

    assert(rc == 0);

}


/* This function simulates heart beat and notifies it to the client */
static void
blehr_tx_hrate(xTimerHandle ev)
{
    static uint8_t hrm[22];
    int rc;
    struct os_mbuf *om;

    if (!notify_state) {        //unsubscribe
        blehr_tx_hrate_stop();
        return;
    }
    /* FSR */
    hrm[0] = (adc_reading_0>>8)&0xFF;
    hrm[1] = adc_reading_0&0xFF;
    hrm[2] = (adc_reading_3>>8)&0xFF;
    hrm[3] = adc_reading_3&0xFF;
    hrm[4] = (adc_reading_4>>8)&0xFF;
    hrm[5] = adc_reading_4&0xFF;
    hrm[6] = (adc_reading_5>>8)&0xFF;
    hrm[7] = adc_reading_5&0xFF;
    hrm[8] = (adc_reading_6>>8)&0xFF;
    hrm[9] = adc_reading_6&0xFF;
    hrm[10] = (adc_reading_7>>8)&0xFF;
    hrm[11] = adc_reading_7&0xFF;
    hrm[12] = (adc2_reading_5>>8)&0xFF;
    hrm[13] = adc2_reading_5&0xFF;
    hrm[14] = (adc2_reading_4>>8)&0xFF;
    hrm[15] = adc2_reading_4&0xFF;
    /* BLE传输欧拉角处理 */
    if(pitch_ble < 0)
    {
        hrm[16] = 1;
        MPU6050_Pitch = -pitch_ble;
    }
    else if(pitch_ble >= 0)
    {
        hrm[16] = 0;
        MPU6050_Pitch = pitch_ble;
    }
        
    if(roll_ble < 0)
    {
        hrm[18] = 1;
        MPU6050_Roll = -roll_ble;
    }
    else if(roll_ble >= 0)
    {
        hrm[18] = 0;
        MPU6050_Roll = roll_ble;
    }
        
    if(yaw_ble < 0)
    {
        hrm[20] = 1;
        MPU6050_Yaw = -yaw_ble;
    }
    else if(yaw_ble >= 0)
    {
        hrm[20] = 0;
        MPU6050_Yaw = yaw_ble;
    }
  
    hrm[17] = MPU6050_Pitch;
    hrm[19] = MPU6050_Roll;
    hrm[21] = MPU6050_Yaw; 
    
    om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));
    rc = ble_gattc_notify_custom(conn_handle, hrs_hrm_handle, om);//hrs_hrm_handle：特征值属性句柄
    
    assert(rc == 0);

    blehr_tx_hrate_reset();
}

static int
blehr_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blehr_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, hrs_hrm_handle);
        if (event->subscribe.attr_handle == hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_reset();
        } else if (event->subscribe.attr_handle != hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static void
blehr_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blehr_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    /* Begin advertising */
    blehr_advertise();
}

static void
blehr_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void blehr_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void app_main(void)
{
    int rc;

    Init_ADC();
    /* sensors-MOS管导通，FSR供电使能 */
    gpio_reset_pin(GPIO_NUM_23);
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_23, 0);

    /* mpu6050初始化及配置 */
    mpu_dmp_init();
    gpio_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));//用于在GPIO中断和gpio_task任务之间发送消息  
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);//完成姿态数据的获取
    gpio_install_isr_service(0);//安装GPIO中断驱动服务
    gpio_isr_handler_add(GPIO_MPU_INTR, gpio_intr_handle, (void*) GPIO_MPU_INTR);//为相应的GPIO管脚添加ISR处理程序

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = blehr_on_sync;
    ble_hs_cfg.reset_cb = blehr_on_reset;

    /* name, period/time,  auto reload, timer ID, callback */
    blehr_tx_timer = xTimerCreate("blehr_tx_timer", pdMS_TO_TICKS(50), pdTRUE, (void *)0, blehr_tx_hrate);

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(blehr_host_task);

    xTaskCreate(&ADC__task,"adc_Task",4096,NULL,4,NULL);

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
static void ADC__task(void *pvParameters)
{
    while (1)
    {
        adc_reading_0 = 0;
        adc_reading_3 = 0;
        adc_reading_4 = 0;
        adc_reading_5 = 0;
        adc_reading_6 = 0;
        adc_reading_7 = 0;
        adc2_reading_5 = 0;
        adc2_reading_4 = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) 
        {   
           adc_reading_0 += adc1_get_raw(ADC1_CHANNEL_0);
           adc_reading_3 += adc1_get_raw(ADC1_CHANNEL_3);
           adc_reading_4 += adc1_get_raw(ADC1_CHANNEL_4);
           adc_reading_5 += adc1_get_raw(ADC1_CHANNEL_5);
           adc_reading_6 += adc1_get_raw(ADC1_CHANNEL_6);
           adc_reading_7 += adc1_get_raw(ADC1_CHANNEL_7);
           esp_err_t err = adc2_get_raw(ADC2_CHANNEL_9,ADC_WIDTH_12Bit,&adc2_5_read_raw); 
            if(err == ESP_OK)
                adc2_reading_5 += adc2_5_read_raw;
           esp_err_t err1 = adc2_get_raw(ADC2_CHANNEL_8,ADC_WIDTH_12Bit,&adc2_4_read_raw); 
           if(err1 == ESP_OK)
                adc2_reading_4 += adc2_4_read_raw;

        }
        adc_reading_0 /= NO_OF_SAMPLES;
        adc_reading_3 /= NO_OF_SAMPLES;
        adc_reading_4 /= NO_OF_SAMPLES;
        adc_reading_5 /= NO_OF_SAMPLES;
        adc_reading_6 /= NO_OF_SAMPLES;
        adc_reading_7 /= NO_OF_SAMPLES;
        adc2_reading_5 /= NO_OF_SAMPLES;
        adc2_reading_4 /= NO_OF_SAMPLES;
        //ESP_LOGI(tag,"%d-%d-%d-%d-",adc_reading_0,adc_reading_3,adc_reading_4,adc_reading_5);
        vTaskDelay(50 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}