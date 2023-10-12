#include <stdio.h>
#include "main.h"
#include "adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t ITG3205_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(0, ITG3205_SENSOR_ADDR, &reg_addr, 1, data, len, 1000);
}
esp_err_t ADXL345_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(0, ADXL345_SENSOR_ADDR, &reg_addr, 1, data, len, 1000);
}
esp_err_t HMC5883L_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(0, HMC5883L_SENSOR_ADDR, &reg_addr, 1, data, len, 1000);
}
esp_err_t ITG3205_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(0, ITG3205_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000);

    return ret;
}
esp_err_t ADXL345_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(0, ADXL345_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000);

    return ret;
}
esp_err_t HMC5883L_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(0, HMC5883L_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000);

    return ret;
}
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 19,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}
void Init_ITG3205(void)
{
    ITG3205_register_write_byte(ITG3205_PWR_M,0x80);    //电源管理-将设备和内部寄存器重置为开机默认设置
    ITG3205_register_write_byte(ITG3205_SMPL,0x07);     //内部采样率1khz，采样率=1khz/（7+1）=125Hz or 8ms per sample
    ITG3205_register_write_byte(ITG3205_DLPF,0x1E);     //设置量程±2000°/sec，低通滤波带宽5HZ、内部采样率1khz
    ITG3205_register_write_byte(ITG3205_INT_C,0x00);    //设置中断（默认值）
    ITG3205_register_write_byte(ITG3205_PWR_M,0x00);    //置电源管理（默认值）
}
void Init_ADXL345(void)
{
    ADXL345_register_write_byte(ADXL345_DATA_FORMAT,0X0B);//禁用自测力 4线spi模式 高电平中断有效,13位全分辨率,输出数据右对齐,16g量程
    ADXL345_register_write_byte(ADXL345_BW_RATE,0x0A);//正常模式，数据输出速度为100Hz
    ADXL345_register_write_byte(ADXL345_POWER_CTL,0x08);//测量模式、普通工作模式
    ADXL345_register_write_byte(ADXL345_INT_ENABLE,0x80);//使能数据准备中断
}
void Init_HMC5883L(void)
{
    HMC5883L_register_write_byte(HMC_CONFIG_A_REG,0X78);//8次采样平均，75hz输出速率，正常测量配置
    HMC5883L_register_write_byte(HMC_CONFIG_B_REG,0x00);//+-0.88GA增益 
    HMC5883L_register_write_byte(HMC_MODE_REG,0x00);//连续测量模式 
}
void Read_ITG3205(void)//后续需要加偏移量及其他转换
{
    uint8_t Buf[6] = {0};
    ITG3205_register_read(ITG3205_GX_H,Buf,6);
    xGyro = ((Buf[0] << 8) | Buf[1]);                   
    yGyro = ((Buf[2] << 8) | Buf[3]);
    zGyro = ((Buf[4] << 8) | Buf[5]);

    //温度数据
    ITG3205_register_read(ITG3205_TMP_H,Buf,2);
    temp = ((Buf[0] << 8) | Buf[1]);
}
void Read_ADXL345(void)//后续需要作其他转换
{
    uint8_t Buf[6] = {0};
    ADXL345_register_read(ADXL345_AX_L,Buf,6);
    xAcce = ((Buf[1] << 8) | Buf[0]);                   //需要加偏移量
    yAcce = ((Buf[3] << 8) | Buf[2]);
    zAcce = ((Buf[5] << 8) | Buf[4]);
}
void Read_HMC5883L(void)//后续需要作其他转换
{
    uint8_t Buf[6] = {0};
    HMC5883L_register_read(HMC_XMSB_REG,Buf,6);
    xMagn = ((Buf[0] << 8) | Buf[1]);                   //需要加偏移量
    zMagn = ((Buf[2] << 8) | Buf[3]);
    yMagn = ((Buf[4] << 8) | Buf[5]);
}

void app_main(void)
{
    uint8_t data[2] = {0};
    uint8_t id[2] = {0};
    uint8_t reg[3] = {0};
    Init_ADC();
   if(i2c_master_init() == ESP_OK)
        printf("Success\n");
    else 
        printf("Error\n");   
   
    Init_ITG3205();
    Init_ADXL345();
    Init_HMC5883L();
    ITG3205_register_read(ITG3205_WHO,data,1);
    printf("WHO_AM_I = %X\n", data[0]);
    ADXL345_register_read(ADXL345_WHO,id,1);
    printf("ADXL345_ID:%X\n",id[0]);
    HMC5883L_register_read(HMC_CHEAK_A_REG,reg,3);
    printf("HMC5883L_A:%X\tHMC5883L_B:%X\tHMC5883L_C:%X\n",reg[0],reg[1],reg[2]);
    while (1)
    {
        Read_ITG3205();
        Read_ADXL345();
        Read_HMC5883L();
        ADC_Pro();
        printf("陀螺仪各轴角速度:xGyro=%.2f°/s\tyGyro=%.2f°/s\tzGyro=%.2f°/s\n",xGyro/14.375+1.1,yGyro/14.375-2.9,zGyro/14.375-0.1);
        printf("加速度计各轴分量:xAcce=%.2fg\tyAcce=%.2fg\tzAcce=%.2fg\n",(float)xAcce*0.0039,(float)yAcce*0.0039,(float)zAcce*0.0039);
        printf("磁力计原始参数:  xMagn=%d\tyMagn=%d\tzMagn=%d\n",xMagn,yMagn,zMagn);
        printf("芯片温度:%.1f\n",35 + ((double)(temp + 13200)) / 280);
        
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

