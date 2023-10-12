#include "MPU6050.h"  

esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(0, MPU6050_ADDR, &reg_addr, 1, data, len, 1000);//1表示写1字节，1000表示超时等待的最大时间间隔为1s
}
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(0, MPU6050_ADDR, write_buf, sizeof(write_buf), 1000);

    return ret;
}

/**
* @brief 初始化 i2c
*/
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MPU6050_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(MPU6050_I2C_PORT_NUM, &conf);
    if (err != ESP_OK) {
        printf("i2c_param_config: %d \n", err);
    }
    return i2c_driver_install(MPU6050_I2C_PORT_NUM, conf.mode, 0, 0, 0);
}
/**
* @brief 初始化 mpu6050
*/
void Init_mpu6050(void)
{
    mpu6050_register_write_byte(0x6B,0x80);// PWR_MGMT_1, DEVICE_RESET 
    vTaskDelay(500 / portTICK_RATE_MS);
    mpu6050_register_write_byte(0x6B,0x00);// cleat SLEEP
    mpu6050_register_write_byte(0x1B,0x18);// Gyroscope Full Scale Range = ± 2000 °/s
    mpu6050_register_write_byte(0x1C,0x01);// Accelerometer Full Scale Range = ± 16g
    mpu6050_register_write_byte(0x38,0x00);// Interrupt Enable.disenable 
    mpu6050_register_write_byte(0x6A,0x00);// User Control.auxiliary I2C are logically driven by the primary I2C bus
    mpu6050_register_write_byte(0x23,0x00);// FIFO Enable.disenable
    mpu6050_register_write_byte(0x19,0x63);// Sample Rate Divider.Sample Rate = 1KHz / (1 + 99) **采样率有问题**
    mpu6050_register_write_byte(0x1A,0x13);// EXT_SYNC_SET = GYRO_XOUT_L[0]; Bandwidth = 3
    mpu6050_register_write_byte(0x6B,0x01);// Power Management 1.PLL with X axis gyroscope reference
    mpu6050_register_write_byte(0x6C,0x00);// Power Management 2
}

/**
* @brief 读取加速度计、温度和陀螺仪数据
*/
measurement_out_t mpu6050_get_value()
{
    uint8_t measurement_bytes_out[14];
    mpu6050_register_read(0x3B, measurement_bytes_out, 14);
    measurement_out_t measurement_out = {
        .accel_out.accel_xout = (int16_t)(measurement_bytes_out[0]<<8 | measurement_bytes_out[1]),
        .accel_out.accel_yout = (int16_t)(measurement_bytes_out[2]<<8 | measurement_bytes_out[3]),
        .accel_out.accel_zout = (int16_t)(measurement_bytes_out[4]<<8 | measurement_bytes_out[5]),
        .temp_out.temp_xout = (int16_t)(measurement_bytes_out[6]<<8 | measurement_bytes_out[7]),
        .gyro_out.gyro_xout = (int16_t)(measurement_bytes_out[8]<<8 | measurement_bytes_out[9]),
        .gyro_out.gyro_yout = (int16_t)(measurement_bytes_out[10]<<8 | measurement_bytes_out[11]),
        .gyro_out.gyro_zout = (int16_t)(measurement_bytes_out[12]<<8 | measurement_bytes_out[13]),
    };
    return measurement_out;
}
