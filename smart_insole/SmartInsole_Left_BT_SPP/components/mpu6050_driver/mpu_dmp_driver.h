// #ifdef _MPU_DMP_DRIVER_H
// #define _MPU_DMP_DRIVER_H

// #define MPU6500
#define MPU_I2C_SCL 18
#define MPU_I2C_SDA 19

uint8_t mpu_init_i2c(void);

void gyro_data_ready_cb(void);

// 初始化mpu6050
uint8_t mpu_dmp_init(void);

// 获取数据，给外部调用 
uint8_t dmp_get_data(void);



// #endif // DEBUG