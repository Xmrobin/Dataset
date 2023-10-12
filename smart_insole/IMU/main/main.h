#include "driver/i2c.h"

#define ITG3205_SENSOR_ADDR 0x68   /*!< slave address for ITG3205 sensor */
#define ADXL345_SENSOR_ADDR 0x53   /*!< slave address for ADXL345 sensor */
#define HMC5883L_SENSOR_ADDR 0x1E   /*!< slave address for HMC5883L sensor */
//定义ITG3205内部地址********************
#define ITG3205_WHO	            0x00
#define	ITG3205_SMPL	        0x15
#define ITG3205_DLPF	        0x16
#define ITG3205_INT_C	        0x17
#define	ITG3205_TMP_H	        0x1B
#define	ITG3205_TMP_L	        0x1C
#define	ITG3205_GX_H	        0x1D
#define ITG3205_PWR_M	        0x3E
//定义ADXL345内部地址
#define ADXL345_BW_RATE         0x2C
#define ADXL345_POWER_CTL       0x2D
#define ADXL345_INT_ENABLE      0x2E
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_DATA_FORMAT     0x31
#define	ADXL345_AX_L	        0x32
#define ADXL345_WHO	            0x00        //默认值为0x5E
//定义HMC5883L内部地址
#define HMC_CONFIG_A_REG        0X00
#define HMC_CONFIG_B_REG        0X01
#define HMC_MODE_REG            0X02
#define HMC_XMSB_REG            0X03        //数据寄存器
#define HMC_CHEAK_A_REG         0X0A        //默认值为0x48
#define HMC_CHEAK_B_REG         0X0B        //默认值为0x34
#define HMC_CHEAK_C_REG         0X0C        //默认值为0x33

short xGyro = 0, yGyro = 0, zGyro = 0;//存放各个轴向的角速度值原始值
short temp = 0; 
short xAcce = 0, zAcce = 0, yAcce = 0;//存放各个轴向的加速度值原始值
short xMagn = 0, yMagn = 0, zMagn = 0;


esp_err_t ITG3205_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t ADXL345_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t HMC5883L_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t ITG3205_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t ADXL345_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t HMC5883L_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t i2c_master_init(void);
void Init_ITG3205(void);
void Init_ADXL345(void);
void Init_HMC5883L(void);
void Read_ITG3205(void);
void Read_ADXL345(void);
void Read_HMC5883L(void);
