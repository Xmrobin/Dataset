#include "driver/i2c.h"


#define WRITE_BIT I2C_MASTER_WRITE  //I2C master write 
#define READ_BIT I2C_MASTER_READ    //I2C master read 
#define ACK_CHECK_EN 0x1            //I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0           //I2C master will not check ack from slave 
#define ACK_VAL 0x0                 //I2C ack value 
#define NACK_VAL 0x1                //I2C nack value  

int esp32_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int esp32_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
int esp32_delay_ms(unsigned long num_ms);
int esp32_get_clock_ms(unsigned long *count);