#include "mpu9250.h"


int Deviation_Count;
short gyro[3], accel[3],magnet[3], sensors;
short Deviation_gyro[3],Original_gyro[3];    
 int Flag_Mpu6050;  
  


//void MPU9250_task(void *pvParameters)
//{
//  u32 lastWakeTime = getSysTickCnt();
//    while(1)
//    {
//			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
//
//		  	Deviation_Count++;
//			memcpy(Deviation_gyro,gyro,sizeof(gyro));
//
//			MPU_Get_Accelerometer(accel);
//			MPU_Get_Gyroscope(gyro);
//			MPU_Get_Magnetometer(magnet);
//    }
//}


uint8_t MPU9250_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t res=0;
   // IIC_Init();
  MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);
  HAL_Delay(100);
  MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);
	
  MPU_Set_Gyro_Fsr(hi2c,3);
	MPU_Set_Accel_Fsr(hi2c,0);
  MPU_Set_Rate(hi2c,50);

  MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_INT_EN_REG,0X00);
	MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);
	MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);
	MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);
    res=MPU_Read_Byte(hi2c,MPU9250_ADDR,MPU_DEVICE_ID_REG);
    if(res==MPU6500_ID) 
    {
       MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);
       MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);
		MPU_Set_Rate(hi2c,50);
    }
	else return 1;
 
    res=MPU_Read_Byte(hi2c,AK8963_ADDR,MAG_WIA);
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(hi2c,AK8963_ADDR,MAG_CNTL1,0X11);
    }
	else return 1;

    return 0;
}


uint8_t MPU_trial(I2C_HandleTypeDef *hi2c)
{
	return MPU_Read_Byte(hi2c,MPU9250_ADDR,MPU_DEVICE_ID_REG);
}


uint8_t MPU_Set_Gyro_Fsr(I2C_HandleTypeDef *hi2c,uint8_t fsr)
{
	return MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);
}


uint8_t MPU_Set_Accel_Fsr(I2C_HandleTypeDef *hi2c,uint8_t fsr)
{
	return MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);
}

uint8_t MPU_Set_LPF(I2C_HandleTypeDef *hi2c,uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_CFG_REG,data);
}

uint8_t MPU_Set_Rate(I2C_HandleTypeDef *hi2c,uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(hi2c,MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);
 	return MPU_Set_LPF(hi2c,rate/2);
}


short MPU_Get_Temperature(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2];
    short raw;
	float temp;
	MPU_Read_Len(hi2c,MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}

uint8_t MPU_Get_Gyroscope(I2C_HandleTypeDef *hi2c,short *x, short *y, short *z)
{
    uint8_t buf[6],res;
	res=MPU_Read_Len(hi2c,MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		
		*x =(((uint16_t)buf[0]<<8)|buf[1]);
		*y =(((uint16_t)buf[2]<<8)|buf[3]);
		*z= (((uint16_t)buf[4]<<8)|buf[5]);
			
	} 	
    return res;;
}

uint8_t MPU_Get_Accelerometer(I2C_HandleTypeDef *hi2c,short *x, short *y, short *z)
{
    uint8_t buf[6],res;
	res=MPU_Read_Len(hi2c,MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
{
		*x =((uint16_t)buf[0]<<8)|buf[1];
		*y =((uint16_t)buf[2]<<8)|buf[3];
		*z =((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}

uint8_t MPU_Get_Magnetometer(I2C_HandleTypeDef *hi2c,short *x,short *y,short *z)
{
    uint8_t buf[6],res;
	res=MPU_Read_Len(hi2c,AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*x=((uint16_t)buf[1]<<8)|buf[0];
		*y=((uint16_t)buf[3]<<8)|buf[2];
		*z=((uint16_t)buf[5]<<8)|buf[4];
	} 	
    MPU_Write_Byte(hi2c,AK8963_ADDR,MAG_CNTL1,0X11);
    return res;;
}

//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(I2C_HandleTypeDef *hi2c, uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
     HAL_I2C_Mem_Write(hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
     return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(I2C_HandleTypeDef *hi2c, uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    HAL_I2C_Mem_Read(hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
    return 0;       
}

//IIC写一个字节
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(I2C_HandleTypeDef *hi2c, uint8_t addr,uint8_t reg,uint8_t data)
{
    HAL_I2C_Mem_Write(hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xfff);
    return 0;
}

//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
uint8_t MPU_Read_Byte(I2C_HandleTypeDef *hi2c, uint8_t addr,uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 0xfff);
    return res;  
}
