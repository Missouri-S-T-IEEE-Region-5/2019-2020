/**
  ******************************************************************************
  * @file    Waveshare_10Dof-D.h
  * @author  Waveshare Team
  * @version V1.0
  * @date    Dec-2018
  * @brief   
  
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2018 Waveshare</center></h2>
  ******************************************************************************
  */
#ifndef __Waveshare_10DOF_D_H__
#define __Waveshare_10DOF_D_H__
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/* define ICM-20948 Device I2C address*/
#define I2C_ADD_ICM20948            0x68
#define I2C_ADD_ICM20948_AK09916    0x0C
#define I2C_ADD_ICM20948_AK09916_READ  0x80
#define I2C_ADD_ICM20948_AK09916_WRITE 0x00
/* define ICM-20948 Register */
/* user bank 0 register */
#define REG_ADD_WIA             0x00
    #define REG_VAL_WIA             0xEA
#define REG_ADD_USER_CTRL       0x03
    #define REG_VAL_BIT_DMP_EN          0x80
    #define REG_VAL_BIT_FIFO_EN         0x40
    #define REG_VAL_BIT_I2C_MST_EN      0x20
    #define REG_VAL_BIT_I2C_IF_DIS      0x10
    #define REG_VAL_BIT_DMP_RST         0x08
    #define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MIGMT_1     0x06
    #define REG_VAL_ALL_RGE_RESET   0x80
    #define REG_VAL_RUN_MODE        0x01    //Non low-power mode
#define REG_ADD_LP_CONFIG       0x05
#define REG_ADD_PWR_MGMT_1      0x06
#define REG_ADD_PWR_MGMT_2      0x07
#define REG_ADD_ACCEL_XOUT_H    0x2D
#define REG_ADD_ACCEL_XOUT_L    0x2E
#define REG_ADD_ACCEL_YOUT_H    0x2F
#define REG_ADD_ACCEL_YOUT_L    0x30
#define REG_ADD_ACCEL_ZOUT_H    0x31
#define REG_ADD_ACCEL_ZOUT_L    0x32
#define REG_ADD_GYRO_XOUT_H     0x33
#define REG_ADD_GYRO_XOUT_L     0x34
#define REG_ADD_GYRO_YOUT_H     0x35
#define REG_ADD_GYRO_YOUT_L     0x36
#define REG_ADD_GYRO_ZOUT_H     0x37
#define REG_ADD_GYRO_ZOUT_L     0x38
#define REG_ADD_EXT_SENS_DATA_00 0x3B
#define REG_ADD_REG_BANK_SEL    0x7F
    #define REG_VAL_REG_BANK_0  0x00
    #define REG_VAL_REG_BANK_1  0x10
    #define REG_VAL_REG_BANK_2  0x20
    #define REG_VAL_REG_BANK_3  0x30

/* user bank 1 register */
/* user bank 2 register */
#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1   0x01
    #define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
    #define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
    #define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3] */
    #define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
    #define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] */
    #define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
    #define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */    
    #define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]   */
#define REG_ADD_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ADD_ACCEL_CONFIG        0x14
    #define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
    #define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
    #define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
    #define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
    #define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
    #define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
    #define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */    
    #define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */

/* user bank 3 register */
#define REG_ADD_I2C_SLV0_ADDR   0x03
#define REG_ADD_I2C_SLV0_REG    0x04
#define REG_ADD_I2C_SLV0_CTRL   0x05
    #define REG_VAL_BIT_SLV0_EN     0x80
    #define REG_VAL_BIT_MASK_LEN    0x07
#define REG_ADD_I2C_SLV0_DO     0x06
#define REG_ADD_I2C_SLV1_ADDR   0x07
#define REG_ADD_I2C_SLV1_REG    0x08
#define REG_ADD_I2C_SLV1_CTRL   0x09
#define REG_ADD_I2C_SLV1_DO     0x0A

/* define ICM-20948 Register  end */

/* define ICM-20948 MAG Register  */
#define REG_ADD_MAG_WIA1    0x00
    #define REG_VAL_MAG_WIA1    0x48
#define REG_ADD_MAG_WIA2    0x01
    #define REG_VAL_MAG_WIA2    0x09
#define REG_ADD_MAG_ST2     0x10
#define REG_ADD_MAG_DATA    0x11
#define REG_ADD_MAG_CNTL2   0x31
    #define REG_VAL_MAG_MODE_PD     0x00
    #define REG_VAL_MAG_MODE_SM     0x01
    #define REG_VAL_MAG_MODE_10HZ   0x02
    #define REG_VAL_MAG_MODE_20HZ   0x04
    #define REG_VAL_MAG_MODE_50HZ   0x05
    #define REG_VAL_MAG_MODE_100HZ  0x08
    #define REG_VAL_MAG_MODE_ST     0x10
/* define ICM-20948 MAG Register  end */

#define MAG_DATA_LEN    6

/* 
 *  BMP280 I2c address
 */
#define BMP280_AD0_LOW     0x76 //address pin low (GND)
#define BMP280_AD0_HIGH    0x77 //address pin high (VCC)
#define BMP280_ADDR        BMP280_AD0_HIGH     // default I2C address  
/* 
 *  BMP280 register address
 */
#define BMP280_REGISTER_DIG_T1      0x88
#define BMP280_REGISTER_DIG_T2      0x8A
#define BMP280_REGISTER_DIG_T3      0x8C

#define BMP280_REGISTER_DIG_P1      0x8E
#define BMP280_REGISTER_DIG_P2      0x90
#define BMP280_REGISTER_DIG_P3      0x92
#define BMP280_REGISTER_DIG_P4      0x94
#define BMP280_REGISTER_DIG_P5      0x96
#define BMP280_REGISTER_DIG_P6      0x98
#define BMP280_REGISTER_DIG_P7      0x9A
#define BMP280_REGISTER_DIG_P8      0x9C
#define BMP280_REGISTER_DIG_P9      0x9E 

#define BMP280_REGISTER_CHIPID      0xD0
#define BMP280_REGISTER_VERSION     0xD1
#define BMP280_REGISTER_SOFTRESET   0xE0
#define BMP280_REGISTER_STATUS      0xF3
#define BMP280_REGISTER_CONTROL     0xF4
#define BMP280_REGISTER_CONFIG      0xF5

#define BMP280_TEMP_XLSB_REG        0xFC      /*Temperature XLSB Register */
#define BMP280_TEMP_LSB_REG         0xFB        /*Temperature LSB Register  */ 
#define BMP280_TEMP_MSB_REG         0xFA        /*Temperature LSB Register  */  
#define BMP280_PRESS_XLSB_REG       0xF9    /*Pressure XLSB  Register   */
#define BMP280_PRESS_LSB_REG        0xF8    /*Pressure LSB Register     */
#define BMP280_PRESS_MSB_REG        0xF7    /*Pressure MSB Register     */  

/*calibration parameters */  
#define BMP280_DIG_T1_LSB_REG                0x88  
#define BMP280_DIG_T1_MSB_REG                0x89  
#define BMP280_DIG_T2_LSB_REG                0x8A  
#define BMP280_DIG_T2_MSB_REG                0x8B  
#define BMP280_DIG_T3_LSB_REG                0x8C  
#define BMP280_DIG_T3_MSB_REG                0x8D  
#define BMP280_DIG_P1_LSB_REG                0x8E  
#define BMP280_DIG_P1_MSB_REG                0x8F  
#define BMP280_DIG_P2_LSB_REG                0x90  
#define BMP280_DIG_P2_MSB_REG                0x91  
#define BMP280_DIG_P3_LSB_REG                0x92  
#define BMP280_DIG_P3_MSB_REG                0x93  
#define BMP280_DIG_P4_LSB_REG                0x94  
#define BMP280_DIG_P4_MSB_REG                0x95  
#define BMP280_DIG_P5_LSB_REG                0x96  
#define BMP280_DIG_P5_MSB_REG                0x97  
#define BMP280_DIG_P6_LSB_REG                0x98  
#define BMP280_DIG_P6_MSB_REG                0x99  
#define BMP280_DIG_P7_LSB_REG                0x9A  
#define BMP280_DIG_P7_MSB_REG                0x9B  
#define BMP280_DIG_P8_LSB_REG                0x9C  
#define BMP280_DIG_P8_MSB_REG                0x9D  
#define BMP280_DIG_P9_LSB_REG                0x9E  
#define BMP280_DIG_P9_MSB_REG                0x9F

#ifdef __cplusplus
extern "C" {
#endif

typedef enum 
{  
  IMU_EN_SENSOR_TYPE_NULL = 0,
  IMU_EN_SENSOR_TYPE_ICM20948,
  IMU_EN_SENSOR_TYPE_BMP280,
  IMU_EN_SENSOR_TYPE_MAX
}IMU_EN_SENSOR_TYPE;

typedef struct imu_st_angles_data_tag
{
  float fYaw;
  float fPitch;
  float fRoll;
}IMU_ST_ANGLES_DATA;

typedef struct imu_st_sensor_data_tag
{
  int16_t s16X;
  int16_t s16Y;
  int16_t s16Z;
}IMU_ST_SENSOR_DATA;

typedef struct icm20948_st_avg_data_tag
{
  uint8_t u8Index;
  int16_t s16AvgBuffer[8];
}ICM20948_ST_AVG_DATA;

void imuInit(IMU_EN_SENSOR_TYPE *penMotionSensorType, IMU_EN_SENSOR_TYPE *penPressureType);
void imuDataGet(IMU_ST_ANGLES_DATA *pstAngles, 
                IMU_ST_SENSOR_DATA *pstGyroRawData,
                IMU_ST_SENSOR_DATA *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData); 
void pressSensorDataGet(int32_t *ps32Temperature, int32_t *ps32Pressure, int32_t *ps32Altitude);

uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif //__Waveshare_10DOF_D_H__
