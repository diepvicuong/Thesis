#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f4xx_hal.h"

// Using the MPU-9250, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
// mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif  

/* AK8961 address */
#define AK8963_ADDRESS 		0x0C<<1 //0x18

/* Magnetometer Address Register */

#define AK8963_WIA 				0x00	//WHO I AM
#define AK8963_INFO 			0x01
#define AK8963_ST1       	0x02  // data ready status bit 0
#define AK8963_XOUT_L    	0x03  // data
#define AK8963_XOUT_H    	0x04
#define AK8963_YOUT_L    	0x05
#define AK8963_YOUT_H    	0x06
#define AK8963_ZOUT_L    	0x07
#define AK8963_ZOUT_H    	0x08
#define AK8963_ST2       	0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      	0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      	0x0C  // Self test control
#define AK8963_I2CDIS    	0x0F  // I2C disable
#define AK8963_ASAX      	0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      	0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ     	0x12  // Fuse ROM z-axis sensitivity adjustment value

/*  Gyroscope and Accelerometer */
#define SELF_TEST_X_GYRO 	0x00                  
#define SELF_TEST_Y_GYRO 	0x01                                                                          
#define SELF_TEST_Z_GYRO 	0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define XG_OFFSET_H      	0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      	0x14
#define YG_OFFSET_H      	0x15
#define YG_OFFSET_L      	0x16
#define ZG_OFFSET_H      	0x17
#define ZG_OFFSET_L      	0x18
#define SMPLRT_DIV      	0x19
#define CONFIG           	0x1A
#define GYRO_CONFIG      	0x1B
#define ACCEL_CONFIG     	0x1C
#define ACCEL_CONFIG2    	0x1D
#define LP_ACCEL_ODR     	0x1E   
#define WOM_THR          	0x1F   

#define FIFO_EN          	0x23
#define I2C_MST_CTRL     	0x24   
#define I2C_SLV0_ADDR    	0x25
#define I2C_SLV0_REG     	0x26
#define I2C_SLV0_CTRL    	0x27
#define I2C_SLV1_ADDR    	0x28
#define I2C_SLV1_REG     	0x29
#define I2C_SLV1_CTRL    	0x2A
#define I2C_SLV2_ADDR    	0x2B
#define I2C_SLV2_REG     	0x2C
#define I2C_SLV2_CTRL    	0x2D
#define I2C_SLV3_ADDR    	0x2E
#define I2C_SLV3_REG     	0x2F
#define I2C_SLV3_CTRL    	0x30
#define I2C_SLV4_ADDR    	0x31
#define I2C_SLV4_REG     	0x32
#define I2C_SLV4_DO      	0x33
#define I2C_SLV4_CTRL    	0x34
#define I2C_SLV4_DI      	0x35
#define I2C_MST_STATUS   	0x36
#define INT_PIN_CFG      	0x37
#define INT_ENABLE       	0x38
#define DMP_INT_STATUS   	0x39  // Check DMP interrupt
#define INT_STATUS       	0x3A
#define ACCEL_XOUT_H     	0x3B
#define ACCEL_XOUT_L     	0x3C
#define ACCEL_YOUT_H     	0x3D
#define ACCEL_YOUT_L     	0x3E
#define ACCEL_ZOUT_H     	0x3F
#define ACCEL_ZOUT_L     	0x40
#define TEMP_OUT_H       	0x41
#define TEMP_OUT_L       	0x42
#define GYRO_XOUT_H      	0x43
#define GYRO_XOUT_L      	0x44
#define GYRO_YOUT_H      	0x45
#define GYRO_YOUT_L      	0x46
#define GYRO_ZOUT_H      	0x47
#define GYRO_ZOUT_L      	0x48
#define EXT_SENS_DATA_00 	0x49
#define EXT_SENS_DATA_01 	0x4A
#define EXT_SENS_DATA_02 	0x4B
#define EXT_SENS_DATA_03 	0x4C
#define EXT_SENS_DATA_04 	0x4D
#define EXT_SENS_DATA_05 	0x4E
#define EXT_SENS_DATA_06 	0x4F
#define EXT_SENS_DATA_07 	0x50
#define EXT_SENS_DATA_08 	0x51
#define EXT_SENS_DATA_09 	0x52
#define EXT_SENS_DATA_10 	0x53
#define EXT_SENS_DATA_11 	0x54
#define EXT_SENS_DATA_12 	0x55
#define EXT_SENS_DATA_13 	0x56
#define EXT_SENS_DATA_14 	0x57
#define EXT_SENS_DATA_15 	0x58
#define EXT_SENS_DATA_16 	0x59
#define EXT_SENS_DATA_17 	0x5A
#define EXT_SENS_DATA_18 	0x5B
#define EXT_SENS_DATA_19 	0x5C
#define EXT_SENS_DATA_20 	0x5D
#define EXT_SENS_DATA_21 	0x5E
#define EXT_SENS_DATA_22 	0x5F
#define EXT_SENS_DATA_23 	0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      	0x63
#define I2C_SLV1_DO      	0x64
#define I2C_SLV2_DO      	0x65
#define I2C_SLV3_DO      	0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  	0x69
#define USER_CTRL        	0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       	0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       	0x6C
#define DMP_BANK         	0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       	0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          	0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        	0x70
#define DMP_REG_2        	0x71 
#define FIFO_COUNTH      	0x72
#define FIFO_COUNTL      	0x73
#define FIFO_R_W         	0x74
#define WHO_AM_I_MPU9250 	0x75 // Should return 0x73 on 9255 or 0x71 on 9250
#define XA_OFFSET_H      	0x77
#define XA_OFFSET_L      	0x78
#define YA_OFFSET_H      	0x7A
#define YA_OFFSET_L      	0x7B
#define ZA_OFFSET_H      	0x7D
#define ZA_OFFSET_L      	0x7E

/* Set Gyroscope, Accelerometer, Magnetometer Parameter*/
enum Gscale {
	GFS_250dps,
	GFS_500dps,
	GFS_1000dps,
	GFS_2000dps
};

enum Ascale {
	AFS_2g,
	AFS_4g,
	AFS_8g,
	AFS_16g
};

enum Mscale {
	MFS_14BITS,	// 0.6 mG per LSB
	MFS_16BITS	// 0.15 mG per LSB

};

typedef struct MPU_9250 {
	uint8_t Ascale;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	uint8_t Gscale; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	uint8_t Mscale; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
	uint8_t Mmode;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR  

	float magScale[3];
	float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	float magCalibration[3], magbias[3]; // Factory mag calibration and mag bias
	float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
	int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
	float temperature;
	float SelfTest[6];
	
	float pitch, yaw, roll;
	float deltat;                             // integration interval for both filter schemes
	int lastUpdate , firstUpdate, Now;    // used to calculate integration interval                               // used to calculate integration interval
	float q[4];           // vector to hold quaternion
	float eInt[3];
} MPU9250_t;

void initModuleIMU(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu);
void initParameter(MPU9250_t *mpu);
void readData(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu);
char readByte(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t regAddr);
int16_t readTempData(I2C_HandleTypeDef *hi2c);
void resetMPU9250(I2C_HandleTypeDef *hi2c);
void initAK8963(I2C_HandleTypeDef *hi2c ,MPU9250_t *mpu , float * dest);
void initMPU9250(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu);
void calibrateMPU9250(I2C_HandleTypeDef *hi2c ,float * dest1, float * dest2);
void magcalMPU9250(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu ,float * dest1, float * dest2) ;
void MPU9250SelfTest(I2C_HandleTypeDef *hi2c ,float * destination); // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void MadgwickQuaternionUpdate(I2C_HandleTypeDef *hi2c , MPU9250_t *mpu, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
#endif