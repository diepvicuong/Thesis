#include "MPU9250.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "uart.h"

static void getGres(MPU9250_t *mpu); 
static void getMres(MPU9250_t *mpu);
static void getAres(MPU9250_t * mpu);
static void writeByte(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t regAddr, uint8_t data);
static void readBytes(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t regAddr, uint8_t *dest, uint8_t size);
static void readAccelData(I2C_HandleTypeDef *hi2c, int16_t * dest);
static void readGyroData(I2C_HandleTypeDef *hi2c, int16_t * dest);
static void readMagData(I2C_HandleTypeDef *hi2c ,int16_t* dest);

extern I2C_HandleTypeDef hi2c1;

// parameters for 6 DoF sensor fusion calculations
const float PI = 3.14159265358979323846f;
const float GyroMeasError = PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

void initModuleIMU(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu)
{
	// Reset registers to default in preparation for device calibration
	resetMPU9250(hi2c); 
	
	// Initialize parameter.
	initParameter(mpu);
	
	// get sensor resolutions, only need to do this once
	getAres(mpu); 
	getGres(mpu); 
	getMres(mpu); 
	
	calibrateMPU9250(hi2c, mpu->gyroBias, mpu->accelBias);  // Calibrate gyro and accelerometers, load biases in bias registers  
	HAL_Delay(1000);
	initMPU9250(hi2c, mpu);
	initAK8963(hi2c, mpu, mpu->magCalibration);
	magcalMPU9250(hi2c, mpu, mpu->magbias, mpu->magScale);
	HAL_Delay(1000);
}
void initParameter(MPU9250_t *mpu)
{
	static float temp[3] = {0.0f, 0.0f, 0.0f};
	mpu->Ascale = AFS_2g;     
	mpu->Gscale = GFS_250dps; 
	mpu->Mscale = MFS_16BITS;
	memcpy(mpu->magCalibration, temp, sizeof(temp));
	memcpy(mpu->magbias, temp, sizeof(temp));
	memcpy(mpu->gyroBias, temp, sizeof(temp));
	memcpy(mpu->accelBias, temp, sizeof(temp));
	memcpy(mpu->magScale, temp, sizeof(temp));
	mpu->Mmode = 0x06;
	mpu->deltat = 0.0f;                             // integration interval for both filter schemes
	mpu->lastUpdate = 0, mpu->firstUpdate = 0, mpu->Now = 0;  // used to calculate integration interval
	static float temp_2[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	memcpy(mpu->q, temp_2, sizeof(temp_2));       // vector to hold quaternion
	memcpy(mpu->eInt, temp, sizeof(temp));
	mpu->roll = 0;
	mpu->pitch = 0;
	mpu->yaw = 0;
}

void readData(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu)
{
	readAccelData(hi2c, mpu->accelCount);  // Read the x/y/z adc values   
	// Now we'll calculate the accleration value into actual g's
	mpu->ax = (float)mpu->accelCount[0] * mpu->aRes - mpu->accelBias[0];  // get actual g value, this depends on scale being set
	mpu->ay = (float)mpu->accelCount[1] * mpu->aRes - mpu->accelBias[1];   
	mpu->az = (float)mpu->accelCount[2] * mpu->aRes - mpu->accelBias[2];  
   
	readGyroData(hi2c, mpu->gyroCount);  // Read the x/y/z adc values
	// Calculate the gyro value into actual degrees per second
	mpu->gx = (float)mpu->gyroCount[0] * mpu->gRes - mpu->gyroBias[0];  // get actual gyro value, this depends on scale being set
	mpu->gy = (float)mpu->gyroCount[1] * mpu->gRes - mpu->gyroBias[1];  
	mpu->gz = (float)mpu->gyroCount[2] * mpu->gRes - mpu->gyroBias[2];   
  
	readMagData(hi2c, mpu->magCount);  // Read the x/y/z adc values   
	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	mpu->mx = (float)mpu->magCount[0] * mpu->mRes * mpu->magCalibration[0] - mpu->magbias[0];  // get actual magnetometer value, this depends on scale being set
	mpu->my = (float)mpu->magCount[1] * mpu->mRes * mpu->magCalibration[1] - mpu->magbias[1];  
	mpu->mz = (float)mpu->magCount[2] * mpu->mRes * mpu->magCalibration[2] - mpu->magbias[2];   
	mpu->mx *= mpu->magScale[0];
	mpu->my *= mpu->magScale[1];
	mpu->mz *= mpu->magScale[2];	
}

static void writeByte(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t regAddr, uint8_t data) 
{
	uint8_t data_write[2];
	data_write[0] = regAddr;
  data_write[1] = data;
	HAL_I2C_Master_Transmit(hi2c, addr, data_write, 2, 10);
}

char readByte(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t regAddr) 
{
	uint8_t data[1]; 	//data will be stored in the register data
	uint8_t data_write[1];
	data_write[0] = regAddr;
	HAL_I2C_Master_Transmit(hi2c, addr, data_write, 1, 10);
	HAL_I2C_Master_Receive(hi2c, addr, data, 1, 10);
	return data[0];
}

static void readBytes(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t regAddr, uint8_t *dest, uint8_t size) 
{
	uint8_t data[16];
	uint8_t data_write[1];
	data_write[0] = regAddr;
	HAL_I2C_Master_Transmit(hi2c, addr, data_write, 1, 10); // no stop
	HAL_I2C_Master_Receive(hi2c, addr, data, size, 10);
	for(int i = 0; i < size; i++) {
			dest[i] = data[i];
	}
}

static void getMres(MPU9250_t *mpu) 
{
  switch (mpu->Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mpu->mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mpu->mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
          break;
  }
}

static void getGres(MPU9250_t *mpu) 
{
  switch (mpu->Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250dps:
          mpu->gRes = 250.0/32768.0;
          break;
    case GFS_500dps:
          mpu->gRes = 500.0/32768.0;
          break;
    case GFS_1000dps:
          mpu->gRes = 1000.0/32768.0;
          break;
    case GFS_2000dps:
          mpu->gRes = 2000.0/32768.0;
          break;
  }
}

static void getAres(MPU9250_t * mpu) 
{
  switch (mpu->Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2g:
          mpu->aRes = 2.0/32768.0;
          break;
    case AFS_4g:
          mpu->aRes = 4.0/32768.0;
          break;
    case AFS_8g:
          mpu->aRes = 8.0/32768.0;
          break;
    case AFS_16g:
          mpu->aRes = 16.0/32768.0;
          break;
  }
}

static void readAccelData(I2C_HandleTypeDef *hi2c, int16_t * dest) 
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(hi2c, MPU9250_ADDRESS, ACCEL_XOUT_H, rawData, 6);  // Read the six raw data registers into data array
  dest[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  dest[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  dest[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

static void readGyroData(I2C_HandleTypeDef *hi2c, int16_t * dest)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(hi2c ,MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
  dest[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  dest[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  dest[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

static void readMagData(I2C_HandleTypeDef *hi2c ,int16_t* dest)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(hi2c, AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		readBytes(hi2c, AK8963_ADDRESS, AK8963_XOUT_L, &rawData[0], 7);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    dest[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
    dest[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
   }
  }
}

int16_t readTempData(I2C_HandleTypeDef *hi2c)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(hi2c ,MPU9250_ADDRESS, TEMP_OUT_H, &rawData[0], 2);  // Read the two raw data registers sequentially into data array 
  return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
}

void resetMPU9250(I2C_HandleTypeDef *hi2c) 
{
  // reset device
  writeByte(hi2c ,MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(10);
}


void initAK8963(I2C_HandleTypeDef *hi2c ,MPU9250_t *mpu , float * dest)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(hi2c ,AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  HAL_Delay(100);
  writeByte(hi2c ,AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  HAL_Delay(100);
  readBytes(hi2c, AK8963_ADDRESS, AK8963_ASAX, &rawData[0], 3);  // Read the x-, y-, and z-axis calibration values
  dest[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  dest[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  dest[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
  writeByte(hi2c ,AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  HAL_Delay(100);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(hi2c ,AK8963_ADDRESS, AK8963_CNTL, mpu->Mscale << 4 | mpu->Mmode); // Set magnetometer data resolution and sample ODR
  HAL_Delay(100);
}

void initMPU9250(I2C_HandleTypeDef *hi2c, MPU9250_t * mpu)
{  
 // Initialize MPU9250 device
 // wake up device
  writeByte(hi2c ,MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  HAL_Delay(10); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // get stable time source
  writeByte(hi2c ,MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  writeByte(hi2c ,MPU9250_ADDRESS, CONFIG, 0x03);  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(hi2c ,MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(hi2c ,MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | mpu->Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(hi2c ,MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | mpu->Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(hi2c ,MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(hi2c ,MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.

void calibrateMPU9250(I2C_HandleTypeDef *hi2c ,float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(hi2c ,MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(hi2c ,MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(hi2c ,MPU9250_ADDRESS, PWR_MGMT_2, 0x00); 
  HAL_Delay(200);
  
// Configure device for bias calculation
  writeByte(hi2c ,MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(hi2c ,MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(hi2c ,MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(hi2c ,MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(hi2c ,MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(hi2c ,MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay(15);
  
// Configure MPU9250 gyro and accelerometer for bias calculation
  writeByte(hi2c ,MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(hi2c ,MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(hi2c ,MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(hi2c ,MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(hi2c ,MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  HAL_Delay(400); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(hi2c ,MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(hi2c ,MPU9250_ADDRESS, FIFO_COUNTH, &data[0], 2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(hi2c ,MPU9250_ADDRESS, FIFO_R_W, &data[0], 12); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeByte(hi2c ,MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(hi2c ,MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(hi2c ,MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(hi2c ,MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(hi2c ,MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(hi2c ,MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(hi2c ,MPU9250_ADDRESS, XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(hi2c ,MPU9250_ADDRESS, YA_OFFSET_H, &data[0], 2);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(hi2c ,MPU9250_ADDRESS, ZA_OFFSET_H, &data[0], 2);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void magcalMPU9250(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu ,float * dest1, float * dest2) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

//  printf("Mag Calibration: Wave device in a figure eight until done!\r\n");
//  HAL_Delay(4000);
  
// shoot for ~fifteen seconds of mag data
    if(mpu->Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(mpu->Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
    for(ii = 0; ii < sample_count; ii++) {
    readMagData(hi2c, mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(mpu->Mmode == 0x02) HAL_Delay(125);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(mpu->Mmode == 0x06) HAL_Delay(10);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

//    printf("mag x min/max: = %d/%d\r\n", mag_min[0], mag_max[0]);
//		printf("mag y min/max: = %d/%d\r\n", mag_min[1], mag_max[1]);
//		printf("mag z min/max: = %d/%d\r\n", mag_min[2], mag_max[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mpu->mRes*mpu->magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mpu->mRes*mpu->magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*mpu->mRes*mpu->magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= (double)3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
//   printf("Mag Calibration done!\r\n");
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(I2C_HandleTypeDef *hi2c ,float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;
   
	writeByte(hi2c ,MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	writeByte(hi2c ,MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(hi2c ,MPU9250_ADDRESS, GYRO_CONFIG, FS<<3); // Set full scale range for the gyro to 250 dps
	writeByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer
		readBytes(hi2c ,MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6); // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
		
		readBytes(hi2c ,MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }
  
  for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
	writeByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(hi2c ,MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_Delay(25); // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer 
		readBytes(hi2c ,MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
		
		readBytes(hi2c ,MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }
  
  for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
  }
  
	// Configure the gyro and accelerometer for normal operation
	writeByte(hi2c ,MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(hi2c ,MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	HAL_Delay(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(hi2c ,MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(hi2c ,MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(hi2c ,MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(hi2c ,MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
	selfTest[4] = readByte(hi2c ,MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	selfTest[5] = readByte(hi2c ,MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (double)(2620/1<<FS)*(pow( 1.01 , ((double)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (double)(2620/1<<FS)*(pow( 1.01 , ((double)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (double)(2620/1<<FS)*(pow( 1.01 , ((double)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (double)(2620/1<<FS)*(pow( 1.01 , ((double)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (double)(2620/1<<FS)*(pow( 1.01 , ((double)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (double)(2620/1<<FS)*(pow( 1.01 , ((double)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
	destination[i] = 100.0*((double)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0; // Report percent differences
	destination[i+3] = 100.0*((double)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0; // Report percent differences
	}  
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(I2C_HandleTypeDef *hi2c , MPU9250_t *mpu, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
		const float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
		float q1 = mpu->q[0], q2 = mpu->q[1], q3 = mpu->q[2], q4 = mpu->q[3];   // short name local variable for readability
	  float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx, _2q1my, _2q1mz, _2q2mx;
		float _4bx, _4bz;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _2q4 = 2.0f * q4;
		float _2q1q3 = 2.0f * q1 * q3;
		float _2q3q4 = 2.0f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		ax *= norm;
		ay *= norm;
		az *= norm;
//		printf("DEBUG ax = %.2f\r\n", ax);
		
		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f/norm;
		mx *= norm;
		my *= norm;
		mz *= norm;
//		printf("DEBUG mz = %.2f\r\n", mz);

		// Reference direction of Earth's magnetic field
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

//		printf("DEBUG _4bx = %.2f\r\n", _4bx);
		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		norm = 1.0f/norm;
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;
//	printf("DEBUG s3 = %.2f\r\n", s3);
		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

//		printf("DEBUG qDot3 = %.2f\r\n", qDot3);
		// Integrate to yield quaternion
		q1 += qDot1 *  mpu->deltat;
		q2 += qDot2 * mpu->deltat;
		q3 += qDot3 * mpu->deltat;
		q4 += qDot4 * mpu->deltat;
		
//		printf("DEBUG q3 = %.2f\r\n", q3);
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		norm = 1.0f/norm;
		mpu->q[0] = q1 * norm;
		mpu->q[1] = q2 * norm;
		mpu->q[2] = q3 * norm;
		mpu->q[3] = q4 * norm;
}
// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones. 
void MahonyQuaternionUpdate(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
		float q1 = mpu->q[0], q2 = mpu->q[1], q3 = mpu->q[2], q4 = mpu->q[3];   // short name local variable for readability
		float norm;
		float hx, hy, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;
		float pa, pb, pc;

		// Auxiliary variables to avoid repeated arithmetic
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;   

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return; // handle NaN
		norm = 1.0f / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
		hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
		bx = sqrt((hx * hx) + (hy * hy));
		bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

		// Estimated direction of gravity and magnetic field
		vx = 2.0f * (q2q4 - q1q3);
		vy = 2.0f * (q1q2 + q3q4);
		vz = q1q1 - q2q2 - q3q3 + q4q4;
		wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
		wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
		wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

		// Error is cross product between estimated direction and measured direction of gravity
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
		if (Ki > 0.0f)
		{
				mpu->eInt[0] += ex;      // accumulate integral error
				mpu->eInt[1] += ey;
				mpu->eInt[2] += ez;
		}
		else
		{
				mpu->eInt[0] = 0.0f;     // prevent integral wind up
				mpu->eInt[1] = 0.0f;
				mpu->eInt[2] = 0.0f;
		}

		// Apply feedback terms
		gx = gx + Kp * ex + Ki * mpu->eInt[0];
		gy = gy + Kp * ey + Ki * mpu->eInt[1];
		gz = gz + Kp * ez + Ki * mpu->eInt[2];

		// Integrate rate of change of quaternion
		pa = q2;
		pb = q3;
		pc = q4;
		q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * mpu->deltat);
		q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * mpu->deltat);
		q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * mpu->deltat);
		q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * mpu->deltat);

		// Normalise quaternion
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
		norm = 1.0f / norm;
		mpu->q[0] = q1 * norm;
		mpu->q[1] = q2 * norm;
		mpu->q[2] = q3 * norm;
		mpu->q[3] = q4 * norm;
}
