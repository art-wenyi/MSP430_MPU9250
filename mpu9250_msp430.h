/*
 * mpu9250_msp430.h
 * mpu9250 driver for msp430f5659, I2C interface
 * Created on: 2016-06-27
 *      Author: wenyi zou
 */

#ifndef MPU9250_MSP430_H_
#define	MPU9250_MSP430_H_

#include <msp430f5659.h>
#define MPU_ADDR 0x68        // set AD0 to ground, then 0x68 is the MPU 9250 address


// Register address, reference of MPU 9150 datasheet
// magnetometer registers
#define AK8963_ADDRESS   0x0C<<1
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2	 0x0B
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

// gyro and accel registers
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69

#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 

#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74

#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// sensor data
struct Sensor{
	unsigned char gyro_x[2];
	unsigned char gyro_y[2];
	unsigned char gyro_z[2];
	unsigned char accel_x[2];
	unsigned char accel_y[2];
	unsigned char accel_z[2];
	unsigned char mag_x[2];
	unsigned char mag_y[2];
	unsigned char mag_z[2];
	unsigned char magasa[3];    		// mag asistant sensity data
	unsigned char int_status;          // use for dummy read, need for mpu9150
	unsigned char mag_st1_status;
	unsigned char mag_st2_status;
	float gyro_reso;	// dafault use 1000dps
	float accel_reso;		// default use 4g/s
	float mag_reso; 	// default mag 16bit reso
	float gyro_x_float;
	float gyro_y_float;
	float gyro_z_float;
	float accel_x_float;
	float accel_y_float;
	float accel_z_float;
	float mag_x_float;
	float mag_y_float;
	float mag_z_float;
	float mag_x_asa;
	float mag_y_asa;
	float mag_z_asa;
	int cov_tmp;   		// for data convert from char -> int -> float, middle int tmp
	void (*convert)(struct Sensor *sensor);			// function for convert data from char -> int -> float, need to be registered to Mpu_Convert_Data
	void (*convert_asa)(struct Sensor *sensor);		// convert mag asa data
};

void Mpu_Convert_Data(struct Sensor *sensor);		// callback function for convert the raw data to float
void Mpu_Convert_Asa(struct Sensor *sensor);		// convert mag asa data to float

// use i2c to set register value in mpu9250
void Mpu_I2c_SetReg(unsigned char reg_addr, unsigned char value);
void Mpu_I2c_SetReg_Mag(unsigned char reg_addr, unsigned char value);

// Init the MPU-9250 Chip for motor segway
void Mpu_Init();
void Mpu_9250_Init();
void Mpu_I2c_AK8963C_Init();

//Read the fifo information, return the bytes number that read, fifo value store in *buffer
int Mpu_I2c_ReadFIFO(unsigned char *buffer);
//
int Mpu_I2c_CheckDataReady();		//check if data is ready for read, 1 means ok, 0 means not yet
// Read the register information, return the bytes number that read, though here is 1, value store in buffer
int Mpu_I2c_ReadReg(unsigned char reg_addr, unsigned char len ,unsigned char *buffer);
int Mpu_I2c_ReadReg_Mag(unsigned char reg_addr, unsigned char len ,unsigned char *buffer);

// Read the Gyro information, return the bytes number that read, fifo value store in *buffer
int Mpu_I2c_ReadGyro_x(unsigned char *buffer);
int Mpu_I2c_ReadGyro_y(unsigned char *buffer);
int Mpu_I2c_ReadGyro_z(unsigned char *buffer);
void Mpu_I2c_ReadGyro(unsigned char *x, unsigned char *y, unsigned char *z, unsigned char *st);

// Read the Acceleration information, return the bytes number that read, fifo value store in *buffer
int Mpu_I2c_ReadAccel_x(unsigned char *buffer);
int Mpu_I2c_ReadAccel_y(unsigned char *buffer);
int Mpu_I2c_ReadAccel_z(unsigned char *buffer);
void Mpu_I2c_ReadAccel(unsigned char *x, unsigned char *y, unsigned char *z, unsigned char *st);

// Read the Magneto information, return the bytes number that read, fifo value store in *buffer
void Mpu_I2c_ReadMagASA(unsigned char *buffer);
int Mpu_I2c_ReadMag_x(unsigned char *buffer);
int Mpu_I2c_ReadMag_y(unsigned char *buffer);
int Mpu_I2c_ReadMag_z(unsigned char *buffer);
void Mpu_I2c_ReadMag(unsigned char *x, unsigned char *y, unsigned char *z, unsigned char *st1, unsigned char *st2);
int Mpu_I2c_CheckMag(unsigned char *buffer);

// Read the temperature
int Mpu_I2c_ReadTemp(unsigned char *buffer);

// Read interrupt status register and clear it
int Mpu_I2c_ReadAndClearIntStatusRegister(unsigned char *buffer);
int Mpu_I2c_ReadAndClearMagStatusRegister(unsigned char *buffer);
#endif	/* MPU9250_MSP430_H_ */
