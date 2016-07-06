
#include <msp430f5659.h>
#include "mpu9250_msp430.h"
#include "i2c.h"


/* Below are the function defined here but not in mpu_i2c.h.
 * The reason for this is to hide these function to the user.
 * just lile the private proprety in C++ class
 *
 * reference from internet device_i2c.c, someone's github , thanks for that one here
 *
 */

void Mpu_Convert_Data(struct Sensor *sensor){
	// gyro convert
	sensor->cov_tmp = ((int)sensor->gyro_x[0] << 8) | sensor->gyro_x[1];    // [0] is high bit, [1] is low bit
	sensor->gyro_x_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	sensor->cov_tmp = ((int)sensor->gyro_y[0] << 8) | sensor->gyro_y[1];    // [0] is high bit, [1] is low bit
	sensor->gyro_y_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	sensor->cov_tmp = ((int)sensor->gyro_z[0] << 8) | sensor->gyro_z[1];    // [0] is high bit, [1] is low bit
	sensor->gyro_z_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	// accel convert
	sensor->cov_tmp = ((int)sensor->accel_x[0] << 8) | sensor->accel_x[1];    // [0] is high bit, [1] is low bit
	sensor->accel_x_float = (float)sensor->cov_tmp * sensor->accel_reso;
	sensor->cov_tmp = ((int)sensor->accel_y[0] << 8) | sensor->accel_y[1];    // [0] is high bit, [1] is low bit
	sensor->accel_y_float = (float)sensor->cov_tmp * sensor->accel_reso;
	sensor->cov_tmp = ((int)sensor->accel_z[0] << 8) | sensor->accel_z[1];    // [0] is high bit, [1] is low bit
	sensor->accel_z_float = (float)sensor->cov_tmp * sensor->accel_reso;
	//mag convert
	sensor->cov_tmp = ((int)sensor->mag_x[0] << 8) | sensor->mag_x[1];    // [0] is high bit, [1] is low bit
	sensor->mag_x_float = (float)sensor->cov_tmp * sensor->mag_reso * sensor->mag_x_asa;
	sensor->cov_tmp = ((int)sensor->mag_y[0] << 8) | sensor->mag_y[1];    // [0] is high bit, [1] is low bit
	sensor->mag_y_float = (float)sensor->cov_tmp * sensor->mag_reso * sensor->mag_y_asa;
	sensor->cov_tmp = ((int)sensor->mag_z[0] << 8) | sensor->mag_z[1];    // [0] is high bit, [1] is low bit
	sensor->mag_z_float = (float)sensor->cov_tmp * sensor->mag_reso * sensor->mag_z_asa;
}

void Mpu_Convert_Asa(struct Sensor *sensor){
	sensor->mag_x_asa = (float)(sensor->magasa[0] - 128) / 256.0 + 1.0;
	sensor->mag_y_asa = (float)(sensor->magasa[1] - 128) / 256.0 + 1.0;
	sensor->mag_z_asa = (float)(sensor->magasa[2] - 128) / 256.0 + 1.0;
}


/* below are functions definition corresponding to mpu_i2c.h file's functions
 */

// set Mpu register
void Mpu_I2c_SetReg(unsigned char reg_addr, unsigned char value){
    I2C_Write_Packet_To_Gyro(reg_addr, 0x01, &value);
}

void Mpu_I2c_SetReg_Mag(unsigned char reg_addr, unsigned char value){
	I2C_Write_Packet_To_Mag(reg_addr, 0x01, &value);
}

// Initialize the Mpu register settings
void Mpu_Init(){
	I2C_Master_Init(TRUE, SMCLK_FREQ, I2C_CLOCK_FREQ);
	Mpu_9250_Init();
	Mpu_I2c_AK8963C_Init();   // don't mess up the init sequence
}

// Init the gyro and accel part of MPU9250
void Mpu_9250_Init(){
	Mpu_I2c_SetReg(PWR_MGMT_1, 0x80);     // 0x80 means soft reset the device
	// wait at least 100 ms, based on 8Mhz sysclk 0.8m times
	int i=0;
	for(i=0;i<100;i++)
		__delay_cycles(8000);					// delay 0.1s
	Mpu_I2c_SetReg(PWR_MGMT_1, 0x01);      // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	//Mpu_I2c_SetReg(user_ctrl, 0x40, i2cnum);      // 0x40 enable fifo
	//Mpu_I2c_SetReg(fifo_en, 0xf8, i2cnum);      // 0xf8 means enable gyro, temp and accl value write into fifo

	// below configs are from Ke xu's recommendation, which match Ke xu's setting

	Mpu_I2c_SetReg(CONFIG, 0x03);      // 0x01 means set gyro sample frequency to 1khz, bandwidth 184hz, and set corresponding bandwidth,check datasheet to decide
	Mpu_I2c_SetReg(SMPLRT_DIV, 0x04);      // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	Mpu_I2c_SetReg(GYRO_CONFIG, 0x10);      // 0x10 means gyro sensitive rate is 1000dps
	Mpu_I2c_SetReg(ACCEL_CONFIG, 0x08);      // 0x08 means gyro sensitive rate is 4g/sec
	Mpu_I2c_SetReg(ACCEL_CONFIG2, 0x03);      // 0x01, same as CONFIG 0x03
	Mpu_I2c_SetReg(INT_PIN_CFG, 0x02); 			// don't use interrupt to inform msp430, use polling to get data constantly

	// use interrupt here to inform the msp430 data is ready
	/*Mpu_I2c_SetReg(INT_PIN_CFG, 0xA0);      // 0xA0 means int pin active low, push-pull mode, int pin hold until int status is cleared
											//interrupt cleared by reading INT_STATUS register
	Mpu_I2c_SetReg(INT_ENABLE, 0x01);      // 0x00 disable,0x01 means generate interrupt when sensor data is ready
	*/
}

// Init the magneto part of MPU9250
void Mpu_I2c_AK8963C_Init(){
	Mpu_I2c_SetReg_Mag(AK8963_CNTL, 0x00);  // Power down magnetometer
	int i=0;
	for(i=0;i<100;i++)
		__delay_cycles(8000);					// delay 0.1s
	Mpu_I2c_SetReg_Mag(AK8963_CNTL, 0x16);  // Enter continuous measurement mode 2. 100hz
	for(i=0;i<100;i++)
			__delay_cycles(8000);					// delay 0.1s
}


// Read the Mpu 9150 register value, return the number that read(should be 1 byte), store in input *buffer
int Mpu_I2c_ReadReg(unsigned char reg_addr, unsigned char len ,unsigned char *buffer){
    I2C_Read_Packet_From_Gyro(reg_addr, len, buffer);
    return 1;
}

int Mpu_I2c_ReadReg_Mag(unsigned char reg_addr, unsigned char len ,unsigned char *buffer){
    I2C_Read_Packet_From_Mag(reg_addr, len, buffer);
    return 1;
}

//Read the Mpu 9150 fifo value, return the number that read from fifo, store in input *buffer
int Mpu_I2c_ReadFIFO(unsigned char *buffer){
    return 1;           // i don't need this function now, don't like to use fifo in MPU9250

}

int Mpu_I2c_CheckDataReady(){		// check if data is ready for read, 1 means ok, 0 means not yet
       unsigned char buffer = 0x00;
		Mpu_I2c_ReadReg(INT_STATUS, 1, &buffer);   // i2c_status read, clear the high interrupt signal(INT_ANYRD_2CLEAR need to be reset as 0)
       return buffer & 0x01;
}

//Read the Gyro information, return the bytes number that read, gyro value store in *buffer
int Mpu_I2c_ReadGyro_x(unsigned char *buffer){
       Mpu_I2c_ReadReg(GYRO_XOUT_H, 1, buffer);
       Mpu_I2c_ReadReg(GYRO_XOUT_L, 1, buffer+1);     // read Gyro,
       return 1;
}

int Mpu_I2c_ReadGyro_y(unsigned char *buffer){
       Mpu_I2c_ReadReg(GYRO_YOUT_H, 1, buffer);
       Mpu_I2c_ReadReg(GYRO_YOUT_L, 1, buffer+1);     // read Gyro,
       return 1;
}

int Mpu_I2c_ReadGyro_z(unsigned char *buffer){
       Mpu_I2c_ReadReg(GYRO_ZOUT_H, 1, buffer);
       Mpu_I2c_ReadReg(GYRO_ZOUT_L, 1, buffer+1);     // read Gyro,
       return 1;
}

void Mpu_I2c_ReadGyro(unsigned char *x, unsigned char *y, unsigned char *z, unsigned char *st){   // ready gyro x,y,z and int respectively
	Mpu_I2c_ReadGyro_x(x);
	Mpu_I2c_ReadGyro_y(y);
	Mpu_I2c_ReadGyro_z(z);
	Mpu_I2c_ReadAndClearIntStatusRegister(st);

}

//Read the Acceleration information, return the bytes number that read, accel value store in *buffer
int Mpu_I2c_ReadAccel_x(unsigned char *buffer){
       Mpu_I2c_ReadReg(ACCEL_XOUT_H, 1, buffer);
       Mpu_I2c_ReadReg(ACCEL_XOUT_L, 1, buffer+1);     // read Accel,
       return 1;
}

int Mpu_I2c_ReadAccel_y(unsigned char *buffer){
       Mpu_I2c_ReadReg(ACCEL_YOUT_H, 1, buffer);
       Mpu_I2c_ReadReg(ACCEL_YOUT_L, 1, buffer+1);     // read Accel,
       return 1;
}

int Mpu_I2c_ReadAccel_z(unsigned char *buffer){
       Mpu_I2c_ReadReg(ACCEL_ZOUT_H, 1, buffer);
       Mpu_I2c_ReadReg(ACCEL_ZOUT_L, 1, buffer+1);     // read Accel,
       return 1;
}

void Mpu_I2c_ReadAccel(unsigned char *x, unsigned char *y, unsigned char *z, unsigned char *st){
	Mpu_I2c_ReadAccel_x(x);
	Mpu_I2c_ReadAccel_y(y);
	Mpu_I2c_ReadAccel_z(z);
	Mpu_I2c_ReadAndClearIntStatusRegister(st);
}

void Mpu_I2c_ReadMagASA(unsigned char *buffer){		 // read sensitivity adjustment value
	Mpu_I2c_SetReg_Mag(AK8963_CNTL, 0x00);  // Power down magnetometer
	__delay_cycles(800000);					// delay 0.1s
	Mpu_I2c_SetReg_Mag(AK8963_CNTL, 0x1F);  // Enter Fuse ROM access mode
	__delay_cycles(800000);					// delay 0.1s
	Mpu_I2c_ReadReg_Mag(AK8963_ASAX, 1, buffer);
	Mpu_I2c_ReadReg_Mag(AK8963_ASAY, 1, buffer + 1);
	Mpu_I2c_ReadReg_Mag(AK8963_ASAZ, 1, buffer + 2);    // read sensitivity data
	Mpu_I2c_SetReg_Mag(AK8963_CNTL, 0x00);  // Power down magnetometer
	__delay_cycles(800000);					// delay 0.1s
	Mpu_I2c_SetReg_Mag(AK8963_CNTL, 0x16);  // Enter continuous measurement mode
	__delay_cycles(800000);
}

int Mpu_I2c_ReadMag_x(unsigned char *buffer){
       Mpu_I2c_ReadReg_Mag(AK8963_XOUT_H, 1, buffer);
       Mpu_I2c_ReadReg_Mag(AK8963_XOUT_L, 1, buffer+1);     // read mag,
       return 1;
}

int Mpu_I2c_ReadMag_y(unsigned char *buffer){
       Mpu_I2c_ReadReg_Mag(AK8963_YOUT_H, 1, buffer);
       Mpu_I2c_ReadReg_Mag(AK8963_YOUT_L, 1, buffer+1);     // read mag,
       return 1;
}

int Mpu_I2c_ReadMag_z(unsigned char *buffer){
       Mpu_I2c_ReadReg_Mag(AK8963_ZOUT_H, 1, buffer);
       Mpu_I2c_ReadReg_Mag(AK8963_ZOUT_L, 1, buffer+1);     // read mag
       return 1;
}

void Mpu_I2c_ReadMag(unsigned char *x, unsigned char *y, unsigned char *z, unsigned char *st1, unsigned char *st2){
	if(Mpu_I2c_CheckMag(st1)){		// check mag data available or not
		Mpu_I2c_ReadMag_x(x);
		Mpu_I2c_ReadMag_y(y);
		Mpu_I2c_ReadMag_z(z);
		Mpu_I2c_ReadAndClearMagStatusRegister(st2);		// read st2 register, required
	}
}

int Mpu_I2c_CheckMag(unsigned char *buffer){
	Mpu_I2c_ReadReg_Mag(AK8963_ST1, 1, buffer);
	return *buffer & 0x01;         // return 1 if mag info ready, 0 for not
}


int Mpu_I2c_ReadTemp(unsigned char *buffer){
       Mpu_I2c_ReadReg(TEMP_OUT_H, 1, buffer);
       Mpu_I2c_ReadReg(TEMP_OUT_L, 1, buffer+1);     // read Accel,
       return 1;
}

int Mpu_I2c_ReadAndClearIntStatusRegister(unsigned char *buffer){
       Mpu_I2c_ReadReg(INT_STATUS, 1, buffer);   // i2c_status read, clear the high interrupt signal(INT_ANYRD_2CLEAR need to be reset as 0)
       return 1;
}

int Mpu_I2c_ReadAndClearMagStatusRegister(unsigned char *buffer){
       Mpu_I2c_ReadReg_Mag(AK8963_ST2, 1, buffer);   // this is required by mag, tho it does nothing here
       return 1;
}
