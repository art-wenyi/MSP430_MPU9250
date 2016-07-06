/*
 * mpu9250_calibrate.h
 *
 *  Created on: Jul 1, 2016
 *      Author: art
 */

#ifndef MPU9250_CALIBRATE_H_
#define MPU9250_CALIBRATE_H_
// Euler sequence: roll(x),pitch(y),yaw(z)
#define RAD2DEG 57.295780
#define DEG2RAD 0.017453
#define dtTarget 20000
#define MXB 9.85
#define MYB 44.69
#define MZB 10.14
#define MXS 0.94
#define MYS 1.06
#define MZS 1.00
#define GXB -3.02
#define GYB 1.68
#define GZB 0.72
#define GYRO_BIAS_DRIFT 0

struct Sensor_Calibrate{
	float mxr,myr,mzr;		// mag raw data
	float mx,my,mz;			// mag data processed
	float axr,ayr,azr;		// accel raw
	float ax,ay,az;			// accel
	float gxr,gyr,gzr;		// gyro raw
	float gx,gy,gz;			// gyro

	float beta,zeta;
	float wbx, wby, wbz;
	float bx, bz;
	//float wbx = 0, wby = 0, wbz = 0;
	//float bx = 1, bz = 0;

	float SEq1, SEq2, SEq3, SEq4;
	//float SEq1 = 1, SEq2 = 0, SEq3 = 0, SEq4 = 0;

	unsigned int  t_elapse;
	float dt;
};

void PreProcess(struct Sensor_Calibrate *sc);
void UpdateGDFilter_MARG(struct Sensor_Calibrate *sc);
void DLP(float *val_old, float val_new, float alpha);

float getPitch(float q0, float q1, float q2, float q3);
float getYaw(float q0, float q1, float q2, float q3);
float getRoll(float q0, float q1, float q2, float q3);

#endif /* MPU9250_CALIBRATE_H_ */
