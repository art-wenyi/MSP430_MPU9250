#include <msp430f5659.h>
#include "i2c.h"
#include "msp430_uart.h"
#include "mpu9250_msp430.h"
#include "mpu9250_calibrate.h"
#include <math.h>

/*
 * main.c, for testing mpu9250, use i2c to conmunicate, uart to display to pc
 */

int mpu9250_data_ready_flag = 0; 		// indicate gyro and accel ready or not

// sensor for all data from mpu9250
struct Sensor sensor = {
		.gyro_reso = 1000.0/32768.0,	// dafault use 1000dps
		.accel_reso = 4.0/32768.0,		// default use 4g/s
		.mag_reso = 4912.0/32760.0, 	// default mag 16bit reso
		.convert = &Mpu_Convert_Data,	// register the function
		.convert_asa = &Mpu_Convert_Asa
};

// calibrated sensor data
struct Sensor_Calibrate sensor_calibrate = {
	.wbx = 0,
	.wby = 0,
	.wbz = 0,
	.bx = 1,
	.bz = 0,
	.SEq1 = 1,
	.SEq2 = 0,
	.SEq3 = 0,
	.SEq4 = 0,
	.beta = 1,
	.zeta = 0,
	.t_elapse = 0
};

void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftos(float n, char *res, int afterpoint);

int main(void) {
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	UCSCTL3 |= SELREF_2; // Set DCO FLL reference = REFO    internal 32768hz crystal
	UCSCTL4 |= SELA_2; // Set ACLK = REFO, MCLK and SMCLK use default value as DCOCLKDIV
	__bis_SR_register(SCG0);
	// Disable the FLL control loop
	UCSCTL0 = 0x0000; // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_5; // Select DCO range 16MHz operation
	UCSCTL2 = FLLD_1 + 255; // Set DCO Multiplier for 8MHz
							// (N + 1) * FLLRef = Fdco
							// (255 + 1) * 32768 = 8MHz
							// Set FLL Div = fDCOCLK/2
	__bic_SR_register(SCG0);
	// Enable the FLL control loop

	// Worst-case settling time for the DCO when the DCO range bits have been
	// changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	// UG for optimization.
	// 32 x 32 x 8 MHz / 32,768 Hz ~ 262000 = MCLK cycles for DCO to settle
	//__delay_cycles(262000);
	  P1DIR |= BIT2;
	  P1OUT |= BIT2;				// Feedback to button control chip, prevent reset
	// Loop until XT1,XT2 & DCO fault flag is cleared
	do {
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
		// Clear XT2,XT1,DCO fault flags
		SFRIFG1 &= ~OFIFG; // Clear fault flags
	} while (SFRIFG1 & OFIFG); // Test oscillator fault flag

	//== clock init finished, below are the main function ==
	
	// setting ta1, for calculate the exec time
	TA1CTL = TASSEL_2 | ID__8 | MC_2 | TACLR;         // SMCLK, clock div by 8, continuous mode, clear TAR

	UartA2_Init(115200, 'n', 'l', '8', 1);		  // init uart 2
	char sensor_data_disp[16];
	float roll,pitch,yaw;
	Mpu_Init();   // init mpu 9250
	Mpu_I2c_ReadMagASA(sensor.magasa);
	sensor.convert_asa(&sensor);
	//unsigned char tmp;
	while(1){
		//I2C_Write_Packet_To_Display(0x00,0x01,tmp);
		//I2C_Write_Packet_To_Gyro(GYRO_CONFIG, 0x01, &tmp);
		//Mpu_I2c_SetReg(GYRO_CONFIG, tmp);      // 0x18 means gyro sensitive rate is 2000dps
		//Mpu_I2c_ReadReg(INT_STATUS, 1, &tmp);
		if(Mpu_I2c_CheckDataReady()){
			sensor_calibrate.t_elapse =	TA1R;
			TA1CTL |= TACLR;		// clear ta1 count
			Mpu_I2c_ReadGyro(sensor.gyro_x, sensor.gyro_y, sensor.gyro_z, &sensor.int_status);
			Mpu_I2c_ReadAccel(sensor.accel_x, sensor.accel_y, sensor.accel_z, &sensor.int_status);
			Mpu_I2c_ReadMag(sensor.mag_x, sensor.mag_y, sensor.mag_z, &sensor.mag_st1_status, &sensor.mag_st2_status);
			sensor.convert(&sensor);
			sensor_calibrate.gxr = sensor.gyro_x_float;
			sensor_calibrate.gyr = sensor.gyro_y_float;
			sensor_calibrate.gzr = sensor.gyro_z_float;
			sensor_calibrate.axr = sensor.accel_x_float;
			sensor_calibrate.ayr = sensor.accel_y_float;
			sensor_calibrate.azr = sensor.accel_z_float;
			sensor_calibrate.mxr = sensor.mag_x_float;
			sensor_calibrate.myr = sensor.mag_y_float;
			sensor_calibrate.mzr = sensor.mag_z_float;
			PreProcess(&sensor_calibrate);
			UpdateGDFilter_MARG(&sensor_calibrate);
			roll = getRoll(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
			pitch = getPitch(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
			yaw = getYaw(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
			// display to pc from uart
			ftos(roll, sensor_data_disp, 3);
			UartA2_sendstr(sensor_data_disp);
			UartA2_sendstr("   ");
			ftos(pitch, sensor_data_disp, 3);
			UartA2_sendstr(sensor_data_disp);
			UartA2_sendstr("   ");
			ftos(yaw, sensor_data_disp, 3);
			UartA2_sendstr(sensor_data_disp);
			UartA2_sendstr("   \n");
		}
		_nop();
//		if(mpu9250_data_ready_flag == 1){
//			Mpu_I2c_ReadGyro(sensor.gyro_x, sensor.gyro_y, sensor.gyro_z, &sensor.int_status);
//			Mpu_I2c_ReadAccel(sensor.accel_x, sensor.accel_y, sensor.accel_z, &sensor.int_status);
//			Mpu_I2c_ReadMag(sensor.mag_x, sensor.mag_y, sensor.mag_z, &sensor.mag_st1_status, &sensor.mag_st2_status);
//
//		}
	}

}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    if(x==0)		// if only is 0
    	str[i++] = '0';
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// float to string
void ftos(float n, char *res, int afterpoint) 		// after point is the resolution after '.', code from geeksforgeeks web
{
	int ipart;
	float fpart;
	int i;
    if(n>0){
		// Extract integer part
		ipart = (int)n;
		// Extract floating part
		fpart = n - (float)ipart;
		// convert integer part to string
		i = intToStr(ipart, res, 0);
    }else{
    	// Extract integer part
    	n=-n;
		ipart = (int)(n);
		// Extract floating part
		fpart = n - (float)ipart;
		// convert integer part to string
		res[0]='-';
		i = intToStr(ipart, res+1, 0) + 1;
    }
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}


#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
	if (P1IFG & BIT7){		// interrupt from MPU9250, data ready to be read
		mpu9250_data_ready_flag = 1;
	}
}

/* void Port_Mapping(void){
	PMAPPWD = 0x02D52;                        // port mapping password
	PMAPCTL = PMAPRECFG; 					  // Allow reconfiguration during runtime
	P2MAP4 = PM_UCA0TXD;
	P2MAP5 = PM_UCA0RXD;
	PMAPPWD = 0;							  // Disable Write-Access to modify port mapping registers

	P2SEL |= BIT4 + BIT5;                     // active P2.4 to UCA0TXD and P2.5 to UCA0RXD module function
	P2DIR |= BIT4;                            // TXD need to be set as output direction
	P2DIR &= ~BIT5;                           // RXD input direction
	P2REN |= BIT5;                            // RXD need to set a pulldown resister to get input
	P2OUT &= ~BIT5;							  // (don't pull up)

} */
