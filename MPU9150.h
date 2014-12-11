/*
	Library (work in progress) written for the Invensense MPU9150
	This was library was written in C for the PSoC4 ARM.
	My original library was in CPP, but PSoC Creator does not
	support CPP :0(
	
	Author: B Dwyer

*/
#include <project.h>

/* MPU9150: Register Map */
#define rSMPLRT_DIV		0x19	// Sample Rate Divider Register
#define	rCONFIG			0x1A	// Config register
#define	rGYRO_CONFIG	0x1B	// Gyro Config register
#define	rACCEL_CONFIG	0x1C	// Accelerometer Config Register
#define rFIFO_EN		0x23	// FIFO Enable Register
#define rI2C_MSG_CTRL	0x24	// I2C Master Control Register
#define	rINT_PIN_CFG	0x37	// Interrupt-Pin / I2C Bypass Config Register
#define rINT_ENABLE		0x38	// Interrupt Pin Enable Register
#define	rINT_STATUS		0x39	// Interrupt Status Register
#define	rACCEL_XOUT		0x3B	// 16bit Accelerometer X (MSB First)
#define	rACCEL_YOUT		0x3D	// 16bit Accelerometer Y (MSB First)
#define	rACCEL_ZOUT		0x3F	// 16bit Accelerometer Z (MSB First)
#define	rTEMP_OUT		0x41	// 16bit Temperature(MSB First)
#define	rGYRO_XOUT		0x43	// 16bit Gyrometer X (MSB First)
#define	rGYRO_YOUT		0x45	// 16bit Gyrometer Y (MSB First)
#define	rGYRO_ZOUT		0x47	// 16bit Gyrometer Z (MSB First)
#define rUSER_CTRL		0x6A	// User Control Register
#define rPWR_MGMT_1		0x6B	// Power Management Register 1
#define rPWR_MGMT_2		0x6C	// Power Management Register 2
#define rDMP_BANK_ADR	0x6D	// DMP Bank Address Register
#define	rDMP_MEM_R_W	0x6F	// DMP Memory Read/Write Register
#define rWHOAMI			0x75	// Device ID (WHO AM I) Register
/* AK8975 (Compass): Register Map */
#define rMAG_ST1            0x02
#define rMAG_XOUT_L			0x03
#define rMAG_XOUT_H			0x04
#define rMAG_YOUT_L			0x05
#define rMAG_YOUT_H			0x06
#define rMAG_ZOUT_L			0x07
#define rMAG_ZOUT_H			0x08
#define rMAG_CNTL			0x0A
/*Accelerometer Full-Scale Settings*/
#define AFS_SEL_2G 0
#define AFS_SEL_4G 1
#define AFS_SEL_8G 2
#define AFS_SEL_16G 3
/* Useful Register Masks */
#define mCONFIG_EXT_SYNC_SET	0x38	//Clock sync set
#define mACCEL_CONFIG_AFS_SEL	0x18	//Accelerometer Full Scale
#define read_AFS_SEL	((rxBuffer[0]&mACCEL_CONFIG_AFS_SEL)>>3)
#define mGYRO_CONFIG_GFS_SEL	0x18		//Gyrometer Full Scale
#define read_GFS_SEL	((rxBuffer[0]&mGYRO_CONFIG_GFS_SEL)>>3)
#define MPU9150_PWR1_SLEEP_MASK 	0x40
#define mINT_PIN_CFG_I2C_BYPASS_EN				0x02
/* I2C Slave ID Defines */
#define MPU9150_ADDRESS	0x68
#define AK8975_ADDRESS	0x0C
/* Internal Clock Defines */
#define MPU9150_CLOCK_INTERNAL          0x00
#define MPU9150_CLOCK_PLL_XGYRO         0x01
#define MPU9150_CLOCK_PLL_YGYRO         0x02
#define MPU9150_CLOCK_PLL_ZGYRO         0x03
#define MPU9150_CLOCK_PLL_EXT32K        0x04
#define MPU9150_CLOCK_PLL_EXT19M        0x05
#define MPU9150_CLOCK_KEEP_RESET        0x07

void MPU9150_Init();
int testConnection();			//Reads "rWHOAMI" register, checks against devAddr
int isAsleep();					//Returns '1' if sleep is enabled (rPWR_MGMT_1)

//Update IMU Sensor Data
int updateIMU();                //Updates all 'Global IMU Sensor Data Variables'
void updateCompass();

//IMU Config Functions
void setClockSource(int clk_src);   //Sets IMU clock source. Default is to Gyro clk
void setSleepEnabled(int sleep);    //Send a 0 to set awake, 1 for sleep
void initCompass();					//Sets i2c to bypass mode and enables 3axis compass
void setBypassI2C(int set);         //Sets i2c to bypass mode
void readConfig();				    //Reads all General Config registers, and updates all 'IMU Sensor Config Variables'
int readAccelConfig();
int readGyroConfig();
int readPowerConfig();
void setFullscaleAccel(int fs);

//IMU Sensor Config Variables
int fs_accel;					//Full scale Accelerometer Reading (can be 2,4,8,16 x gravity)
float scaler_accel;				//Accelerometer Bit resolution (LSB/g)
int fs_gyro;					//Full scale Gyro Reading (can be 250,500,1000,2000 deg/s)
float scaler_gyro;				//Gyro Bit resolution (LSB/(deg/s)) or
int fs_mag;                     //Full scale Magnetomter (hardcoded to 1200 uTesla)
float scaler_mag;               //Magnetometer Bit Resolution (LSB/uTesla)
int compass_en;                 //Magnetometer 'enabled' variable (read only)

//Global IMU Sensor Data Variables (Engineering Units (Float))
float accel[3];					//Array containing Accelerometer data [X,Y,Z], units g (or 9.8m/s^2)
float gyro[3];					//Array containing Gyrometer data [X,Y,Z], units deg/s
float mag[3];				    //Array containing Magnetometer data [X,Y,Z], units ?
int temperature;				//In Degrees Celsius

//Global IMU Sensor Data Variavbles (Raw dats (unit-less))
int rAccel[3];
int rGyro[3];
int rMag[3];
