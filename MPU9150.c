/*
	Library (work in progress) written for the Invensense MPU9150
	This was library was written in C for the PSoC4 ARM.
	My original library was in CPP, but PSoC Creator does not
	support CPP :0(
	
	Author: B Dwyer

*/

#include "MPU9150.h"

//I2C rx/tx Buffers
uint8 rxBuffer[16];
uint8 txBuffer[16];

// Init mpu9150
void MPU9150_Init() {
    //Wake up the MPU, configure the clk source
	setClockSource(MPU9150_CLOCK_PLL_XGYRO);
	setSleepEnabled(0);
    
    //Do custom settings here (ie. configuring full-scale sensitivity)
    setFullscaleAccel(AFS_SEL_4G);  //Default is 2G if not changed
    
    //Reads all sensor configurations to acquire sensitivity settings
	readConfig();
    
    //Initializes the compass (if not needed, comment out)
    initCompass();
}

void setClockSource(int src)
{
	txBuffer[0] = rPWR_MGMT_1;
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    txBuffer[1] = (rxBuffer[0]&0xF8) | (src & 0x07);
    
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,2, I2C_1_I2C_MODE_COMPLETE_XFER);
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
}

void setSleepEnabled(int sleep)
{
	txBuffer[0] = rPWR_MGMT_1;
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	//While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
	if(sleep){
		txBuffer[1] = (rxBuffer[0]) | (MPU9150_PWR1_SLEEP_MASK);
	}
	else{
		txBuffer[1] = (rxBuffer[0]) & (~MPU9150_PWR1_SLEEP_MASK);
	}
    
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,2, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
}

// Checks the "who am I register"
// Returns true if device responds with 0x68
// Else returns false
//
int testConnection() {
	txBuffer[0] = rWHOAMI;  
	rxBuffer[0] = 0;
  
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }

	if( rxBuffer[0] != MPU9150_ADDRESS )
	{
		return 0;
	}
	return 1;
}



int isAsleep()
{
	txBuffer[0] = rPWR_MGMT_1; 
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
	if(rxBuffer[0] & 0x40)
	{
		return 1;	//Yes, it is in sleep mode
	}
	else
	{
		return 0;	//no, it is not in sleep mode
	}
}

void readConfig()
{
	readPowerConfig();
	readAccelConfig();
	readGyroConfig();
}

int readPowerConfig()
{
    txBuffer[0] = rPWR_MGMT_1; 
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,2, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    return( rxBuffer[1] | (rxBuffer[0] << 8));
}

int readAccelConfig()
{
	txBuffer[0] = rACCEL_CONFIG;
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
	//decode scaler
	fs_accel = (read_AFS_SEL+1)*2;  //gravity (g's)
	scaler_accel = (float)(32768.0 / fs_accel);
	return (int)rxBuffer[0];
}

void setFullscaleAccel(int fs)
{
    if( (fs >= AFS_SEL_2G) && (fs <= AFS_SEL_16G) )
    {
        txBuffer[0] = rACCEL_CONFIG;
    	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
    	
        //While I2C_Master is busy, wait
        while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
        {
            CyDelayUs(1);
        }
        
        I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
    	
        //While I2C_Master is busy, wait
        while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
        {
            CyDelayUs(1);
        }    
        //Build new accel config register data
        txBuffer[1] =  ( rxBuffer[0]&(~mACCEL_CONFIG_AFS_SEL) ) | (fs << 3);
        
        I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,2, I2C_1_I2C_MODE_COMPLETE_XFER);
    	
        //While I2C_Master is busy, wait
        while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
        {
            CyDelayUs(1);
        }
    }

    
}

int readGyroConfig()
{
	txBuffer[0] = rGYRO_CONFIG;
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);

    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
	//decode scaler
	fs_gyro = 250*(read_GFS_SEL+1); //Degrees/Second
	scaler_gyro = (float)(32768.0 / fs_gyro);
	return (int)rxBuffer[0];
}


int updateIMU()
{
	short temp;
	int index;
    int i;
	txBuffer[0] = rACCEL_XOUT; 
    
    //Write to MPU9150 the Accelerometer X-Axis address (start of all sensor data)
	I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        //Check to see if there was an error, if so then clear master status to prevent a frozen state
        if(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_ERR_MASK)
        {
            return I2C_1_I2CMasterStatus();
        }
        CyDelayUs(1);
    }
    
    //Read 14 bytes of data (accel, temperature and gyro data)
    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,14, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        //Check to see if there was an error, if so then clear master status to prevent a frozen state
        if(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_ERR_MASK)
        {
            return I2C_1_I2CMasterStatus();
        }
        CyDelayUs(1);
    }
    
	//rxBuffer now holds accel, temperature and gyro data.
    //accel: 6bytes, temperature:2bytes, gyro:6bytes
	index = 0;
	for(i = 0; i < 3; i++)
	{
		temp = rxBuffer[index+1] | (rxBuffer[index] << 8);
        rAccel[i] = temp;
		accel[i] = (float)(temp / scaler_accel);
		index += 2;
	}
	temp = rxBuffer[index+1] | (rxBuffer[index] << 8);
	temperature = (int)(temp/340 + 35);
	index += 2;
	for(i = 0; i < 3; i++)
	{
		temp = rxBuffer[index+1] | (rxBuffer[index] << 8);
        rGyro[i] = temp;
		gyro[i] = (float)(temp / scaler_gyro);
		index += 2;
	}

	//updateCompass();	//Update compass (through i2c bypass mode if enabled)
    return 0;
}

// Initialize Compass: AK8975_ADDRESS
//	Enables I2C Bypass Mode
//
void initCompass()
{
	setBypassI2C(1);
	
	//Setup for single measurement mode	
	txBuffer[0] = rMAG_CNTL;
	txBuffer[1] = 1;
	I2C_1_I2CMasterWriteBuf(AK8975_ADDRESS, txBuffer,2, I2C_1_I2C_MODE_COMPLETE_XFER);
	
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
    
    compass_en = 1;
	fs_mag = 1229;  //Full uTesla range of sensor
	scaler_mag = (float)(4095.0 / fs_mag);
}

void updateCompass()
{
    int i;
    int mag_dataValid;
	if(compass_en){
	    
	    //Check if data is ready
	    txBuffer[0] = rMAG_ST1; 
        I2C_1_I2CMasterWriteBuf(AK8975_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
        
        //While I2C_Master is busy, wait
        while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
        {
            //Check to see if there was an error, if so then clear master status to prevent a frozen state
            if(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_ERR_MASK)
            {
                return;
            }
            CyDelayUs(1);
        }
        
        I2C_1_I2CMasterReadBuf(AK8975_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
	    
        //While I2C_Master is busy, wait
        while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
        {
            //Check to see if there was an error, if so then clear master status to prevent a frozen state
            if(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_ERR_MASK)
            {
                return;
            }
            CyDelayUs(1);
        }
        
        //Check if mag data is valid
        mag_dataValid = rxBuffer[0] & 0x01;
        
        //If it is valid, update the mag data
	    if(mag_dataValid == 1){
		    short temp;
		    txBuffer[0] = rMAG_XOUT_L;
            
			I2C_1_I2CMasterWriteBuf(AK8975_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
            //While I2C_Master is busy, wait
            while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
            {
                //Check to see if there was an error, if so then clear master status to prevent a frozen state
                if(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_ERR_MASK)
                {
                    return;
                }
                CyDelayUs(1);
            }

            I2C_1_I2CMasterReadBuf(AK8975_ADDRESS, rxBuffer,6, I2C_1_I2C_MODE_COMPLETE_XFER);
            
            //While I2C_Master is busy, wait
            while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
            {
                //Check to see if there was an error, if so then clear master status to prevent a frozen state
                if(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_ERR_MASK)
                {
                    return;
                }
                CyDelayUs(1);
            }
            
		    for(i = 0; i < 3; i++)
		    {
			    temp = (rxBuffer[2*i]) | (rxBuffer[i*2 + 1]<<8);
                rMag[i] = temp;
			    mag[i] = (float)(temp / scaler_mag);
		    }
		    //Setup for single measurement mode
	        txBuffer[0] = rMAG_CNTL;
	        txBuffer[1] = 1;
			I2C_1_I2CMasterWriteBuf(AK8975_ADDRESS, txBuffer,2, I2C_1_I2C_MODE_COMPLETE_XFER);
            
		    //While I2C_Master is busy, wait
            while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
            {
                //Check to see if there was an error, if so then clear master status to prevent a frozen state
                if(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_ERR_MASK)
                {
                    return;
                }
                CyDelayUs(1);
            }
        }
	}
}

//	Sets the 'I2C_BYPASS_EN' bit
//	register: 'INT_PIN_CFG'
//
void setBypassI2C(int set)
{
    txBuffer[0] = rINT_PIN_CFG; 
    
    I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
  
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }

    I2C_1_I2CMasterReadBuf(MPU9150_ADDRESS, rxBuffer,1, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }

    if(set)
    {
        txBuffer[1] = (rxBuffer[0]) | (mINT_PIN_CFG_I2C_BYPASS_EN);	// Enable i2c bypass mode
    }
    else
    {
        txBuffer[1] = (rxBuffer[0]) & (~mINT_PIN_CFG_I2C_BYPASS_EN);// Disabe i2c bypass mode
    }
    
    I2C_1_I2CMasterWriteBuf(MPU9150_ADDRESS, txBuffer,2, I2C_1_I2C_MODE_COMPLETE_XFER);
    
    //While I2C_Master is busy, wait
    while(I2C_1_I2CMasterStatus()&I2C_1_I2C_MSTAT_XFER_INP)
    {
        CyDelayUs(1);
    }
}
