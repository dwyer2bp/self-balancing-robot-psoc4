/* ========================================
 *
 * Self-balancing robot using PID controls
 * 
 * Feel free to use and share
 * Author: Brian Dwyer (dwyer2bp@gmail.com)
 * Date: 12/7/2014
 * 
 * ========================================
*/
#include <project.h>
#include "MPU9150.h"
#include "FloatToString.h"
#include <stdio.h>
#include <math.h>

#define PI 3.1415   //PI
#define accelAverageOn 1    //window filter parameters
#define accelAverageSize  2 //window filter parameters
#define RED 0x6
#define GREEN 0x5
#define BLUE 0x3
#define LEDOFF 0x7
#define PWM_MAX 1024
#define PWM_ZERO 790
#define PWM_MIN 512
//Function Prototypes
void txRollData();  //Transmit only the computed roll-angle
void txSpeedCommand();
void txSensorData();    //Transmit 3-axis accelerometer and 3-axis gyro data
void setMotorSpeed(int spd);
void setMotorTurn(int dir);
void updateRoll();  //updates 'estRoll
void keepBalance();
float PD_Control();
float PID_Control();

//Gyro Offsets
float gyro_offset[3] = {0.6, -0.25, 0.0};

//Startup OK indicator
int startupOk = 0; //0 is not-ok, 1 is ok

//window filter parameters
int windowAccelIndex = 0;
float accelBufferX[accelAverageSize];
float accelBufferZ[accelAverageSize];

//Complementary Filter parameters
float estRollAccel;
float estRollGyro;
float estRoll;
float estPitchAccel;
float estPitchGyro;
float estPitch;
float magAccel;
float ts = 0.01;
float alpha = 0.99;
//PID Parameters (favorite so far: kp = 8.0, kd = 4.0, ki = 0.4 //oscillation without disturbance, but good reaction time to disturbance. Little robot travel
//no, this is my favorite: 8,10,.5: Oscillations are small, but needed to limit robot travel
//This is ok: kp = 15.0, kd = 60.0, ki = 0.9 // Good control on angle, but robot drives too much
//Still drives too much, but not as badly: 10, 40, .5
float pfactor, dfactor, ifactor;
float imax = 200;
const float kp = 8.0;
const float kd = 10; //Increasing controls overshoot and cancels oscillations,  but the robot must travel further to control angle
const float ki = 0.5;
float currentError;
float lastError;
float targetAngle = 10.0;
//Keep the balance
int last_spd_cmd;
int spd_cmd;
float lastRoll;

//UART Variables
char uartTX_buffer[64];
char sFloat[32];    //Used to convert a float to a string

int main()
{   
    CyDelay(10);        //This Delay is needed so the UART + I2C get properly initialized. 
                        //(Could have something to do with being powered from battery?)
    UART_Start();       //Initialize the UART
    I2C_1_Start();      //Intialize SCB for I2C (IMU Connection)
    MCU_PWM_Start();    //Initialize MCU PWM Block
    CyGlobalIntEnable;  //Enable global interrupts 
    pRGB_Write(LEDOFF);      //Turn off LEDs
    MPU9150_Init();     //Initialize the MPU9150 via I2C

    //Checks to see if I2C is functioning, or if there is an error
    if(updateIMU() == 0)
    {
        pRGB_Write(GREEN);  //Turn on only GREEN 
        startupOk = 1;
    }
    else
    {
        pRGB_Write(RED);  //Turn on only RED (to indicate a failed startup);
    }

    //Make sure motors are turned off
    setMotorSpeed(0);
    setMotorTurn(0);
    int cycle_indicator;
    for(;;)
    {
        updateIMU();    //Updates gyro, accel, compass variables of the MPU9150
        updateRoll();   //re-calculate the roll-angle
        keepBalance();  //Control for keeping balance
        //setMotorSpeed(50);
        txRollData();   //Transmit roll-angle through uart
       txSpeedCommand();   //Transmits speed command through uart
        //txSensorData(); //Transmits gyro/accel data through uart
        
        //Toggeling the green led to get an indication of loop frequency
        //Scoped the green led, and saw 32Hz frequency (meaning 64Hz loop frequency)
        //This was used to set the 'ts' (time sample) variable
        if(startupOk)
        {
            if(cycle_indicator == 1){
                pRGB_Write(GREEN);
                cycle_indicator = 0;
            }
            else
            {
                pRGB_Write(LEDOFF); 
                cycle_indicator = 1;
            }      
        }

        CyDelay(6);
    }
}

//UART transmit to processing application (accel + gyro data)
void txSensorData()
{
    //Send accel data (6 bytes of sensor data)
    ftostring(accel[0]*1000.0, sFloat);
    snprintf(uartTX_buffer, 64, "sax%se",sFloat);
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ax' for accelerometer x value) + end 'e')
    ftostring(accel[1]*1000.0, sFloat);
    snprintf(uartTX_buffer, 64, "say%se", sFloat );
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ay' for accelerometer x value) + end 'e')
    ftostring(accel[2]*1000.0, sFloat);
    snprintf(uartTX_buffer, 64, "saz%se", sFloat );
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('az' for accelerometer x value) + end 'e')
    
    ftostring(gyro[0], sFloat);
    snprintf(uartTX_buffer, 64, "sgx%se",sFloat);
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ax' for accelerometer x value) + end 'e')
    ftostring(gyro[1], sFloat);
    snprintf(uartTX_buffer, 64, "sgy%se", sFloat );
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ay' for accelerometer x value) + end 'e')
    ftostring(gyro[2], sFloat);
    snprintf(uartTX_buffer, 64, "sgz%se", sFloat );
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('az' for accelerometer x value) + end 'e')
    /*
    ftostring(accel[0], sFloat);
    snprintf(uartTX_buffer, 64, "sax%se",sFloat);
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ax' for accelerometer x value) + end 'e')
    ftostring(accel[1], sFloat);
    snprintf(uartTX_buffer, 64, "say%se", sFloat );
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ay' for accelerometer x value) + end 'e')
    ftostring(accel[2], sFloat);
    snprintf(uartTX_buffer, 64, "saz%se", sFloat );
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('az' for accelerometer x value) + end 'e')
    */
}

//UART transmit to processing application (roll only)
void txRollData()
{
    //Send accel data (6 bytes of sensor data)
    ftostring(estRoll, sFloat);
    snprintf(uartTX_buffer, 64, "sr%se",sFloat);
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ax' for accelerometer x value) + end 'e')
}

//UART transmit to processing application (roll only)
void txSpeedCommand()
{
    //Send accel data (6 bytes of sensor data)
    //ftostring((float)spd_cmd, sFloat);
    snprintf(uartTX_buffer, 64, "sc%de",spd_cmd);
    UART_UartPutString(uartTX_buffer);  //start + 2-char command ('ax' for accelerometer x value) + end 'e')
}

//8-bit speed control
void setMotorSpeed(int spd)
{
    int pwm_temp = PWM_ZERO + spd;
    
    //Saturate command if too high
    if(pwm_temp > PWM_MAX)
    {
        pwm_temp = PWM_MAX;
    }
    else if(pwm_temp < PWM_MIN)
    {
        pwm_temp = PWM_MIN;
    }
    
    MCU_PWM_WriteCompare1(pwm_temp);  
}

//8-bit turn direction control
void setMotorTurn(int dir)
{
    int pwm_temp = PWM_ZERO + dir;
    
    //Saturate command if too high
    if(pwm_temp > PWM_MAX)
    {
        pwm_temp = PWM_MAX;
    }
    else if(pwm_temp < PWM_MIN)
    {
        pwm_temp = PWM_MIN;
    }
    
    MCU_PWM_WriteCompare2(pwm_temp);     
}


void keepBalance()
{
    //Update speed command
    //spd_cmd = (int)PD_Control();
    spd_cmd = (int)PID_Control();
    //If angle is too large, give up!
    if(estRoll > 60 || estRoll < -60)
    {
        spd_cmd = 0;  
    }
       
    setMotorSpeed(spd_cmd);
}

void updateRoll()
{
    float accel_x, accel_z;
    int i;
    
    lastRoll = estRoll;
    
    //If window averaging is turned on for accelerometer data, then average
    if(accelAverageOn)
    {
        //Update array only at windowIndex
        accelBufferX[windowAccelIndex] = accel[0];
        accelBufferZ[windowAccelIndex] = accel[2];
        windowAccelIndex++;
        //Check to see if windowIndex has reached its max, and reset it
        if(windowAccelIndex >= accelAverageSize)
        {
            windowAccelIndex = 0;
        }
        //Calculate average of 15 samples
        accel_x = 0.0;
        accel_z = 0.0;
        for(i = 0; i < accelAverageSize; i++)
        {
            accel_x += accelBufferX[i];
            accel_z += accelBufferZ[i];
        }
        accel_x = accel_x / accelAverageSize;  
        accel_z = accel_z / accelAverageSize; 
    }
    else
    {
        accel_x = accel[0];
        accel_z = accel[2];
    }
   //Calculate estimated angle based on accelerometer sensors
  estRollAccel =  (180.0/PI)*atan2(-accel_x, -accel_z);
  //estPitchAccel = (180.0/PI)*atan2(accel[1], accel[2]);
  
  //Calculate estimated angle based on gyroscope sensors
  estRollGyro =  estRoll  + (gyro[1] - gyro_offset[1])*ts;
 // estPitchGyro = estPitch + (gyro[0] - gyro_offset[0])*ts;
  
  //Estimate angles using complementary filter
  estRoll = alpha*estRollGyro - (1.0 - alpha)*estRollAccel;
  //estPitch = alpha*estPitchGyro + (1.0 - alpha)*estPitchAccel;   
}

float PD_Control()
{
    lastError = currentError;   //Update last error
    currentError = targetAngle - estRoll;   //Update current error
    pfactor = kp*currentError;
    dfactor = kd*(lastError - currentError);
   // float dfactor = kd*(gyro[1] - gyro_offset[1])*ts;
    float pd = pfactor + dfactor;
    
    return pd;
}


float PID_Control()
{
    lastError = currentError;   //Update last error
    currentError = targetAngle - estRoll;   //Update current error
    pfactor = kp*currentError;
    dfactor = kd*(currentError - lastError);

    ifactor += ki*currentError;
    if(ifactor >= imax)
    {
        ifactor = imax;
    }
    else if(ifactor <= -imax)
    {
        ifactor = -imax;
    }

    float pid = pfactor + dfactor + ifactor;
    
    return pid;
}
/* [] END OF FILE */
