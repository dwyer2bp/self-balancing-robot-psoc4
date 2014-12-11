//This program helps to plot data from the robot.
//At least, you can see how the main.c code and this interract with eachother
//By sending floating point values converted to ascii characters.
//Messages start with an ascii 's', and end with an ascii 'e'.
//In between, there are other ascii indicators to provide more information about the message
//For example, ax means accelerometer data, x axis.

import processing.serial.*;
Serial myPort;
int startDetected=0;
//File output vars
PrintWriter output;
char[] rxMsgBuf = new char[64]; //message buffer (contains no 's' or 'e')
int rxMsgIndex; //length in bytes of msgBuffer
float[] accel = new float[3]; //(in milli-Gs)
float[] gyro = new float[3];
float[] gyro_offset = {0.6, -0.25, 0.0};
float roll, pitch, yaw;
//Complementary Filter parameters
float estRollAccel;
float estRollGyro;
float estRoll;
float estPitchAccel;
float estPitchGyro;
float estPitch;
float magAccel;
float ts = 1/50.0;
float alpha = 0.96;
float speedCommand;
//GUI PARAMETERS
int rect_w = 50;
int rect_l = 400;
void setup()
{
size(1280, 640);
noStroke();
// colorMode(RGB, 1);
frameRate(100);
myPort = new Serial(this, "COM3", 57600); //Right now, PSoC4 has been com3
//File Output
output = createWriter("balanceBot.csv");
output.println("Roll,speed_cmd");
println("Roll, speed command");
}
void draw()
{
//magAccel = sqrt( accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2] );
//Calculate estimated angle based on accelerometer sensors
estRollAccel = (180.0/PI)*atan2(-accel[0], -accel[2]);
estPitchAccel = (180.0/PI)*atan2(accel[1], accel[2]);
//Calculate estimated angle based on gyroscope sensors
estRollGyro = estRoll + (gyro[1] - gyro_offset[1])*ts;
estPitchGyro = estPitch + (gyro[0] - gyro_offset[0])*ts;
//Estimate angles using complementary filter.
// estRoll = alpha*estRollGyro - (1.0 - alpha)*estRollAccel;
estPitch = alpha*estPitchGyro + (1.0 - alpha)*estPitchAccel;
//Draw
print(estRoll);
print(",");
println(speedCommand);
//file write
output.print(estRoll);
output.print(",");
output.println(speedCommand);
background(51);
fill(255, 204);
translate(width/2, 5*height/6);
ellipse(0,0, 130, 130);
rotate((180-estRoll)*PI/180.0);
rect(-rect_w/2,0, rect_w, rect_l);
fill(255, 204);
}
void drawCube()
{
}
void serialEvent(Serial myPort)
{
while(myPort.available() > 0)
{
int inByte = myPort.read();
//Check for start of message ('s')
if (inByte == 's')
{
//Syncrhonize to start of
rxMsgIndex = 0;
startDetected = 1;
}
else if (inByte == 'e')
{
//Parse rxMsgBuf
if(startDetected == 1)
{
startDetected = 0;
parseRxMessage();
}
}
else if(startDetected == 1)
{
//If buffer not full, add incoming byte
if(rxMsgIndex < 64)
{
rxMsgBuf[rxMsgIndex] = (char)inByte;
rxMsgIndex++;
}
}
}
}
void parseRxMessage()
{
int index = 0;
//ACCELEROMETER DATA
if(rxMsgBuf[index] == 'r')
{
index++;
//this is the roll-angle from robot
int[] aRoll = new int[16];
int i = 0;
//Build array
while(index < rxMsgIndex)
{
aRoll[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
estRoll = atof(aRoll, 0, i);
}
else if(rxMsgBuf[index] == 'c')
{
index++;
//this is the roll-angle from robot
int[] aSpeedCommand = new int[16];
int i = 0;
//Build array
while(index < rxMsgIndex)
{
aSpeedCommand[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
speedCommand = atof(aSpeedCommand, 0, i);
}
else if(rxMsgBuf[index] == 'a')
{
index++;
//This is accelerometer information, continue parsing
if(rxMsgBuf[index] == 'x')
{
index++;
//This is x-accel data
int[] aAx = new int[16];
int i=0;
//Build array
while(index < rxMsgIndex)
{
aAx[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
accel[0] = atof(aAx, 0, i);
}
if(rxMsgBuf[index] == 'y')
{
index++;
//This is x-accel data
int[] aAy = new int[16];
int i=0;
//Build array
while(index < rxMsgIndex)
{
aAy[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
accel[1] = atof(aAy, 0, i);
}
if(rxMsgBuf[index] == 'z')
{
index++;
//This is x-accel data
int[] aAz = new int[16];
int i=0;
//Build array
while(index < rxMsgIndex)
{
aAz[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
accel[2] = atof(aAz, 0, i);
}
}
//GYRO DATA
else if(rxMsgBuf[index] == 'g')
{
index++;
//This is accelerometer information, continue parsing
if(rxMsgBuf[index] == 'x')
{
index++;
//This is x-accel data
int[] aGx = new int[16];
int i=0;
//Build array
while(index < rxMsgIndex)
{
aGx[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
gyro[0] = atof(aGx, 0, i);
}
if(rxMsgBuf[index] == 'y')
{
index++;
//This is x-accel data
int[] aGy = new int[16];
int i=0;
//Build array
while(index < rxMsgIndex)
{
aGy[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
gyro[1] = atof(aGy, 0, i);
}
if(rxMsgBuf[index] == 'z')
{
index++;
//This is x-accel data
int[] aGz = new int[16];
int i=0;
//Build array
while(index < rxMsgIndex)
{
aGz[i] = rxMsgBuf[index];
index++;
i++;
}
//Build float out of ascii array
gyro[2] = atof(aGz, 0, i);
}
}
}
//Turns ascii data to floating point
float atof(int[] a, int start, int end)
{
float val = 0.0;
int decFound = 0;
int sign = 1;
int i = 0;
int index_dec = end+1;
//Find index of decimal point, if any
for(i = start; i < end; i++)
{
if(a[i] == '.')
{
index_dec = i;
decFound = 1;
}
}
if(decFound == 0)
{
index_dec--;
}
//Build number (integer value)
int d = 1;
for(i = index_dec-1; i >= start; i--)
{
//Check sign...
if(a[i] == '-')
{
sign = -1;
}
else
{
val += (float)(d*( atoint( a[i]) ));
d *= 10;
}
}
if(decFound == 1){
//Append fraction to number
i = index_dec + 1;
float div = 10;
for(i = index_dec+1; i <= end; i++)
{
val += (float)( atoint( a[i])/(div) );
div *= 10.0;
}
}
val *= sign;
return val;
}
int atoint(int a)
{
return (a - 0x30);
}
//Use to close filewriter else file will be empty
void keyPressed() {
output.flush(); // Writes the remaining data to the file
output.close(); // Finishes the file
exit(); // Stops the program
}
