///////////////////////////////////////////////////
// Control 3 servo motors using 3 potentiometers //
///////////////////////////////////////////////////
#include <Servo.h>

// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 11

const float Pi = 3.14159;

// Control values
int Joint1Control = 512; // middle value between 0 and 1024
int Joint2Control = 512; // middle value between 0 and 1024
int Joint3Control = 512; // middle value between 0 and 1024
// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;
// Starting Joint Angles
int Joint1Angle = 0;
int Joint2Angle = 90;
int Joint3Angle = 90;

int GripperOpen = 60; // Open gripper; Need to tune value
int GripperClose = 120; // Close gripper; Need to tune value



// Joint Angle Offsets
int Joint1Offset = 101; 
int Joint2Offset = 17; 
int Joint3Offset = 9;

const float l1 = 0;   //mm
const float l2 = 95;  //mm
const float l3 = 155; //mm

// coordinates
float u[3][51];

void findTrajectory(float u0[3], float uf[3], int moveTime){
  float a0[3];
  float a1[3];
  float a2[3];
  float a3[3];

  int tf = moveTime;
  
  for (int i = 0; i <3; i++)
  {
    a0[i] = u0[i];
    a1[i] = 0;
    a2[i] = 3 * (uf[i] - u0[i]) / pow(tf, 2);
    a3[i] = -2 * (uf[i] - u0[i]) / pow(tf, 3);
    for (int t = 0; t < 51; t++)
    {
      float time_t = t*100;
      u[i][t] = a0[i] + a1[i] * time_t + a2[i] * pow(time_t, 2) + a3[i] * pow(time_t, 3);
    }
  }
}

void Move(float initialPos[3], float finalPos[3], int moveTime)
{
  findTrajectory(initialPos, finalPos, moveTime);
  for (int i = 0; i < 50; i++)
  {
    float x = u[0][i];
    float y = u[1][i];
    float z = u[2][i];
    
    // Map Analog-Digital-Converted Values into Angles
    Joint1Angle = constrain(findTheta1(x, y, z), -90, 90);
    Joint2Angle = constrain(findTheta2(x, y, z), 0,120);
    Joint3Angle = constrain(radToDeg(findTheta3(x, y, z)), -20, 160);
    
    Joint1.write(Joint1Angle+Joint1Offset);
    Joint2.write(Joint2Angle+Joint2Offset);
    Joint3.write(180-(Joint3Angle+Joint3Offset));

    delay(moveTime/50);
  }
}

void setup()
{
  Serial.begin(9600);
  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Gripper.attach(GripperPin);
  Joint1.write(Joint1Angle+Joint1Offset);
  Joint2.write(Joint2Angle+Joint2Offset);
  Joint3.write(180-(Joint3Angle+Joint3Offset));
  Gripper.write(GripperOpen); // Open gripper
  
  delay(5000); // 5 seconds before robot reads in potentiometer values
}

float radToDeg(float radAngle)
{
  return radAngle * 180 / Pi;
}

float findTheta1(float x, float y, float z)
{
  return radToDeg(atan2(y, x));
}

float findTheta2(float x, float y, float z)
{
  float theta3 = findTheta3(x,y,z);
  float k1 = l2 + l3 * cos(theta3);
  float k2 = l3 * sin(theta3);
  float r = sqrt(sq(k1) + sq(k2));
  float gamma = atan2(k2, k1);
  return radToDeg(gamma - atan2(z/r, sqrt(1-sq(z/r))));
  
}

float findTheta3(float x, float y, float z)
{
  float c3, s3;
  c3 = (sq(x) + sq(y) + sq(z) - sq(l3) - sq(l2))/(2 * l2 * l3);
  s3 = sqrt(1-sq(c3));
  return atan2(s3, c3);
}

void loop()
{
  float u0[3] = {155, 0, -95};
  float uf[3] = {0, -155, -95};
  Move(u0, uf, 5000);
  delay(10000);
  Move(uf, u0, 5000);
  delay(10000);
}
