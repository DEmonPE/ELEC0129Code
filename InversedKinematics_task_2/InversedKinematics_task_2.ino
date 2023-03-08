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


// Control pins from potentiometer
int Pot1 = A1;
int Pot2 = A2;
int Pot3 = A3;

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
int Joint1Angle = 90;
int Joint2Angle = 90;
int Joint3Angle = 90;
int GripperOpen = 60; // Open gripper; Need to tune value
int GripperClose = 120; // Close gripper; Need to tune value



// Joint Angle Offsets
int Joint1Offset = 106; 
int Joint2Offset = 17; 
int Joint3Offset = 9;

const float l1 = 0;   //mm
const float l2 = 95;  //mm
const float l3 = 155; //mm



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
  float x, y, z;
  // Read Potentiometer Values
  int pot1read = analogRead(Pot1);
  int pot2read = analogRead(Pot2);
  int pot3read = analogRead(Pot3);

  x = map(pot1read, 0, 1023, 0, 250);
  y = map(pot2read, 0, 1023, -250, 250);
  z = map(pot3read, 0, 1023, 50, -200);

  Serial.print("\nx: ");
  Serial.print(x);
  Serial.print(",       y: ");
  Serial.print(y);
  Serial.print(",       z: ");
  Serial.print(z);
  
  Serial.print("\npot1: ");
  Serial.print(pot1read);
  Serial.print("        pot2: ");
  Serial.print(pot2read);
  Serial.print("        pot3: ");
  Serial.print(pot3read);

  
  // Map Analog-Digital-Converted Values into Angles
  Joint1Angle = constrain(findTheta1(x, y, z), -90, 90);
   Joint2Angle = constrain(findTheta2(x, y, z), 0,120);
   Joint3Angle = constrain(radToDeg(findTheta3(x, y, z)), -20, 160);

  
  
  Serial.print("\n Joint 1: ");
  Serial.print(Joint1Angle);
  Serial.print(", Joint 2: ");
  Serial.print(Joint2Angle);
  Serial.print(", Joint 3: ");
  Serial.println(Joint3Angle);

  
  Joint1.write(Joint1Angle+Joint1Offset);
  Joint2.write(Joint2Angle+Joint2Offset);
  Joint3.write(180-(Joint3Angle+Joint3Offset));

  
  delay(10);
}
