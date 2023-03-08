const float l1 = 0;
const float l2 = 9.5;
const float l3 = 15.5;

const float Pi = 3.14159;




void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
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


void loop() {
  // put your main code here, to run repeatedly:
  float x, y, z;
  x = 0;
  y = -15.5;
  z = -9.5;
  float Joint1Angle = findTheta1(x, y, z);
  float Joint2Angle = findTheta2(x, y, z);
  float Joint3Angle = radToDeg(findTheta3(x, y, z));

  Serial.print("\n1: ");
  Serial.print(Joint1Angle);
  Serial.print(",       2: ");
  Serial.print(Joint2Angle);
  Serial.print(",       3: ");
  Serial.print(Joint3Angle);
  delay(100);


}
