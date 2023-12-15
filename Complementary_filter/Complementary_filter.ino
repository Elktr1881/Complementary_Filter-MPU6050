#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <SoftwareSerial.h>

// Initialize MPU6050
MPU6050 mpu;
#define set 4

QMC5883LCompass compass;

SoftwareSerial HC12(3, 2); // RX, TX
SoftwareSerial MST(10, 11); // RX, TX

// Complementary filter variables
const float alpha = 0.95; // Complementary filter constant

float gyro_angle_x = 0, gyro_angle_y = 0; // Gyro angles
float accel_angle_x = 0, accel_angle_y = 0; // Accelerometer angles
float angle_x = 0, angle_y = 0; // Complementary filter angles

bool start = true;
int count = 0;
float Angle_x = 0, Angle_y = 0; // Complementary filter save
int pitch, roll, yaw;
int zero_yaw;

unsigned long timer = 0;
float timeStep = 0.01;

void setup() 
{
  Serial.begin(115200);
  HC12.begin(115200);
  compass.init();

  compass.read();
  zero_yaw = compass.getAzimuth();

  pinMode(set, OUTPUT);
  digitalWrite(set, HIGH);
  
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {Serial.println("Could not find a valid MPU6050 sensor, check wiring!");delay(500);}
   
}

void loop()
{
  timer = millis();
  compass.read();
  
  yaw = (compass.getAzimuth()-zero_yaw)+90;
  if(yaw > 180 || yaw < -90){yaw = 180;}
  else if(yaw < 0){yaw = 0;}
  
  // Get accelerometer data
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate accelerometer angles
  accel_angle_x = atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0 / M_PI;
  accel_angle_y = atan2(normAccel.XAxis, normAccel.ZAxis) * 180.0 / M_PI;

  // Get gyro data
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate gyro angles
  gyro_angle_x += normGyro.XAxis * 0.0000611;
  gyro_angle_y += normGyro.YAxis * 0.0000611;

  if(start)
  {
    count++;
    // Calculate complementary filter angles
    Angle_x = alpha * (angle_x + gyro_angle_x * 0.01) + (1 - alpha) * accel_angle_x;
    Angle_y = alpha * (angle_y + gyro_angle_y * 0.01) + (1 - alpha) * accel_angle_y;
    if(count > 50){start = false;}
  }
  else
  {
    // Calculate complementary filter angles
    angle_x = (alpha * (angle_x + gyro_angle_x * 0.01) + (1 - alpha) * accel_angle_x)- Angle_x;
    angle_y = (alpha * (angle_y + gyro_angle_y * 0.01) + (1 - alpha) * accel_angle_y)- Angle_y;
    pitch = angle_y + 90;
    roll  = angle_x + 90;
  }

  int a,b,c,d,e,f,g,h,i;
  a = pitch/100;
  b = pitch%100/10;
  c = pitch%100%10;
  d = roll/100;
  e = roll%100/10;
  f = roll%100%10;
  g = yaw/100;
  h = yaw%100/10;
  i = yaw%100%10;

  // Print angles
  Serial.print("pitch = ");
  Serial.print(a);
  Serial.print(b);
  Serial.print(c);
  Serial.print("  roll = ");
  Serial.print(d);
  Serial.print(e);
  Serial.print(f);
  Serial.print("  yaw = ");
  Serial.print(g);
  Serial.print(h);
  Serial.print(i);
  Serial.println();


  HC12.print(a);
  HC12.print(b);
  HC12.print(c);
  HC12.print(",");
  HC12.print(d);
  HC12.print(e);
  HC12.print(f);
  HC12.print(" ");
  HC12.print(g);
  HC12.print(h);
  HC12.print(i);
  HC12.println();
  
  delay((timeStep*1000) - (millis() - timer));
}
