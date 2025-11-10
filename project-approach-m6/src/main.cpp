#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
Servo servo;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int valx, valy, valz;
int pos = 0, m1 = 25,m2 = 26; 
void setup() {
  Serial.begin(115200);
  servo.attach(13);
  // Change pins here if needed (SDA, SCL)
  Wire.begin(17, 16);
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  Serial.println("Initializing MPU...");
  mpu.initialize();
  
  if (mpu.testConnection())
    Serial.println("MPU6050 connected successfully.");
  else
    Serial.println("MPU6050 connection failed. Check wiring or address.");
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  digitalWrite(m1,HIGH);
  digitalWrite(m2,LOW);
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  valx = map(ax, -17000, 17000, 0, 179);
  valy = map(ay, -17000, 17000, 0, 179);
  valz = map(az, -17000, 17000, 0, 179);
  
  Serial.print("X: "); Serial.print(valx);
  Serial.print(" | Y: "); Serial.print(valy);
  Serial.print(" | Z: "); Serial.println(valz);
  
  delay(100);
}
