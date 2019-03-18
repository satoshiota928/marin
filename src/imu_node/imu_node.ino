#include <ros.h>
#include <Arduino.h>
#include <Wire.h>
#include <std_msgs/Float32.h>

#define OFFSET_GYRO_Z 80

ros::NodeHandle nh;
std_msgs::Float32 yaw_rate;
ros::Publisher pubyaw("/yaw_rate", &yaw_rate);

double gyroX, gyroY, gyroZ;

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pubyaw);
  Wire.begin();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void loop() {
  recordGyroRegisters();
  nh.spinOnce();
  delay(100);
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
  yaw_rate.data = (gyroZ - OFFSET_GYRO_Z) / 131.0 / 180 * PI;
  pubyaw.publish(&yaw_rate);
}
