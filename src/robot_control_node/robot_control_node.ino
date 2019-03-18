#include <ros.h>
#include <Arduino.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <Servo.h>
#include <geometry_msgs/Twist.h>
#include <marin_controller/Ctrl.h>
#include <marin_controller/Ctrl_result.h>
#include <sensor_msgs/Imu.h>

#define VAL_MIN 1000
#define VAL_MAX 2000
#define NEUTRAL 1500

#define OFFSET_GYRO_X 0
#define OFFSET_GYRO_Y 0
#define OFFSET_GYRO_Z 80

const int pinA = 2;
const int pinB = 3;

volatile int enc_count;
int tgt_pulse;
float tgt_angle;
float Kp_current;
float Ki_current;
int old_error;
int control_amount;
int current_error;
int motor_control_val;

float steer_servo_val;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

ros::NodeHandle  nh;
Servo motor;  // create object to control a esc
Servo steer;  // create object to control a steering servo

sensor_msgs::Imu imu;
ros::Publisher pub_imu("/imu/data_raw", &imu);
marin_controller::Ctrl_result result;
ros::Publisher pub_ctrlFB("/controller_feedback", &result);

void enc_changedPinA(){
  if(digitalRead(pinA)){
    if(digitalRead(pinB)) --enc_count;  //逆転
    else ++enc_count;                   //正転
  } else {
    if(digitalRead(pinB)) ++enc_count;  //正転
    else --enc_count;                   //逆転
  }
}

void enc_changedPinB(){
  if(digitalRead(pinB)){
    if(digitalRead(pinA)) ++enc_count;  //正転
    else --enc_count;                   //逆転
  } else {
    if(digitalRead(pinA)) --enc_count;  //逆転
    else ++enc_count;                   //正転
  }
}

void setTargetCB(const marin_controller::Ctrl& msg){
  tgt_pulse = msg.tgt_pulse;
  tgt_angle = msg.tgt_steer_angle;
}

void setPIDparam(float feedback_gain_p, float feedback_gain_i){
  Kp_current = feedback_gain_p; Ki_current = feedback_gain_i;
}

void output(){
  current_error = tgt_pulse - enc_count;
  int control_input = Kp_current * (current_error - old_error) + Ki_current * current_error;
  control_amount += control_input;

  old_error = current_error;
  control_amount = min(150, control_amount);
  control_amount = max(-150, control_amount);
  motor_control_val = control_amount + NEUTRAL;
  steer_servo_val = 90 + tgt_angle  / 20 * 45;
  steer_servo_val = min(135, steer_servo_val);
  steer_servo_val = max(45, steer_servo_val);
  motor.writeMicroseconds(motor_control_val);
  steer.write(steer_servo_val);

  setRobotData();
  //noInterrupts();
  pub_ctrlFB.publish(&result);
  //interrupts();
  setIMUData();
  //noInterrupts();
  pub_imu.publish(&imu);
  //interrupts();
  enc_count = 0;
}

void setRobotData() {
  result.header.stamp = nh.now();
  result.tgt_pulse = tgt_pulse;
  result.enc_count = enc_count;
  result.pulse_error = current_error;
  result.esc_input = motor_control_val;
  result.current_steer_angle =  steer_servo_val;
}

void setupMPU6050() {
  //MPU6050との通信を開始し、ジャイロと加速度の最大範囲を指定
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x6B); //Accessing the register 6B
  Wire.write(0b00000000); //SLEEP register to 0
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration
  Wire.write(0x00000000); //gyro to full scale ± 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration
  Wire.write(0b00000000); //accel to +/- 2g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  //加速度読み取り
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B); // Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  calculateAccelData();
}

void calculateAccelData() {
  //読み取った値をgに変換
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  //ジャイロの値を読み取る
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  calculateGyroData();
}

void calculateGyroData() {
  //読み取った値をrad/secに変換
  rotX = (gyroX - OFFSET_GYRO_X) / 131.0 / 180 * PI;
  rotY = (gyroY - OFFSET_GYRO_Y) / 131.0 / 180 * PI;
  rotZ = (gyroZ - OFFSET_GYRO_Z) / 131.0 / 180 * PI;
}

void setIMUData() {
  imu.header.frame_id = "imu_link";
  imu.header.stamp = nh.now();
  imu.angular_velocity.x = rotX;
  imu.angular_velocity.y = rotY;
  imu.angular_velocity.z = rotZ; // [rad/sec]
  imu.linear_acceleration.x = gForceX;
  imu.linear_acceleration.y = gForceY;
  imu.linear_acceleration.z = gForceZ;
}

ros::Subscriber<marin_controller::Ctrl> sub("/ackermann_cmd", &setTargetCB);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_imu);
  nh.advertise(pub_ctrlFB);

  Wire.begin();

  setupMPU6050();

  setPIDparam(2.0, 0.5);
  motor.attach(9);
  steer.attach(10);
  attachInterrupt(0, enc_changedPinA, CHANGE);
  attachInterrupt(1, enc_changedPinB, CHANGE);

  MsTimer2::set(20, output);
  MsTimer2::start();
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  nh.spinOnce();
  delay(10);
}
