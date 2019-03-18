#include <ros.h>
#include <Arduino.h>
#include <MsTimer2.h>
#include <Servo.h>
#include <geometry_msgs/Twist.h>
#include <marin_controller/Ctrl.h>
#include <marin_controller/Ctrl_result.h>

#define VAL_MIN 1000
#define VAL_MAX 2000
#define NEUTRAL 1500

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

ros::NodeHandle  nh;
Servo motor;  // create object to control a esc
Servo steer;  // create object to control a steering servo

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
  pub_ctrlFB.publish(&result);
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

ros::Subscriber<marin_controller::Ctrl> sub("/ackermann_cmd", &setTargetCB);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_ctrlFB);

  setPIDparam(2.0, 0.5);
  motor.attach(9);
  steer.attach(10);
  attachInterrupt(0, enc_changedPinA, CHANGE);
  attachInterrupt(1, enc_changedPinB, CHANGE);

  MsTimer2::set(20, output);
  MsTimer2::start();
}

void loop() {
  nh.spinOnce();
  delay(10);
}
