#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <marin_controller/Ctrl_result.h>
#include <marin_controller/Robot_state.h>

marin_controller::Robot_state robot_state;

void encCB(const marin_controller::Ctrl_result& input_msg){
  robot_state.header.stamp = ros::Time::now();
  robot_state.header.frame_id = "/base_link";
  robot_state.enc_count = input_msg.enc_count;
}

void imuCB(const sensor_msgs::Imu& input_msg){
  robot_state.orientation = input_msg.orientation;
  robot_state.angular_velocity = input_msg.angular_velocity;
  robot_state.linear_acceleration = input_msg.linear_acceleration;
}

float stat_freq;
int main(int argc, char **argv){
  ros::init(argc, argv, "robot_state_node");
  ros::NodeHandle n;
  ros::NodeHandle private_nh;
  ros::Publisher pub_stat = n.advertise<marin_controller::Robot_state> ("/robot_state", 0);
  ros::Subscriber sub_ctrlFB = n.subscribe("/controller_feedback", 10, encCB);
  ros::Subscriber sub_imu = n.subscribe("/imu/data", 10, imuCB);

  if (!private_nh.getParam ("robot_state_publish_frequency", stat_freq))
    stat_freq = 50;

  ros::Rate loop_rate(stat_freq);
  while(ros::ok()){
    pub_stat.publish(robot_state);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
