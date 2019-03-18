#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <marin_controller/Ctrl_result.h>
#include <marin_controller/Robot_state.h>

class State
{
private:
  ros::NodeHandle private_nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  tf::TransformBroadcaster br;
  geometry_msgs::TransformStamped imu_tf;
  geometry_msgs::Quaternion q;
  marin_controller::Robot_state robot_state;

public:
  State()
  {
    sub = private_nh.subscribe ("control_result", 10, &State::ctrl_cb, this);
    sub = private_nh.subscribe ("/imu/data", 10, &State::imu_cb, this);
    pub = private_nh.advertise<marin_controller::Robot_state> ("robot_state", 0);
  }

  void ctrl_cb (const marin_controller::Ctrl_result& input_msg)
  {
    try{
      robot_state.header.stamp = ros::Time::now();
      robot_state.header.frame_id = "global";
      robot_state.enc_count = input_msg.enc_count;
    }
    catch(...){
      ROS_ERROR("Exception");
    }
    //pub.publish (robot_state);
  }

  void imu_cb (const sensor_msgs::Imu& input_msg)
  {
    try{
      robot_state.orientation = input_msg.orientation;
      robot_state.angular_velocity = input_msg.angular_velocity;
      robot_state.linear_acceleration = input_msg.linear_acceleration;
    }
    catch(...){
      ROS_ERROR("Exception");
    }
    //pub.publish (robot_state);
  }

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "imu_node");
  State Robot_state;
  ros::spin();
}
