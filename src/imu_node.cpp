#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

class IMU
{
private:
  ros::NodeHandle private_nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  sensor_msgs::Imu imu;

public:
  IMU()
  {
    sub = private_nh.subscribe ("/yaw_rate", 1, &IMU::imu_cb, this);
    pub = private_nh.advertise<sensor_msgs::Imu> ("/imu/data_raw", 0);
  }
  void imu_cb (const std_msgs::Float32& input_msg)
  {
    imu.header.frame_id = "imu_link";
    imu.header.stamp = ros::Time::now();
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = input_msg.data;
    imu.linear_acceleration.x = 0;
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 0;
    pub.publish (imu);
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "imu_node");
  IMU Marin_Imu;
  ros::spin();
}
