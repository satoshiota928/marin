#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class IMU
{
private:
  ros::NodeHandle private_nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  tf::TransformBroadcaster br;
  geometry_msgs::TransformStamped imu_tf;
  geometry_msgs::Quaternion q;

public:
  IMU()
  {
    sub = private_nh.subscribe ("imu_rpy", 1, &IMU::imu_cb, this);
    //pub = private_nh.advertise<marin_controller::Ctrl> ("control_cmd", 0);
  }
  void imu_cb (const geometry_msgs::Vector3& input_msg)
  {
    // Convert to control command
    try{
      tf::Quaternion quat = tf::createQuaternionFromRPY(input_msg.x,input_msg.y,input_msg.z);
      quaternionTFToMsg(quat,q);
      //現在の時間の格納
      imu_tf.header.stamp = ros::Time::now();

      //座標系globalとrobotの指定
      imu_tf.header.frame_id = "global";
      imu_tf.child_frame_id  = "imu_link";

      //global座標系からみたrobot座標系の原点位置と方向の格納
      imu_tf.transform.translation.x = 0;
      imu_tf.transform.translation.y = 0;
      imu_tf.transform.translation.z = 0;
      imu_tf.transform.rotation = q;
    }

    catch(...){
      ROS_ERROR("Exception");
    }
    br.sendTransform(imu_tf);
    //pub.publish (output_cmd);
  }
/*
  float convert_trans_rot_vel_to_steering_angle(float v, float omega, float wheelbase)
  {
    if(omega == 0 || v == 0) return 0;
    float radius = v / omega;
    return atan(wheelbase / radius) * 180 / pi;
  }
*/
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "imu_node");
  IMU Marin_Imu;
  ros::spin();
}
