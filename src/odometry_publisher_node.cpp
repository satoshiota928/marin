#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <marin_controller/Robot_state.h>
#include <tf/transform_broadcaster.h>

const float pi = 3.14159265359;

class ODOMPub
{
private:
  ros::NodeHandle private_nh;
  geometry_msgs::TransformStamped output_tf_msg;
  geometry_msgs::Quaternion yaw_quat_msg;
  nav_msgs::Odometry output_odom_msg;
  ros::Subscriber sub;
  ros::Publisher pub;
  tf::TransformBroadcaster br;
  ros::Time current_time, last_time;

  int enc_ppr;
  float enc_dia;
  float ctrl_freq;

  float dt;

  double x; //unit: m
  double y; //unit: m
  double theta; //unit: rad

  double vr;  //unit: m/s
  double vx;  //unit: m/s
  double vy;  //unit: m/s
  double omega; //unit: rad/s

public:
  ODOMPub()
  {
    // **** get paramters
    if (!private_nh.getParam ("encoder_ppr", enc_ppr))
      enc_ppr = 1440;
    if (!private_nh.getParam ("encoder_diameter", enc_dia))
      enc_dia = 0.05;
    if (!private_nh.getParam ("control_frequency", ctrl_freq))
      ctrl_freq = 50;

    x = 0.0;
    y = 0.0;
    theta = 0.0;

    vr = 0.0;
    vx = 0.0;
    vy = 0.0;
    omega = 0.0;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    sub = private_nh.subscribe ("/robot_state", 1, &ODOMPub::OdomPubCB, this);
    pub = private_nh.advertise<nav_msgs::Odometry> ("/odom", 0);
  }

  void OdomPubCB (const marin_controller::Robot_state& msg)
  {
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    omega = msg.angular_velocity.z;

    computeVelosity(msg.enc_count);
    computeOdom();

    yaw_quat_msg = tf::createQuaternionMsgFromYaw(theta);

    //odom
    output_odom_msg.header.stamp = current_time;
    output_odom_msg.header.frame_id = "/odom";
    output_odom_msg.child_frame_id = "/base_footprint";

    output_odom_msg.pose.pose.position.x = x;
    output_odom_msg.pose.pose.position.y = y;
    output_odom_msg.pose.pose.position.z = 0.0;
    output_odom_msg.pose.pose.orientation = yaw_quat_msg;

    output_odom_msg.twist.twist.linear.x = vx;
    output_odom_msg.twist.twist.linear.y = vy;
    output_odom_msg.twist.twist.angular.z = omega;

    pub.publish (output_odom_msg);  //publish odom

    //tf
    output_tf_msg.header.stamp = current_time;
    output_tf_msg.header.frame_id = "/odom";
    output_tf_msg.child_frame_id = "/base_footprint";

    output_tf_msg.transform.translation.x = x;
    output_tf_msg.transform.translation.y = y;
    output_tf_msg.transform.translation.z = 0.0;
    output_tf_msg.transform.rotation = yaw_quat_msg;

    br.sendTransform(output_tf_msg);  //broadcast tf

    last_time = current_time;
  }

  void computeVelosity(int pulse_count)
  {
    vr = pulse_count * ctrl_freq / enc_ppr * enc_dia * pi;

    vx = vr * cos(theta);
    vy = vr * sin(theta);
  }

  void computeOdom()
  {
    double delta_theta;
    double delta_x;
    double delta_y;

    if(omega == 0)
    {
      delta_theta = 0;
      delta_x = vx * dt;
      delta_y = vy * dt;
    }
    else
    {
      double radius = vx / omega;
      delta_theta = omega * dt;
      delta_x = radius * (sin(theta + delta_theta)-sin(theta));
      delta_y = radius * (cos(theta) - cos(theta + delta_theta));
    }

    x += delta_x;
    y += delta_y;
    theta += delta_theta;
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "odometry_publisher_node");
  ODOMPub OdomPublisher;
  ros::spin();
}
