#ifndef TURTLEBOT3_UPFRONT_H_
#define TURTLEBOT3_UPFRONT_H_

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//#include turtlebot3_msgs/SensorState.h>
#include "/home/pallav/ros_work/src/turtlebot3_burger/turtlebot3_upfront/include/turtlebot3_upfront/SensorState.h"

#include "turtlebot3_upfront.h"

#define WHEEL_RADIUS                    0.033     // meter

#define LEFT                            0
#define RIGHT                           1

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor


class Turtlebot3Upfront
{
 public:
  Turtlebot3Upfront();
  ~Turtlebot3Upfront();
  bool init();
  bool update();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;

  // ROS Parameters
  // (TODO)

  // ROS Time
  ros::Time last_cmd_vel_time;
  ros::Time prev_time;

  // ROS Topic Publishers
  ros::Publisher joint_states_pub;
  ros::Publisher odom_pub;

  // ROS Topic Subscribers
  ros::Subscriber cmd_vel_sub;

  sensor_msgs::JointState joint_states;
  nav_msgs::Odometry odom;
  tf::TransformBroadcaster tf_broadcaster;

  double wheel_speed_cmd[2];
  double goal_linear_velocity;
  double goal_angular_velocity;
  double cmd_vel_timeout;

  float  odom_pose[3];
  float  odom_vel[3];
  double pose_cov[36];

  std::string joint_states_name[2];

  double last_position[2];
  double last_velocity[2];

  double wheel_separation;
  double turning_radius;
  double robot_radius;

  // Function prototypes
  void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
  bool updateOdometry(ros::Duration diff_time);
  void updateJoint(void);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);
};

#endif // TURTLEBOT3_UPFRONT_H_
