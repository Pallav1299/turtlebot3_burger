#include "turtlebot3.h"

#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
//#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

ros::Time rosNow(void);
ros::Time current_time, last_time;


//ros::init("odometry_publisher");
ros::NodeHandle n;

nav_msgs::Odometry odom;

ros::Publisher odom_pub("odom", &odom);
//ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

tf::TransformBroadcaster odom_broadcaster;



double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

//void callback2(const ros::TimerEvent&);

void setup()
{
  n.initNode();
  n.advertise(odom_pub);
  //tf_broadcaster.init(n);

  //ros::Rate r(1); //1Hz

  current_time = rosNow();
  last_time = rosNow();
}

void loop()
{
  n.spinOnce();               // check for incoming messages
  current_time = rosNow();
  double recent_time = current_time.toSec();
  double earlier_time = last_time.toSec();
  //compute odometry in a typical way given the velocities of the robot
  double dt = (recent_time - earlier_time);
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  geometry_msgs::Quaternion odom_quat;
  odom_quat.x = 0;
  odom_quat.y = 0;
  odom_quat.z = sin(th * 0.5);
  odom_quat.w = cos(th * 0.5);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(&odom);

  last_time = current_time;
  //ros::Timer timer = n.createTimer(ros::Duration(1.0, timerCallback);
  rate(1);
}
/*
  void timerCallback(const ros::TimerEvent&)
  {
  ROS_INFO("Callback 2 triggered");
  }
*/

/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return n.now();
}

void rate(unsigned long frequency)
{
  unsigned long start = rosNow().toSec();
  unsigned long expected_cycle_time = 1.0 / frequency;
  unsigned long actual_cycle_time = 0.0;

  unsigned long expected_end = start + expected_cycle_time;
  unsigned long actual_end = rosNow().toSec();

  if (actual_end < start) {
    expected_end = actual_end + expected_cycle_time;
  }

  unsigned long sleep_time = expected_end - actual_end;

  actual_cycle_time = actual_end - start;

  start = expected_end;

  if (sleep_time <= 0.0)
  {
    if (actual_end > expected_end + expected_cycle_time)
    {
      start = actual_end;
    }
    //return true;
  }
  //unsigned long track = millis();
  while((millis()<sleep_time*1000)){
    }
   //delay(sleep_time *1000);
}
