//#include "../include/turtlebot3_upfront/turtlebot3_upfront.h"


#include "../include/turtlebot3_upfront/turtlebot3_upfront.h"


Turtlebot3Upfront::Turtlebot3Upfront()
: nh_priv("~")
{
  //Init fake turtlebot node
  bool init_result = init();
  ROS_ASSERT(init_result);
}

Turtlebot3Upfront::~Turtlebot3Upfront()
{
}


bool Turtlebot3Upfront::init()
{
  std::string robot_model = nh.param<std::string>("tb3_model", "");
  wheel_separation = 0.160;
  turning_radius   = 0.080;
  robot_radius     = 0.105;

  nh.param("wheel_left_joint_name", joint_states_name[LEFT],  std::string("wheel_left_joint"));
  nh.param("wheel_right_joint_name", joint_states_name[RIGHT],  std::string("wheel_right_joint"));
  nh.param("joint_states_frame", joint_states.header.frame_id, std::string("base_footprint"));
  nh.param("odom_frame", odom.header.frame_id, std::string("odom"));
  nh.param("base_frame", odom.child_frame_id, std::string("base_footprint"));

  // initialize variables
  wheel_speed_cmd[LEFT]  = 0.0;
  wheel_speed_cmd[RIGHT] = 0.0;
  goal_linear_velocity   = 0.0;
  goal_angular_velocity  = 0.0;
  cmd_vel_timeout        = 1.0;
  last_position[LEFT]    = 0.0;
  last_position[RIGHT]   = 0.0;
  last_velocity[LEFT]    = 0.0;
  last_velocity[RIGHT]   = 0.0;

  double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
  memcpy(&(odom.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom.twist.covariance),pcov,sizeof(double)*36);

  odom_pose[0] = 0.0;
  odom_pose[1] = 0.0;
  odom_pose[2] = 0.0;

  odom_vel[0] = 0.0;
  odom_vel[1] = 0.0;
  odom_vel[2] = 0.0;

  joint_states.name.push_back(joint_states_name[LEFT]);
  joint_states.name.push_back(joint_states_name[RIGHT]);
  joint_states.position.resize(2,0.0);
  joint_states.velocity.resize(2,0.0);
  joint_states.effort.resize(2,0.0);

  // initialize publishers
  joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub         = nh.advertise<nav_msgs::Odometry>("odom", 100);

  // initialize subscribers
  cmd_vel_sub  = nh.subscribe("cmd_vel", 100,  &Turtlebot3Upfront::commandVelocityCallback, this);

  prev_time = ros::Time::now();

  return true;
}

void Turtlebot3Upfront::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time = ros::Time::now();

  goal_linear_velocity  = cmd_vel_msg->linear.x;
  goal_angular_velocity = cmd_vel_msg->angular.z;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * wheel_separation / 2);
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * wheel_separation / 2);
}


bool Turtlebot3Upfront::updateOdometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r;  // rotation of wheels in radians
  double delta_s, delta_th;
  double v[2], w[2];

  wheel_r = wheel_l = 0.0;
  delta_s = delta_th = 0.0;

  v[LEFT] = wheel_speed_cmd[LEFT];
  v[RIGHT] = wheel_speed_cmd[RIGHT];
  w[LEFT] = v[LEFT]/WHEEL_RADIUS; //  w = v/r
  w[RIGHT] = v[RIGHT]/WHEEL_RADIUS;

  last_velocity[LEFT] = w[LEFT];
  last_velocity[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * diff_time.toSec();  // [rad/sec] * [total seconds] = [rad]
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position[LEFT] += wheel_l;
  last_position[RIGHT] += wheel_r;

  delta_s = (wheel_l + wheel_r)/2.0 * WHEEL_RADIUS;
  delta_th = WHEEL_RADIUS * (wheel_r - wheel_l)/ wheel_separation;

  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_th / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_th / 2.0));
  odom_pose[2] += delta_th;

  odom_vel[0] += delta_s / diff_time.toSec(); //  x
  odom_vel[1] += 0.0;
  odom_vel[2] += delta_th / diff_time.toSec();  //  z

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[1];

  return true;
}


void Turtlebot3Upfront::updateJoint(void)
{
  joint_states.position[LEFT] = last_position[LEFT];
  joint_states.position[RIGHT] = last_position[RIGHT];
  joint_states.velocity[LEFT] = last_velocity[LEFT];
  joint_states.velocity[LEFT] = last_velocity[LEFT];
}

void Turtlebot3Upfront::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}


bool Turtlebot3Upfront::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_time;
  prev_time = time_now;

  if((time_now - last_cmd_vel_time).toSec() > cmd_vel_timeout)
  {
    wheel_speed_cmd[LEFT] = 0.0;
    wheel_speed_cmd[RIGHT] = 0.0;
  }

  updateOdometry(step_time);
  odom.header.stamp = time_now;
  odom_pub.publish(odom);

  updateJoint();
  joint_states.header.stamp = time_now;
  joint_states_pub.publish(joint_states);

  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf_broadcaster.sendTransform(odom_tf);

  return true;
}




int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_virtual");
  Turtlebot3Upfront tb3;
  //init();

  ros::Rate loop(30);

  while(ros::ok())
  {
    tb3.update();
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
