
/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */
#include <ros.h>
#include "turtlebot3.h"
#include <std_msgs/String.h>


ros::NodeHandle nh;

void callback(const std_msgs::String& data)
{
  String key = data.data;
  if (key=="w")
  {
    forward(); 
  }

  else if (key=="x")
  {
    backward();
  }

  else if (key=="d")
  {
    right();
  }

  else if (key=="a")
  {
    left();
  }

  else if (key=="s")
  {
    Stop();
  }
}

ros::Subscriber<std_msgs::String> sub("teleop_key", &callback);


Turtlebot3MotorDriver motor_driver;
//float goal_velocity[2];
/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  motor_driver.init(NAME);
}

void loop()
{
  nh.spinOnce();
  delay(10);

  }



void forward(){
  float goal_velocity[2] = {0.5,0};
  motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
}

void backward(){
  float goal_velocity[2] = {-0.5,0};
  motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
}

void right(){
  float goal_velocity[2] = {0,-2};
  motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
}

void left(){
  float goal_velocity[2] = {0,2};
  motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
}

void Stop(){
  float goal_velocity[2] = {0,0};
  motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
}
