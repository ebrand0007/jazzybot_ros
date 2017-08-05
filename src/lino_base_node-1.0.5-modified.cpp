/*
 Copyright (c) 2016, Juan Jimeno
 Source: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of  nor the names of its contributors may be used to
 endorse or promote products derived from this software without specific
 prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

double g_vel_x = 0.0;
double g_vel_y = 0.0;

double g_vel_dt = 0.0;
double g_imu_dt = 0.0;
double g_imu_z = 0.0;

ros::Time g_last_loop_time(0.0);
ros::Time g_last_vel_time(0.0);
ros::Time g_last_imu_time(0.0);


void vel_callback( const geometry_msgs::Vector3Stamped& vel) {
  //callback every time the robot's linear velocity is received
  ros::Time current_time = ros::Time::now();
  g_vel_x = vel.vector.x;
  g_vel_dt = vel.vector.z;  //TODO: believe dt is incorrect!!!
  
  g_last_vel_time = vel.header.stamp;
}

void imu_callback( const sensor_msgs::Imu& imu){
  //callback every time the robot's angular velocity is received
  ros::Time current_time = ros::Time::now();
  //this block is to filter out imu noise
  //if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
  if(imu.angular_velocity.z > -0.005 && imu.angular_velocity.z < 0)
  {
    g_imu_z = 0.00;
  }
  else
  {
    g_imu_z = imu.angular_velocity.z;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  tf::TransformBroadcaster odom_broadcaster;
  
  //lauch params
  std::string baselink_frame;
  std::string odom_frame;
  std::string imu_topic;
  std::string vel_topic;

  nh_private_.param<std::string>("baselink_frame", baselink_frame, "base_link");
  nh_private_.param<std::string>("odom_frame", odom_frame, "jbot_wheelodom");
  nh_private_.param<std::string>("imu_topic", imu_topic, "imu/data");
  nh_private_.param<std::string>("vel_topic", vel_topic, "raw_vel");  
 
  //ros::Subscriber sub = n.subscribe("raw_vel", 50, vel_callback);
  //ros::Subscriber imu_sub = n.subscribe("imu/data", 50, imu_callback);
  //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe(vel_topic, 50, velCallback);
  ros::Subscriber imu_sub = n.subscribe(imu_topic, 50, IMUCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_frame, 50);

  double rate = 10.0; //TODO: this should be a parameter
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    ros::Time current_time = ros::Time::now();

    
    //calculate change in time (dt)
    double dtt = (current_time - g_last_loop_time).toSec();

    //linear velocity is the linear velocity published from the Teensy board(raw_vel)
    double linear_velocity = g_vel_x;
    //angular velocity is the rotation in Z from imu_filter_madgwick's output
    double angular_velocity = g_imu_z;

    //calculate angular displacement  θ = ω * t
    double delta_theta = angular_velocity * dtt; //radians
    //calculate linear displacement in X axis
    double delta_x = (linear_velocity * cos(theta)) * g_last_vel_time; //m
    //calculate linear displacement in Y axis
    double delta_y = (linear_velocity * sin(theta)) * g_last_vel_time; //m

    //calculate current position of the robot
    //where (x,y) is summation of linear and angular displacement
    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(odom_trans);


    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from IMU
    odom.twist.twist.angular.z = g_imu_z;

    odom_pub.publish(odom);

    g_last_loop_time = current_time;
    r.sleep();
  }
}
