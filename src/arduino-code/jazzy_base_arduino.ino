#include <Wire.h>

/*
 Copyright (c) 2016, Juan Jimeno



 
 Credits to:
 Sungjik https://github.com/sungjik 
 Tony Baltowski https://github.com/tonybaltovski
 
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

/*Changelog: 
 *   Jul 16 2017 -        Addded encoder ros publisher
 *   Jul 23 2017 -        Added jointstate ros publiser
 *   
 *   
*/

//Version
const String firmware_version = "Arduino Firmware Version: 2017-Jul-23-v1.01";

/* Publishing Encoder PID: 
 *  https://github.com/tonybaltovski/ros_arduino/blob/indigo-devel/ros_arduino_base/firmware/two_wheel_base/two_wheel_base.ino
*/

/* TODO: Striving to make this compatible with ros_control
 * http://wiki.ros.org/ros_control#Examples
*/

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//Version of code
#define ARDUINO_CODE_VERSION 

//for dtostr() string handeling
#include <stdlib.h>

//Note: Must export CPATH="./libraries/ros_lib" to use these local libraries"
#include <ros.h>

//header file for publishing "rpm"
#include <geometry_msgs/Vector3Stamped.h>

//header for Encoder messages
#include <ros_arduino_msgs/Encoders.h>

//header for cmdDiff velocity messages
#include <ros_arduino_msgs/CmdDiffVel.h>

//JointState Messages
#include <sensor_msgs/JointState.h>

//header file for cmd_subscribing to "cmd_vel"
//#include "ros_lib/geometry_msgs/Twist.h"
#include <geometry_msgs/Twist.h>

//header file for pid server
#include "linoPID.h"

//header files for imu
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

#include <ros/time.h>

#if defined(WIRE_T3)
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#include "imu/imu_configuration.h"
#include "lino_base_config.h"

//Pick one option below to use interupts or not
#define ENCODER_OPTIMIZE_INTERRUPTS
//#define ENCODER_DO_NOT_USE_INTERRUPTS

#include "Encoder/Encoder.h"

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 10 //hz
#define ENCODER_RATE 10 //hz
#define JOINTSTATE_PUB_RATE 10 //hz

typedef struct
{
  double previous_pid_error; //last measured pid error
  double total_pid_error; //overall pid error of the motor
  long previous_encoder_ticks; //last measured encoder ticks
  double current_rpm; //current speed of the motor
  double required_rpm;//desired speed of the motor 
  int pwm;//desired speed of the motor mapped to PWM
  double radians_per_sec; // speed of the motor in radians per sec = rpm * two_pi/60
}
Motor;

//create a Motor object
Motor left_motor;
Motor right_motor;

//create an Encoder object
Encoder left_encoder(left_encoder_a, left_encoder_b);
Encoder right_encoder(right_encoder_a, right_encoder_b);

//function prototypes
void drive_robot(int, int);
void check_imu();
void publish_imu();
void publish_linear_velocity(unsigned long);
void read_motor_rpm_(Motor * mot, long current_encoder_ticks, unsigned long dt );
void calculate_pwm(Motor * mot);

//callback function prototypes
void command_callback( const geometry_msgs::Twist& cmd_msg);
void pid_callback( const lino_pid::linoPID& pid);

unsigned long lastMilli = 0;       
unsigned long lastMilliPub = 0;
unsigned long previous_command_time = 0;
unsigned long previous_control_time = 0;
unsigned long publish_vel_time = 0;
unsigned long previous_imu_time = 0;
unsigned long previous_encoder_time = 0;
unsigned long previous_jointstate_time = 0;

bool is_first = true;

float Kp = k_p;
float Kd = k_d;
float Ki = k_i;
char buffer[50];

ros::NodeHandle nh;
//ros::NodeHandle nh_private_("~");

// ROS subscriber msgs
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<lino_pid::linoPID> pid_sub("pid", pid_callback);

// ROS imu publishers msgs
ros_arduino_msgs::RawImu raw_imu_msg;
//raw_imu_msg.header.frame_id = "linoimu_link";
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
// ROS velocity publishers msgs
geometry_msgs::Vector3Stamped raw_vel_msg;
//raw_imu_msg.header.frame_id = "linobase_link";
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
// ROS encoder publishers msgs
ros_arduino_msgs::Encoders encoders_msg;
ros::Publisher pub_encoders("encoders", &encoders_msg);

//Joint state publiser
sensor_msgs::JointState lino_joint_state_msg;
ros::Publisher pub_jointstates("wheel_jointstate", &lino_joint_state_msg);
char *lino_joint_state_name[2]= {"left_wheel_joint","right_wheel_joint"};
float lino_joint_state_pos[2] = { 0.0,0.0 }; 
float lino_joint_state_vel[2] = { 0.0,0.0 };
float lino_joint_state_eff[2] = { 0.0,0.0 };
/* JointState details
  name[i]: '<component_id>' of i-th joint in message value arrays.
  position[i]: position of joint i rad
  velocity[i]: velocity of joint i in rad/s
  effort[i]: effort applied in joint i in Nm

  calculate radian position based on endocer ticks:
    Example: https://github.com/uos/kurt_driver/blob/kinetic/kurt_base/src/kurt_base.cc#L162
             http://answers.ros.org/question/185245/sensor_msgsjointstate-on-arduino/

  calculate velocity based on incoder tick and encoder time:
    Example: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom#CA-e6fba4e92077b8226b57e9a62abdb695f9f5a84f_72
*/

//TODO:move this up in the code
float ticks_per_wheel_rotation = encoder_pulse * gear_ratio; //number of encoder tics per wheel rotation //TODO:move this out of loop
//float one_encodertick_in_radians = ( pi * wheel_diameter ) / ticks_per_wheel_rotation;
float one_encodertick_in_radians = two_pi / ticks_per_wheel_rotation;
//radian(r) is the radius of a circle.   2 pi radians(r) = 360 deg
    //calculate 1 tick=how many radians
    //one_encodertick_in_radians = 2 pi r / ticks_per_wheel_rotation //or pi diameter /ticks_per_wheel_rotation



void setup()
{
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_direction, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_direction, OUTPUT);
  drive_robot(0,0);

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);

  encoders_msg.header.frame_id = "lino_wheelencoders";
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);
  nh.advertise(pub_encoders);

  //Joint States
  lino_joint_state_msg.header.stamp = nh.now();
  lino_joint_state_msg.name_length = 2; //2 wheel joints for all of these
  lino_joint_state_msg.position_length = 2;
  lino_joint_state_msg.velocity_length = 2;
  lino_joint_state_msg.effort_length =  2;
  lino_joint_state_msg.header.frame_id = "test_frameid";
  lino_joint_state_msg.name = lino_joint_state_name;
  //Uncomment these
  lino_joint_state_msg.velocity = lino_joint_state_pos; //array of float 
  lino_joint_state_msg.position = lino_joint_state_vel; //array of float
  lino_joint_state_msg.effort =  lino_joint_state_eff;  //array of float
  nh.advertise(pub_jointstates);
  
  
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("Connected to microcontroller...");
  nh.loginfo("ROS Arduino IMU started.");
  nh.loginfo(firmware_version.c_str());
  char s_float[20];  //convert float to string, as sprintf %f doesnt work
  dtostrf(ticks_per_wheel_rotation, 10, 4, s_float);
      /*
        where
        floatvar   float variable
        StringLengthIncDecimalPoint   This is the length of the string that will be created
        numVarsAfterDecimal   The number of digits after the deimal point to print
        charbuf   the array to store the results
      */
  sprintf (buffer, "Encoder ticks_per_wheel_rotation:%s",s_float);
  nh.loginfo(buffer);
  
  dtostrf(one_encodertick_in_radians, 10, 4, s_float );
  sprintf (buffer, "one_encodertick_in_radians: %s", s_float);
  nh.loginfo(buffer);
  
#if defined(WIRE_T3)
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
  Wire.begin();
#endif
  delay(5);
}

void loop()
{
  //this block publishes velocity based on defined rate
  if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
  {
    unsigned long current_time = millis();
    publish_linear_velocity(current_time - publish_vel_time);
    publish_vel_time = millis();
  }

  //this block drives the robot based on defined rate
  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE))
  {
    unsigned long current_time = millis();
    unsigned long dt = current_time - previous_control_time;
    //calculate motor's current speed
    read_motor_rpm_(&left_motor, left_encoder.read(), dt);
    read_motor_rpm_(&right_motor, right_encoder.read(), dt);
    //calculate how much PWM is needed based on required RPM
    calculate_pwm(&left_motor);
    calculate_pwm(&right_motor);    
    //move the wheels based on the PWM calculated
    drive_robot(left_motor.pwm, right_motor.pwm);
    previous_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - previous_command_time) >= 400)
  {
    left_motor.required_rpm = 0;
    right_motor.required_rpm = 0;
    right_motor.pwm = 0;
    left_motor.pwm = 0;
    drive_robot(left_motor.pwm, right_motor.pwm);
  }


  //this block publishes the IMU data based on defined rate
  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU exits
    if (is_first)
    {
    check_imu();
    }
    else
    {
    //publish the IMU data
    publish_imu();
    }
    previous_imu_time = millis();
  }
  
  //this block publishes the encoder readings. change DEBUG to 0 if you don't want to display
  if ((millis() - previous_encoder_time) >= (1000 / ENCODER_RATE))
  {

      if(DEBUG)
      {
      sprintf (buffer, "Encoded Right: %d", right_encoder.read());
      nh.logdebug(buffer);
      sprintf (buffer, "Encoder Left: %d", left_encoder.read());
      nh.loginfo(buffer);
      //TODO: change above to ns.logdebug - http://wiki.ros.org/rosserial/Overview/Logging
      }
      encoders_msg.left = left_encoder.read();
      encoders_msg.right = right_encoder.read();
      encoders_msg.header.stamp = nh.now();
      pub_encoders.publish(&encoders_msg);    
      previous_encoder_time = millis();
  }

  //this block publishes the joint_state position and angular velocity_length
  if ((millis() - previous_jointstate_time) >= (1000 / JOINTSTATE_PUB_RATE ))
  {
      long left_motor_total_tics=left_encoder.read(); //long
      long right_motor_total_tics=right_encoder.read(); 
      //calculate radians and publish to lino_joint_state.position
      //ticks wrap around, so only get the remainder of the ticks
      //float remainder_left_ticks =left_motor_total_tics-(left_motor_total_tics/ticks_per_wheel_rotation)*ticks_per_wheel_rotation;
      int remainder_left_ticks  =int(left_motor_total_tics)-int((left_motor_total_tics)/ticks_per_wheel_rotation)*ticks_per_wheel_rotation;
      int remainder_right_ticks =int(right_motor_total_tics)-int((right_motor_total_tics)/ticks_per_wheel_rotation)*ticks_per_wheel_rotation;
      if (DEBUG) {
        sprintf (buffer, "remainder_left_ticks: %d", remainder_left_ticks);
        nh.loginfo(buffer);
        sprintf (buffer, "remainder_right_ticks: %d", remainder_right_ticks);
        nh.loginfo(buffer);
      }
      lino_joint_state_msg.position[0]=one_encodertick_in_radians * remainder_left_ticks; //joint state position in radians 
      lino_joint_state_msg.position[1]=one_encodertick_in_radians * remainder_right_ticks; //joint state position in radians
      lino_joint_state_msg.velocity[0]=left_motor.radians_per_sec; //TODO: this is calculated when left_motor.read() is called, timming may be off
      lino_joint_state_msg.velocity[1]=right_motor.radians_per_sec; //TODO: this is calculated when left_motor.read() is called, timming may be off
      
      //TODO: calculate velocity in radian/sec & publish to lino_joint_state.velocity
      
    
      //No CANDO: calualate effort and to publish to lino_joint_state.effort


      //Publis Joint States
      lino_joint_state_msg.header.stamp = nh.now();
      pub_jointstates.publish(&lino_joint_state_msg);
      pub_jointstates.publish(&lino_joint_state_msg);

      
      previous_jointstate_time = millis();
  }
  
  //call all the callbacks waiting to be called
  nh.spinOnce();
}

void pid_callback( const lino_pid::linoPID& pid) 
{
  //callback function every time PID constants are received from lino_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  Kp = pid.p;
  Kd = pid.d;
  Ki = pid.i;
  sprintf (buffer, "P: %f D: %f D: %f", pid.p, pid.d, pid.i);
  nh.loginfo(buffer);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored

  previous_command_time = millis();
  double linear_vel = cmd_msg.linear.x;
  double angular_vel = cmd_msg.angular.z;
  //convert m/s to m/min
  double linear_vel_mins = linear_vel * 60;
  //convert rad/s to rad/min
  double angular_vel_mins = angular_vel * 60;
  //calculate the wheel's circumference
  double circumference = pi * wheel_diameter;
  //calculate the tangential velocity of the wheel if the robot's rotating where Vt = ω * radius
  double tangential_vel = angular_vel_mins * (track_width / 2);

  //calculate and assign desired RPM for each motor
  left_motor.required_rpm = (linear_vel_mins / circumference) - (tangential_vel / circumference);
  right_motor.required_rpm = (linear_vel_mins / circumference) + (tangential_vel / circumference);

}

void drive_robot( int command_left, int command_right)
{
  //this functions spins the left and right wheel based on a defined speed in PWM  
  //change left motor direction
  //forward
  if (command_left >= 0)
  {
    digitalWrite(left_motor_direction, HIGH);
  }
  //reverse
  else
  {
    digitalWrite(left_motor_direction, LOW);
  }
  //spin the motor
  analogWrite(left_motor_pwm, abs(command_left));
  
  //change right motor direction
  //forward
  if (command_right >= 0)
  {
    digitalWrite(right_motor_direction, HIGH);
  }
  //reverse
  else
  {
    digitalWrite(right_motor_direction, LOW);
  }
  //spin the motor
  analogWrite(right_motor_pwm, abs(command_right));
}

void read_motor_rpm_(Motor * mot, long current_encoder_ticks, unsigned long dt )
{
  // this function calculates the motor's RPM based on encoder ticks and delta time
  
  //convert the time from milliseconds to minutes
  dt = 60000 / dt;
  //calculate change in number of ticks from the encoder
  double delta_ticks = current_encoder_ticks - mot->previous_encoder_ticks;
  //calculate wheel's speed (RPM)
  mot->current_rpm = (delta_ticks / double(encoder_pulse * gear_ratio)) * dt;
  mot->radians_per_sec = (mot->current_rpm * two_pi)/60;
  mot->previous_encoder_ticks = current_encoder_ticks;
}

void publish_linear_velocity(unsigned long time)
{
  // this function publishes the linear speed of the robot
  
  //calculate the average RPM 
  double average_rpm = (left_motor.current_rpm + right_motor.current_rpm) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  double average_rps = average_rpm / 60; // RPS
  //calculate linear speed
  double linear_velocity = (average_rps * (wheel_diameter * pi)); // m/s 
  
  //fill in the object 
  raw_vel_msg.header.stamp = nh.now();
  raw_vel_msg.vector.x = linear_velocity;
  raw_vel_msg.vector.y = 0.00;
  raw_vel_msg.vector.z = double(time) / 1000;
  //publish raw_vel_msg object to ROS
  raw_vel_pub.publish(&raw_vel_msg); 
  nh.spinOnce();
}

void calculate_pwm(Motor * mot)
{
  // this function takes a Motor object argument,
  // implements a PID controller and calculates the PWM required to drive the motor
  double pid;
  double new_rpm;
  double error;
  
  //calculate the error ()
  error = mot->required_rpm - mot->current_rpm;
  //calculate the overall error
  mot->total_pid_error += error;
  //PID controller
  pid = Kp * error  + Ki * mot->total_pid_error + Kd * (error - mot->previous_pid_error);
  mot->previous_pid_error = error;
  //adds the calculated PID value to the required rpm for error compensation
  new_rpm = constrain(double(mot->pwm) * max_rpm / 255 + pid, -max_rpm, max_rpm);
  //maps rpm to PWM signal, where 255 is the max possible value from an 8 bit controller
  mot->pwm = (new_rpm / max_rpm) * 255;
}


void check_imu()
{
  //this function checks if IMU is present
  raw_imu_msg.accelerometer = check_accelerometer();
  raw_imu_msg.gyroscope = check_gyroscope();
  raw_imu_msg.magnetometer = check_magnetometer();

  if (!raw_imu_msg.accelerometer)
  {
    nh.logerror("Accelerometer NOT FOUND!");
  }

  if (!raw_imu_msg.gyroscope)
  {
    nh.logerror("Gyroscope NOT FOUND!");
  }

  if (!raw_imu_msg.magnetometer)
  {
    nh.logerror("Magnetometer NOT FOUND!");
  }
  
  is_first = false;
}

void publish_imu()
{
  //this function publishes raw IMU reading
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";
  //measure accelerometer
  if (raw_imu_msg.accelerometer)
  {
    measure_acceleration();
    raw_imu_msg.raw_linear_acceleration = raw_acceleration;
  }

  //measure gyroscope
  if (raw_imu_msg.gyroscope)
  {
    measure_gyroscope();
    raw_imu_msg.raw_angular_velocity = raw_rotation;
  }
  
  //measure magnetometer
  if (raw_imu_msg.magnetometer)
  {
    measure_magnetometer();
    raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
  }
  //publish raw_imu_msg object to ROS
  raw_imu_pub.publish(&raw_imu_msg);
}

