/* Goal of this code:
 * 1. Read Encoder tics and:
 *    a) update l/r rmp/velocity Motor Stricts
 *    b) Calculate PID from wheel encoders, update motor struct
 * 2. Publish odom
 * 3. Publish lwheel/rwheel Joint States (dup #1)
 *    
 *  
 * 4. set up cmd_vel listener
 *    a) calculate needed raw PWM signals to send to:
 *       a1) set pwm in motor struct
 *    b) send PWM to Arduino raw pwm listener
 * 
 * Arduino code update:  
 *   change to only publish
 *     1) imu_raw
 *     2) encoder_msgs
 *     3) raw_pwm Subscriber listener
 * 
 * 
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

//covariance matrix https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/diff_drive_controller/src/diff_drive_controller.cpp
#include <boost/assign.hpp>
#include <boost/array.hpp>

//encoder Messagges
#include <ros_arduino_msgs/Encoders.h>

#define DEBUG 0

#define pi 3.1415926 
#define two_pi 6.2831853

//TODO: these should be parameters
#define k_p 0.4 // P constant
#define k_i 0.0 // I constant
#define k_d 1.0 // D constant

//define your motors' specs here
const double max_rpm=1241; //motor's maximum RPM(1241)?? or Wheel Max RPM(70ish)? How to set?? Was 330
const double encoder_pulse=8;               // encoder's number of ticks per revolution 
const double gear_ratio=17.737;             // motor's gear ratio

//define your robot base's specs here
const double wheel_diameter=0.2286;         // wheel's diameter in meters 9"
const double wheel_width=0.0762;            // wheel's width in meters 3"
const double track_width=0.5715;            // width of the plate you are using - 22.5"?? between tires

// calculate number of encoder tics per wheel rotation
const double ticks_per_wheel_rotation = encoder_pulse * gear_ratio;                                 
//calculate 1 tick=how many radians
const double one_encodertick_in_radians = two_pi / ticks_per_wheel_rotation;
  //radian(r) is the radius of a circle.   2 pi r in radians = 360 deg
  
const char *prog_name="jbot_base2_node";
const char *prog_ver="2.0.0";

typedef struct
{
  double previous_pid_error; //last measured pid error
  double total_pid_error; //overall pid error of the motor
  long previous_encoder_ticks; //last measured encoder ticks
  double encoder_lasttime;      //last timestamp encoder was read
  double current_rpm; //current speed of the motor
  double required_rpm;//desired speed of the motor 
  int pwm;//desired speed of the motor mapped to PWM
  double radians_per_sec; // speed of the motor in radians per sec = rpm * two_pi/60
  double linear_vel; //distance traveled in meters
}
Motor;
//create a Motor object
Motor left_motor;
Motor right_motor;

//ros launch params definitions
std::string baselink_frame; //TF base frame for robot usually:baselink_frame or base_link
std::string odom_frame; // to publish
std::string vel_topic;  //to publish
std::string encoder_topic; //to subscribe to
int odom_publish_rate = 50; // pub rate for odom
std::string imu_topic;  //to subscribe to
//TODO: joint states

//odom pos values
double x = 0.0;
double y = 0.0;
double th = 0.0;
//odom velocities
double vx = 0;
double vy = 0;
double vth = 0.0;

//prototypes


void init_motor(Motor * mot)
{
  mot->previous_pid_error=0; 
  mot->total_pid_error=0;  
  mot->previous_encoder_ticks=0;  //TODO: should read this once from the encoder header
  mot->encoder_lasttime=ros::Time::now().toSec(); //TODO: should read this once from the encoder header
  mot->current_rpm=0;
  mot->required_rpm=0;
  mot->pwm=0;
  mot->radians_per_sec=0;
  mot->linear_vel=0;
}

void setup()
{
  //init motors
  init_motor(&left_motor);
  init_motor(&right_motor);  

  char buffer[80]; //string buffer for info logging
  
  //Print info
  sprintf (buffer, "%s::Version: %s",prog_name,prog_ver);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Subscribing to encoder topic: %s",prog_name,encoder_topic.c_str());
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Max motor rpm(max_rpm): %.4f",prog_name,max_rpm);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Ticks per motor revolution(encoder_pulse): %.4f",prog_name,encoder_pulse);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Ticks per Wheel revolution: %.4f",prog_name,ticks_per_wheel_rotation);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Motor gear Ratio(gear_ratio): %.4f",prog_name,gear_ratio);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Wheel Diameter(wheel_diameter): %.4f",prog_name,wheel_diameter);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Wheel Width(wheel_width) : %.4f",prog_name,wheel_width);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::Track Width(track_width): %.4f",prog_name,track_width);
  ROS_INFO_STREAM(buffer);
  
  //Debugging
  if ( DEBUG) 
  {
    double one_rpm_in_tics_per_sec=ticks_per_wheel_rotation/60;
    sprintf (buffer, "%s::*One rpm in ticks/second: %.4f",prog_name,one_rpm_in_tics_per_sec);
    ROS_INFO_STREAM(buffer);
    sprintf (buffer, "%s::*10 rpms in ticks/second : %.4f",prog_name,one_rpm_in_tics_per_sec*10);
    ROS_INFO_STREAM(buffer);

    //sprintf (buffer, "%s:: : %s",prog_name,);
    //ROS_INFO_STREAM(buffer);  
  }
}


void calculate_motor_rpm_and_radian(Motor * mot, long current_encoder_ticks, double current_time )
{
  // this function calculates the motor's RPM based on encoder ticks and delta time
  char buffer[80]; //string buffer for info logging
  double delta_time_motor=current_time - mot->encoder_lasttime;
  mot->encoder_lasttime=current_time;
 
  //calculate change in number of ticks from the encoder
  double delta_ticks = current_encoder_ticks - mot->previous_encoder_ticks;
  double delta_ticks_per_sec=(delta_ticks / delta_time_motor);
  double delta_ticks_per_min=delta_ticks_per_sec*60;
  
  //calculate wheel's speed (RPM)
  mot->current_rpm = (delta_ticks_per_min/ticks_per_wheel_rotation);
  mot->radians_per_sec = (mot->current_rpm * two_pi)/60;
  mot->linear_vel= ((mot->current_rpm /60) * (wheel_diameter * pi)); //convert rpms to sec
  mot->previous_encoder_ticks = current_encoder_ticks;
  
  //debug code:
  if (delta_ticks != 0.0) 
  {
    if ( DEBUG) 
    {
      ROS_INFO_STREAM("Motor Changed");
      sprintf (buffer, "  ***dt_seconds: %15.8f",delta_time_motor);
      //#sprintf (buffer, "  ***dt_seconds: %15.8f",dt_seconds);
      ROS_INFO_STREAM(buffer); 
      
      //sprintf (buffer, "  ***dt_minutes: %15.8f",dt_minutes);
      //ROS_INFO_STREAM(buffer); 
      
      sprintf (buffer, "  *delta_ticks: %15.4f",delta_ticks);
      ROS_INFO_STREAM(buffer);
      
      sprintf (buffer, "  *delta_ticks_per_sec: %15.4f",delta_ticks_per_sec);
      ROS_INFO_STREAM(buffer);
      /* 
      * double rpms= (delta_ticks / double(encoder_pulse * gear_ratio)) * dt_minutes;
      * sprintf (buffer, "  *rpms: %15.4f",rpms);
      * ROS_INFO_STREAM(buffer);
      * 
      */ 
      //double rpms= (delta_ticks / double(encoder_pulse * gear_ratio)) * dt_minutes;
      sprintf (buffer, "  *rpms: %15.4f",mot->current_rpm);
      ROS_INFO_STREAM(buffer);
      
      sprintf (buffer, "  *radians_per_sec: %15.4f",mot->radians_per_sec);
      ROS_INFO_STREAM(buffer);
      
      sprintf (buffer, "  *linear_vel: %15.4f",mot->linear_vel);
      ROS_INFO_STREAM(buffer);
    }


  }
 
}



void encoderCallback( const ros_arduino_msgs::Encoders& encoder_msg) {
  //callback every time the robot's wheel encoder message received
  
  //ros::Time current_time = ros::Time::now(); //alternately use encoder_msg.header.stamp
  ros::Time current_time = encoder_msg.header.stamp;
  //read the encoder tics
  long left_encoder_ticks_current= encoder_msg.left;
  long right_encoder_ticks_current = encoder_msg.right;
  
  calculate_motor_rpm_and_radian(&left_motor, left_encoder_ticks_current, current_time.toSec());
  calculate_motor_rpm_and_radian(&right_motor, right_encoder_ticks_current, current_time.toSec());
    
}


ros::Time g_last_loop_time(0.0); //time stamp for main loop

int main(int argc, char** argv){
  ros::init(argc, argv, "jbot_base_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");
  tf::TransformBroadcaster odom_broadcaster;
  char buffer[80]; //string buffer for info logging

  
  //set ros launch params
  nh_private_.param<std::string>("baselink_frame", baselink_frame, "base_link2");
  nh_private_.param<std::string>("odom_frame", odom_frame, "jbot_wheelodom2");
  nh_private_.param<std::string>("encoder_topic",encoder_topic,"encoders");
  nh_private_.param("odom_publish_rate", odom_publish_rate, 50); //odom pubish rate
  nh_private_.param<std::string>("imu_topic", imu_topic, "imu/data");
  //nh_private_.param<std::string>("vel_topic", vel_topic, "raw_vel");
  
  
  
  setup();
  
  //ROS Subscribers 
  ros::Subscriber encoder_sub = nh.subscribe(encoder_topic, 50, encoderCallback);
  //ros::Subscriber imu_sub = n.subscribe(imu_topic, 50, IMUCallback);
  
  //Ros publishers
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_frame, odom_publish_rate);

  double rate = odom_publish_rate; //launch parameter: odom_publish_rate
  
  
  ros::Rate r(rate); 
  while(nh.ok()){
    //see: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
    ros::spinOnce();
    ros::Time main_current_time = ros::Time::now(); 
    
    
    /*************************/
    //compute odometry in a typical way given the velocities of the robot
    double dt = (main_current_time - g_last_loop_time).toSec();
    
    vx = (left_motor.linear_vel + right_motor.linear_vel)/2; //Ave vel in x direction
    vy = 0; //wheels  cant roll  in y direction
    vth = (right_motor.linear_vel - left_motor.linear_vel)/(track_width+wheel_width);  //subject to lots of wheel slippage
    
    double delta_x = (vx* cos(th) - vy * sin(th)) * dt;  
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;  
    double delta_th = vth * dt; 
    
    x += delta_x;
    y += delta_y;
    th += delta_th;  //theta is yay, rotation around z axix
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = main_current_time;
    odom_trans.header.frame_id = odom_frame;
    odom_trans.child_frame_id = baselink_frame;
    
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = main_current_time;
    odom.header.frame_id = odom_frame;
    
    //set the pose position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;    
    //Pose Covariance matrix
    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.01;
    odom.pose.covariance[14] = 99999;
    odom.pose.covariance[21] = 99999;
    odom.pose.covariance[28] = 99999;
    odom.pose.covariance[35] = 0.01;
    
    //set the velocity twist
    odom.child_frame_id = baselink_frame;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;   
    //Twist Covariance matrix
    odom.twist.covariance[0]  = 0.001;
    odom.twist.covariance[7]  = 0.001;
    odom.twist.covariance[14] = 99999;
    odom.twist.covariance[21] = 99999;
    odom.twist.covariance[28] = 99999;
    odom.twist.covariance[35] = 0.001;
    
    
    // covariance matrix revferances here
    // See https://answers.ros.org/question/51007/covariance-matrix/
    // and https://github.com/chicagoedt/revo_robot/commit/620f3f61ea8ac832e2040fb4f4e5583a15e23e29
    // and https://answers.ros.org/question/12808/how-to-solve-timestamps-of-odometry-and-imu-are-x-seconds-apart/\
    // and https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/diff_drive_controller/src/diff_drive_controller.cpp
    
    
    //publish the message
    odom_pub.publish(odom);
        
    g_last_loop_time = main_current_time;
    r.sleep();
  }
}