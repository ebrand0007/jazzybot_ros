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

//encoder Messagges
#include <ros_arduino_msgs/Encoders.h>


#define pi 3.1415926 
#define two_pi 6.2831853

//TODO: these should be parameters
#define k_p 0.4 // P constant
#define k_i 0.0 // I constant
#define k_d 1.0 // D constant
//define your motors' specs here
const double max_rpm=330; //motor's maximum RPM
const double encoder_pulse=8;               //encoder's number of ticks per revolution 
const double gear_ratio=17.737;             //motor's gear ratio
//ticsPerWheelRotation=141.9ish;            //ticsPerMotorRev*gearRatio;
//define your robot base's specs here
const double wheel_diameter=0.2286;         //wheel's diameter in meters 9"
const double wheel_width=0.0762;            //wheel's width in meters 3"
const double track_width=0.5715;     // width of the plate you are using - 22.5"?? between tires
                                            //number of encoder tics per wheel rotation 
const double ticks_per_wheel_rotation = encoder_pulse * gear_ratio; 



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
}
Motor;
//create a Motor object
Motor left_motor;
Motor right_motor;

//ros launch params definitions
std::string baselink_frame;
std::string odom_frame;
std::string encoder_topic;
//std::string imu_topic;
//std::string vel_topic;

//time stamps for message reads/writes
double g_encoder_dt;
//ros::Time g_encoder_lasttime(0.0);
//ros::Time g_last_loop_time(0.0); //time stamp for main loop

//prototypes


void init_motor(Motor * mot)
{
  mot->previous_pid_error=0; 
  mot->total_pid_error=0;  
  mot->previous_encoder_ticks=0;
  mot->current_rpm=0;
  mot->required_rpm=0;
  mot->pwm=0;
  mot->radians_per_sec=0;
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
  double one_rpm_in_tics_per_sec=ticks_per_wheel_rotation/60;
  sprintf (buffer, "%s::*One rpm in ticks/second: %.4f",prog_name,one_rpm_in_tics_per_sec);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "%s::*10 rpms in ticks/second : %.4f",prog_name,one_rpm_in_tics_per_sec*10);
  ROS_INFO_STREAM(buffer);

  //sprintf (buffer, "%s:: : %s",prog_name,);
  //ROS_INFO_STREAM(buffer);  
}


void calculate_motor_rpm_and_radian(Motor * mot, long current_encoder_ticks, double dt_seconds )
{
  // this function calculates the motor's RPM based on encoder ticks and delta time
  char buffer[80]; //string buffer for info logging
  
  //convert the time from ROS::time::now() seconds to minutes
  double dt_minutes = dt_seconds * 3500; //TODO aint right shold be 60
  //double dt_minutes = dt_seconds * 6000; //rpm=110 shold be 60
  //double dt_minutes = dt_seconds * 10000; //goiung the wrong way rpm=182 shold be 60
 
  //calculate change in number of ticks from the encoder
  double delta_ticks = current_encoder_ticks - mot->previous_encoder_ticks;
  double delta_ticks_per_sec=(delta_ticks * delta_ticks)/60;
  
  //calculate wheel's speed (RPM)
  mot->current_rpm = (delta_ticks / double(encoder_pulse * gear_ratio)) * dt_minutes;
  //mot->current_rpm = double((delta_ticks*60*1000) / double(encoder_pulse * gear_ratio) * dt_seconds);
  mot->radians_per_sec = (mot->current_rpm * two_pi)/60;
  mot->previous_encoder_ticks = current_encoder_ticks;
  
  //debug code:
  if (delta_ticks != 0.0) 
  {
    ROS_INFO_STREAM("Motor Changed");
    sprintf (buffer, "  ***dt_seconds: %15.8f",dt_seconds);
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

  }
 
}



void encoderCallback( const ros_arduino_msgs::Encoders& encoder_msg) {
  //callback every time the robot's wheel encoder message received
  
  //ros::Time current_time = ros::Time::now(); //TODO: alternately use encoder_msg.header.stamp??
  ros::Time current_time = encoder_msg.header.stamp;
  //read the encoder tics
  long left_encoder_ticks_current= encoder_msg.left;
  long right_encoder_ticks_current = encoder_msg.right;
  
  //calculate delta time
  g_encoder_dt = ( current_time - g_encoder_lasttime).toSec(); //returns a double like 0.10
  g_encoder_lasttime =  current_time;
  
  //Set current_rpm and radians_per_sec in motor struct
  //  also sets motor->previous_encoder_ticks
  calculate_motor_rpm_and_radian(&left_motor, left_encoder_ticks_current, g_encoder_dt);
  calculate_motor_rpm_and_radian(&right_motor, right_encoder_ticks_current, g_encoder_dt);
  
  //Debugging
  char buffer[80]; //string buffer for info logging
  /*sprintf (buffer, "  Right encoder_msg: %d", right_motor.previous_encoder_ticks); //right_encoder_ticks_current);
  ROS_INFO_STREAM(buffer);
  sprintf (buffer, "  Left encoder_msg: %d", left_motor.previous_encoder_ticks); //left_encoder_ticks_current);
  ROS_INFO_STREAM(buffer);  
  sprintf (buffer, "  Time Now: %15.4f",current_time.toSec());
  ROS_INFO_STREAM(buffer); 
  sprintf (buffer, "  Delta time: %10.4f", g_encoder_dt ); //right_encoder_ticks_current);
  ROS_INFO_STREAM(buffer); 
  */

  
}



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
  //nh_private_.param<std::string>("imu_topic", imu_topic, "imu/data");
  //nh_private_.param<std::string>("vel_topic", vel_topic, "raw_vel");
  
  setup();
  
  //ros Subscribers 
  ros::Subscriber encoder_sub = nh.subscribe(encoder_topic, 50, encoderCallback);
  

  //ros::Subscriber imu_sub = n.subscribe(imu_topic, 50, IMUCallback);
  
  //Ros publishers
  //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_frame, 50);
  

  double rate = 10.0; //TODO: this should be a parameter: odom_publish_rate
  
  
  ros::Rate r(rate); 
  while(nh.ok()){
    //see: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
    ros::spinOnce();
    ros::Time main_current_time = ros::Time::now(); 
    
    //debugging info
    //sprintf (buffer, "Motor Left RPM: %15.4f", left_motor.current_rpm);
    //ROS_INFO_STREAM(buffer);
    //sprintf (buffer, "Motor Right RPM: %15.4f", right_motor.current_rpm);
    //ROS_INFO_STREAM(buffer);
      
    g_last_loop_time = main_current_time;
    r.sleep();
  }
}