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


const char *prog_name="jbot_base2_node";
const char *prog_ver="2.1.10";
/* Changelog
 * 2.0.10 - changed odom.header.frame_id = odom_frame, was wheel_odom_topic
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
//IMU sensor_msgs
#include <sensor_msgs/Imu.h>
//header file for cmd_subscribing to "cmd_vel"
//#include "ros_lib/geometry_msgs/Twist.h"
#include <geometry_msgs/Twist.h>
//header file for drive_robot_raw_publisher
#include "jbot2_msgs/jbot2_pwm.h"

#define pi 3.1415926 
#define two_pi 6.2831853

//TODO: these should be parameters
#define k_p 0.4 // P constant
#define k_i 0.0 // I constant
#define k_d 1.0 // D constant

int debug_level=0; 

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
  


//Pid values
float Kp = 0.4; // P constant
float Ki = 0.0; // I constant
float Kd = 1.0; // D constant

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
  if(x < a) {
    return a;
  }
  else if(b < x) {
    return b;
  }
  else
    return x;
}

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
std::string baselink_frame; //TF base frame for robot usually: baselink_frame or base_link
std::string odom_frame; // world frome to publish odom
std::string wheel_odom_topic; // name of wheel odom topic to publish
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

//global imu values
double g_imu_dt = 0.0; //imu delta time
double g_imu_z = 0.0; //angular velocity is the rotation in Z from imu_filter_madgwick's output
ros::Time g_last_imu_time(0.0);

//cmd_vel related
int raw_pwm_pub_hz=10; //rate to pubish raw_pwm_pub updates to hardware
double raw_pwm_next_pub_time=0; // used in main control loop for when to publish next pwm pid message time::now().toSec+(1/raw_pwm_pub_h); 

//prototypes
void calculate_pwm(Motor * mot);
void drive_robot(int, int, int);

//callback function prototypes for Ros subscribers
void command_callback( const geometry_msgs::Twist& cmd_msg);
double last_command_callback_time =0.0; //when exceeded, set desired rpms for motors to 0
//TODO: delete? void pid_callback( const lino_pid::linoPID& pid);

//ros publishers messages
jbot2_msgs::jbot2_pwm jbot2_pwm_msg;
ros::Publisher raw_pwm_pub;


void init_motor(Motor * mot)
{
  mot->previous_pid_error=0; 
  mot->total_pid_error=0;  
  mot->previous_encoder_ticks=0;  
  mot->encoder_lasttime=ros::Time::now().toSec(); 
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
  drive_robot(0,0,1000); //stop motors TODO: cant call this here, need nodehandler(nh) to be setup in main first

  char buffer[80]; //string buffer for info logging
  
  //Print info
  sprintf (buffer, "%s::Jazzybot Software Version: %s",prog_name,prog_ver);
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
  if ( debug_level > 1) 
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
    if ( debug_level > 1) 
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


//ROS subsciber call back for incoder tics
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

/* ----------------------------------------------------------------------------------------
 *
 *---------------------------------------------------------------------------------------- 
*/
//Ros subcriber callback for imu messages
void IMUCallback( const sensor_msgs::Imu& imu){
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
  
  //Issue is in this block, also releates to delta_theta calculation
  g_imu_dt = (current_time - g_last_imu_time).toSec();  //1.1.2 //get this from the header??
  g_last_imu_time = current_time;                       //1.1.2
}

/* ----------------------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------------------- 
 */
ros::Time g_last_loop_time(0.0); //time stamp for main loop

int main(int argc, char** argv){
  ros::init(argc, argv, "jbot_base_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");
  tf::TransformBroadcaster odom_broadcaster;
  
  char buffer[80]; //string buffer for info logging
  //set ros launch params
  nh_private_.param<std::string>("baselink_frame", baselink_frame, "base_link");
  nh_private_.param<std::string>("odom_frame", odom_frame, "odom"); 
  nh_private_.param<std::string>("wheel_odom_topic", wheel_odom_topic, "/odometry/wheelodom");
  nh_private_.param<std::string>("encoder_topic",encoder_topic,"encoders");
  nh_private_.param<std::string>("imu_topic", imu_topic, "imu/data");
  nh_private_.param("odom_publish_rate", odom_publish_rate, 40); //odom pubish rate hz
  nh_private_.param("raw_pwm_pub_hz",raw_pwm_pub_hz,10); //rate in hrz to update the hardware pid
  nh_private_.param("debug_level",debug_level,0); //debug_level 1 to 10
  //nh_private_.param<std::string>("vel_topic", vel_topic, "raw_vel");
  
  //ROS Subscribers 
  ros::Subscriber encoder_sub = nh.subscribe(encoder_topic, 50, encoderCallback); //TODO: 50 should be a parameter
  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 30, IMUCallback); //TODO: 30 should be a parameter
  ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 30, command_callback); //Command callback Subscriber
  //ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
  // Delete? ros::Subscriber<lino_pid::linoPID> pid_sub("pid", pid_callback);
  
  //Ros publishers
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(wheel_odom_topic, odom_publish_rate);
  raw_pwm_pub = nh.advertise<jbot2_msgs::jbot2_pwm>("raw_pwm",raw_pwm_pub_hz);
  
  //raw_pwm timmer
  raw_pwm_next_pub_time=ros::Time::now().toSec()+double(1.0/raw_pwm_pub_hz);  //Set next publish time
  
  setup();

  double rate = odom_publish_rate; //launch parameter: odom_publish_rate 
  ros::Rate r(rate); 
  while(nh.ok())
  {
    //see: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
    ros::spinOnce();
    ros::Time main_current_time = ros::Time::now();  
    double main_current_time_toSec=main_current_time.toSec();
    
    /*************************/
    //compute odometry in a typical way given the velocities of the robot
    double dt = (main_current_time - g_last_loop_time).toSec();
    
    vx = (left_motor.linear_vel + right_motor.linear_vel)/2; //Ave vel in x direction
    double delta_x = (vx* cos(th) - vy * sin(th)) * dt;  
    
    vy = 0; //wheels  cant roll  in y direction
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;  
    
    //velocity for z(yaw) 
       // note, relying on this with just wheel odom subject to lots of slippage
       /*vth = (right_motor.linear_vel - left_motor.linear_vel)/(track_width+wheel_width);  
       double delta_th = vth * dt;
       */
    
      //TODO: Alternately, get this from or fuse with IMU pose data
      vth= g_imu_z; //angular velocity is the rotation in Z from imu_filter_madgwick's output
      double delta_th = vth * dt; //g_imu_dt;
    
    
    //update x,y, theta(direction)
    x += delta_x;     
    y += delta_y;
    th += delta_th;  //theta is yaw, rotation around z axix
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    
    //first, we'll publish the odom transform world frame over tf
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
    
    //next, we'll publish the odometry topics over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = main_current_time;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = baselink_frame;
    
    
    //set the pose position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;    
    //Pose Covariance matrix: TODO: these should not be hard set
    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.01;
    odom.pose.covariance[14] = 99999;
    odom.pose.covariance[21] = 99999;
    odom.pose.covariance[28] = 99999;
    odom.pose.covariance[35] = 0.01;
    
    //set the velocity twist
    
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth; //The   
    //Twist Covariance matrix: TODO: these should not be hard set
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

    //calculate pid
    //  No need to read current motor rpms, the encoderCallback sets current encoder_msg are received
    calculate_pwm(&left_motor);
    calculate_pwm(&right_motor);

    //sprintf (buffer, "*rosTime.tosec(): %15.4f",main_current_time.toSec());
    //ROS_INFO_STREAM(buffer);
    //PID control loop, updates needed pid every at intervals defined in raw_pwm_pub_hz
    if ( main_current_time_toSec > raw_pwm_next_pub_time) 
    {
      
      int millis_duration=600; //Oh crap timeout in millisec to stop motors when no new pwm signal is received\
      
      double command_callback_timediff=main_current_time_toSec - last_command_callback_time; 
      //this block stops the motor when no command is received
      if (command_callback_timediff > 0.600) //TODO: need to get time interval last cmd_vel was received, when exceeded  stop motors 
      {
        if (debug_level > 1) 
        {
          sprintf (buffer, "     main_current_time_toSec: %15.4f",main_current_time_toSec);
          ROS_INFO_STREAM(buffer);
          sprintf (buffer, "     last_command_callback_time: %15.4f",last_command_callback_time);
          ROS_INFO_STREAM(buffer);
          sprintf (buffer, "     *cmd_vel exceeded. reseting requred_rpm to 0 Timediff: %15.4f",command_callback_timediff); //double(main_current_time_toSec - last_command_callback_time));
          ROS_INFO_STREAM(buffer);
        }
        
        
        left_motor.required_rpm = 0;
        right_motor.required_rpm = 0;        
        right_motor.pwm = 0; //TODO, leads to jerk
        left_motor.pwm = 0; //TODO, leads to jerk 
        //calculate_pwm(&left_motor);   //TODO: added to slow motors slowly to remove jerk
        //calculate_pwm(&right_motor);  //TODO: added to slow motors slowly tp remove jerk
      }
      
      
      //Send calculated motor pwm values to the hardware to move the robot
      drive_robot(left_motor.pwm,right_motor.pwm,millis_duration); //drive robot with pwm signals for millis_duration      
      //Calculate next publish time
      raw_pwm_next_pub_time=main_current_time_toSec+double(1.0/raw_pwm_pub_hz);  //set next pid update interval 
      //sprintf (buffer, "  *raw_pwm_next_pub_time: %15.4f",raw_pwm_next_pub_time);
      //ROS_INFO_STREAM(buffer);          
      }
      

    g_last_loop_time = main_current_time;
    r.sleep();
  }
}

/* ----------------------------------------------------------------------------------------
 * calculate_pwm: this function takes a Motor object argument, 
 * implements a PID controller and calculates the PWM required to drive the motor*
 *---------------------------------------------------------------------------------------- 
 */
void calculate_pwm(Motor * mot)
{

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



/* ----------------------------------------------------------------------------------------
 * Callback when a cmd_vel message is recieved
 *---------------------------------------------------------------------------------------- 
 */
void command_callback( const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  
  last_command_callback_time = ros::Time::now().toSec();
  
  //TODO: Do we need these?
  left_motor.required_rpm=0;
  right_motor.required_rpm =0;
  
  
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
  if (debug_level > 0) 
  {
    char buffer[80]; //string buffer for info logging
    sprintf (buffer, "  *left_motor.required_rpm: %15.4f",left_motor.required_rpm);
    ROS_INFO_STREAM(buffer);
    sprintf (buffer, "  *right_motor.required_rpm: %15.4f",right_motor.required_rpm);
    ROS_INFO_STREAM(buffer);
  }
  
  //Note: main Control loop recaluclates pid based and publishes raw_pwm
  
}
/* ----------------------------------------------------------------------------------------
 * drive_robot - raw motor control:  
 * this functions spins the left and right wheel based on a defined speed in +/-PWM  for a duration given in millisec
 * writes jbot2_msgs::jbot2_pwm(int pwm,int pwm,int duration) to arduino
 *---------------------------------------------------------------------------------------- 
 */
//void drive_robot( int command_left, int command_right)
void drive_robot( int left_pwm, int right_pwm, int duration)
{
  //Debugging
  /*
  char buffer[80]; //string buffer for info logging
  sprintf (buffer, "  Recieved left_pwm_pwm: %d", left_pwm);
  ROS_INFO_STREAM(buffer);  
  sprintf (buffer, "  Recieved pwm_right_pwm: %d", pwm_right);
  ROS_INFO_STREAM(buffer);
  */
  
  //publish write to raw_pwm topic jbot2_msgs::jbot2_pwm(pwm,pwm,duration) to arduino
  jbot2_pwm_msg.header.stamp = ros::Time::now(); 
  //jbot2_pwm_msg.header.frame_id = odom_frame; 
  jbot2_pwm_msg.left_pwm=left_pwm;
  jbot2_pwm_msg.right_pwm=right_pwm;
  jbot2_pwm_msg.duration=duration;
  raw_pwm_pub.publish(jbot2_pwm_msg);
}

/* ----------------------------------------------------------------------------------------
 * 
 *---------------------------------------------------------------------------------------- 
 */


/* ----------------------------------------------------------------------------------------
 * 
 *---------------------------------------------------------------------------------------- 
 */