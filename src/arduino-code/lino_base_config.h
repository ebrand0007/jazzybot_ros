#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define DEBUG 0

#define k_p 0.4 // P constant
#define k_i 0.0 // I constant
#define k_d 1.0 // D constant

#define pi 3.1415926 
#define two_pi 6.2831853

//define your motors' specs here
#define max_rpm 330 //motor's maximum RPM
#define encoder_pulse 8 //encoder's number of ticks per revolution 
#define gear_ratio 17.737 //motor's gear ratio
//ticsPerWheelRotation=141.9ish; //ticsPerMotorRev*gearRatio;



//define your robot base's specs here
//defaults
//#define wheel_diameter 0.069 //wheel's diameter in meters
//#define wheel_width 0.027 //wheel's width in meters
//#define track_width 0.22 // width of the plate you are using

#define wheel_diameter 0.2286 //wheel's diameter in meters 9"
#define wheel_width 0.0762 //wheel's width in meters 3"
#define track_width 0.5715 // width of the plate you are using - 22.5"?? between tires


//arduino intrupt pins:
//https://www.arduino.cc/en/Reference/AttachInterrupt
//Board                                   Digital Pins Usable For Interrupts
//Uno, Nano, Mini, other 328-based        2, 3
//Mega, Mega2560, MegaADK                 2, 3, 18, 19, 20, 21
//Micro, Leonardo, other 32u4-based       0, 1, 2, 3, 7
//Zero                                    all digital pins, except 4
//MKR1000 Rev.1                           0, 1, 4, 5, 6, 7, 8, 9, A1, A2
//Due                                     all digital pins
//teensey                                 all digital pins
//don't change this if you followed the schematic diagram
//ENCODER PINS
#define left_encoder_a 19 //interupt for left encoder
#define left_encoder_b 18 //interupt for left encoder
#define right_encoder_a 3 //interupt for right encoder 
#define right_encoder_b 2 //interupt for right encoder

//don't change this if you followed the schematic diagram
//MOTOR PINS
//Left Motor
#define left_motor_direction 7
#define left_motor_pwm 8

//Right Motor
#define right_motor_direction 9
#define right_motor_pwm 10

#endif
