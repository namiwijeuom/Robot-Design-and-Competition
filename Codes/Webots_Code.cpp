// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
// to run the robot continuously for 64s
#define TIME_STEP 64 
//setting maximum speed
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int count = 0;
  int color = 1;  // red = 0, blue = 1
  int turn = 1; // right = 1, left = -1
  //initializing motor
  Motor *front_right_wheel = robot->getMotor("front_right_wheel");
  Motor *front_left_wheel = robot->getMotor("front_left_wheel");
  Motor *rear_right_wheel = robot->getMotor("rear_right_wheel");
  Motor *rear_left_wheel = robot->getMotor("rear_left_wheel");
  
  Motor *front_right_arm_motor = robot->getMotor("front_right_arm_motor");
  Motor *front_left_arm_motor = robot->getMotor("front_left_arm_motor");
  Motor *front_arm_holder = robot->getMotor("front_arm_holder");   
  DistanceSensor *top_ds_front_left = robot->getDistanceSensor("top_ds_front_left");
  DistanceSensor *top_ds_front_right = robot->getDistanceSensor("top_ds_front_right");
  DistanceSensor *bottom_back_IR_8 = robot->getDistanceSensor("Bottom back IR sensor 08");
  DistanceSensor *bottom_back_IR_7 = robot->getDistanceSensor("Bottom back IR sensor 07");
  DistanceSensor *bottom_back_IR_6 = robot->getDistanceSensor("Bottom back IR sensor 06");
 DistanceSensor *bottom_back_IR_5 = robot->getDistanceSensor("Bottom back IR sensor 05");
  DistanceSensor *bottom_back_IR_4 = robot->getDistanceSensor("Bottom back IR sensor 04");
  DistanceSensor *bottom_back_IR_3 = robot->getDistanceSensor("Bottom back IR sensor 03");
  DistanceSensor *bottom_back_IR_2 = robot->getDistanceSensor("Bottom back IR sensor 02");
  DistanceSensor *bottom_back_IR_1 = robot->getDistanceSensor("Bottom back IR sensor 01");
  DistanceSensor *Bottom_back_right_forward_distance_sensor = robot->getDistanceSensor("Bottom_back_right_forward_distance_sensor");
  DistanceSensor *Bottom_back_left_forward_distance_sensor = robot->getDistanceSensor("Bottom_back_left_forward_distance_sensor");

    DistanceSensor * front_left_ir = robot->getDistanceSensor("front_left_ir");
  DistanceSensor * front_right_ir = robot->getDistanceSensor("front_right_ir");

  
  
  
// set the wheel motors run by velociy and not by position
  front_right_wheel->setPosition(INFINITY);
  front_left_wheel->setPosition(INFINITY);
  rear_right_wheel->setPosition(INFINITY);
  rear_left_wheel->setPosition(INFINITY);

  front_right_wheel->setVelocity(0.0);
  front_left_wheel->setVelocity(0.0);
  rear_right_wheel->setVelocity(0.0);
  rear_left_wheel->setVelocity(0.0);
  
  front_right_arm_motor->setPosition(0);
  front_left_arm_motor->setPosition(0);
  front_arm_holder->setPosition(0);
  top_ds_front_left-> enable(TIME_STEP);
  top_ds_front_right-> enable(TIME_STEP);
  bottom_back_IR_8-> enable(TIME_STEP);
bottom_back_IR_7-> enable(TIME_STEP);
 bottom_back_IR_6-> enable(TIME_STEP);
bottom_back_IR_5-> enable(TIME_STEP);
bottom_back_IR_4-> enable(TIME_STEP);
bottom_back_IR_3-> enable(TIME_STEP);
bottom_back_IR_2-> enable(TIME_STEP);
bottom_back_IR_1-> enable(TIME_STEP);

 front_left_ir-> enable(TIME_STEP);
 front_right_ir-> enable(TIME_STEP);

Bottom_back_right_forward_distance_sensor->enable(TIME_STEP);
Bottom_back_left_forward_distance_sensor->enable(TIME_STEP);
 
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  // robot
  while (robot->step(TIME_STEP) != -1) {
  
  
 // setting motors to maxspeed
  
  
  
  
 double bottom_back_IR_8_val =  bottom_back_IR_8 -> getValue(); 
 double bottom_back_IR_7_val =  bottom_back_IR_7 -> getValue(); 
 double bottom_back_IR_6_val =  bottom_back_IR_6 -> getValue(); 
 double bottom_back_IR_5_val =  bottom_back_IR_5 -> getValue(); 
 double bottom_back_IR_4_val =  bottom_back_IR_4 -> getValue(); 
 double bottom_back_IR_3_val =  bottom_back_IR_3 -> getValue(); 
 double bottom_back_IR_2_val =  bottom_back_IR_2 -> getValue(); 
 double bottom_back_IR_1_val =  bottom_back_IR_1 -> getValue(); 
 double Bottom_back_right_forward_distance_sensor_val =  Bottom_back_right_forward_distance_sensor -> getValue(); 

 double Bottom_back_left_forward_distance_sensor_val =  Bottom_back_left_forward_distance_sensor -> getValue(); 

  double  front_left_ir_val =   front_left_ir -> getValue(); 
 double  front_right_ir_val =   front_right_ir -> getValue(); 

  double top_ds_front_left_val =  top_ds_front_left -> getValue(); 
double top_ds_front_right_val =  top_ds_front_right -> getValue(); 

 
if (count == 0)

{
front_right_wheel->setVelocity(MAX_SPEED);
  front_left_wheel->setVelocity(MAX_SPEED);
  rear_right_wheel->setVelocity(MAX_SPEED);
  rear_left_wheel->setVelocity(MAX_SPEED);
if ((bottom_back_IR_1_val  >500) ||( bottom_back_IR_8_val  >500 )){

count = 1;
}

}

if  ( count == 1)
{
   front_right_wheel->setVelocity(MAX_SPEED);
  front_left_wheel->setVelocity(MAX_SPEED);
  rear_right_wheel->setVelocity(MAX_SPEED);
  rear_left_wheel->setVelocity(MAX_SPEED);
  if((bottom_back_IR_4_val  >500) && (bottom_back_IR_6_val  < 500 ) && (bottom_back_IR_8_val < 500)) 
 {
 // go to right
 rear_right_wheel->setVelocity(MAX_SPEED);
 rear_left_wheel->setVelocity(MAX_SPEED/3);
 front_right_wheel->setVelocity(MAX_SPEED);
 front_left_wheel->setVelocity(MAX_SPEED/3);
 
 }
 if((bottom_back_IR_3_val  < 500 )&& (bottom_back_IR_1_val < 500) && (bottom_back_IR_5_val  > 500)) {
 // go to left
 rear_right_wheel->setVelocity(MAX_SPEED/3);
 rear_left_wheel->setVelocity(MAX_SPEED);
 front_right_wheel->setVelocity(MAX_SPEED/3);
  front_left_wheel->setVelocity(MAX_SPEED);
 
}
 
 
if (top_ds_front_left_val < 1600.0 )
{

  rear_right_wheel->setVelocity(MAX_SPEED);
  rear_left_wheel->setVelocity(MAX_SPEED/1.5);
 front_right_wheel->setVelocity(MAX_SPEED);
  front_left_wheel->setVelocity(MAX_SPEED/1.5);
}
 
 // else if any sensor reads 1250 < x <2250 tuen left
if  ( top_ds_front_left_val > 1600 && top_ds_front_left_val < 1900 )
 
 {
rear_right_wheel->setVelocity(MAX_SPEED/1.5);
rear_left_wheel->setVelocity(MAX_SPEED);
front_right_wheel->setVelocity(MAX_SPEED/1.5);
front_left_wheel->setVelocity(MAX_SPEED);
 
 

} 


 
 if ( (bottom_back_IR_1_val  <500)&&(bottom_back_IR_8_val  <500))
{
count = 2;
 }
 
 }
 
// line following and wall following finished move to the next step

if (count == 2)
{
  front_right_wheel->setVelocity(MAX_SPEED);
  front_left_wheel->setVelocity(MAX_SPEED);
  rear_right_wheel->setVelocity(MAX_SPEED);
  rear_left_wheel->setVelocity(MAX_SPEED);

if( bottom_back_IR_8_val  == 1000 ){
count = 3;
}

 
  }
  
 if(count == 3)
 {
     if((bottom_back_IR_4_val  >500) && (bottom_back_IR_6_val  < 500 ) ) 
 {
 // go to right
 rear_right_wheel->setVelocity(MAX_SPEED);
 rear_left_wheel->setVelocity(MAX_SPEED/3);
 front_right_wheel->setVelocity(MAX_SPEED);
 front_left_wheel->setVelocity(MAX_SPEED/3);
 
 }
 if((bottom_back_IR_3_val  < 500 )&& (bottom_back_IR_5_val  > 500)) {
 // go to left
 rear_right_wheel->setVelocity(MAX_SPEED/3);
 rear_left_wheel->setVelocity(MAX_SPEED);
 front_right_wheel->setVelocity(MAX_SPEED/3);
  front_left_wheel->setVelocity(MAX_SPEED);
 
}

if ((front_left_ir_val != 1000)&(front_right_ir_val != 1000)){
count = 4;
} 
 
 }

if (count ==4){
// 
    
   rear_right_wheel->setVelocity(0);
 rear_left_wheel->setVelocity(0);
 front_right_wheel->setVelocity(0);
  front_left_wheel->setVelocity(0);
if((color == 1) & (front_right_ir_val >500 ) & (front_left_ir_val <500 ))
{
turn = 1;
}
else if((color == 0) & (front_right_ir_val >500 ) & (front_left_ir_val <500 ))
{
turn = -1;
}
else if((color == 1) & (front_left_ir_val >500 ) & (front_right_ir_val <500 ))
{
turn = -1;
}
else if((color == 1) & (front_left_ir_val >500 ) & (front_right_ir_val <500 ))
{
turn = 1;
}
std::cout << "front_left_ir_val " << front_left_ir_val << std::endl;
std::cout << "turn " << turn << std::endl;

count = 5;
turn=2
if (turn==1){
rear_right_wheel->setVelocity(MAX_SPEED);
 rear_left_wheel->setVelocity(-MAX_SPEED);
 front_right_wheel->setVelocity(MAX_SPEED);
  front_left_wheel->setVelocity(-MAX_SPEED);
 }
 else{
 rear_right_wheel->setVelocity(-MAX_SPEED);
 rear_left_wheel->setVelocity(MAX_SPEED);
 front_right_wheel->setVelocity(-MAX_SPEED);
  front_left_wheel->setVelocity(MAX_SPEED);
 
 }
}

// if (count ==5)
// {
// std::cout << "front_left_ir_val " << front_left_ir_val << std::endl;
  // while ((front_right_ir_val  > 900 ) | (front_left_ir_val  < 500)){
 // rear_right_wheel->setVelocity(MAX_SPEED);
 // rear_left_wheel->setVelocity(-MAX_SPEED);
 // front_right_wheel->setVelocity(MAX_SPEED);
  // front_left_wheel->setVelocity(-MAX_SPEED);
  // }
 // count = 6;
 
 
 // }
 // if (count==6){
 // front_right_wheel->setVelocity(0);
  // front_left_wheel->setVelocity(0);
  // rear_right_wheel->setVelocity(0);
  // rear_left_wheel->setVelocity(0);
// if ((Bottom_back_right_forward_distance_sensor_val  <45) ||( Bottom_back_left_forward_distance_sensor_val  <45 )){
 // front_right_wheel->setVelocity(0);
  // front_left_wheel->setVelocity(0);
  // rear_right_wheel->setVelocity(0);
  // rear_left_wheel->setVelocity(0);
  
 // front_right_arm_motor->setPosition(-0.2);
  // front_left_arm_motor->setPosition(0.2);
  // front_arm_holder->setPosition(-0.2);
 
// }}
}
 delete robot;
  return 0;

}
