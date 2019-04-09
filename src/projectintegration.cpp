#include "../include/projectintegration.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "ros/ros.h"

volatile bool taskActive = true;
bool isInitialised = false;
ros::Subscriber projState_sub, joystick_sub;
ros::Publisher projState_pub;

/**
 * @brief Callback function for the project status. Starts this task if the right term is send
 * @param state Most recent state of the project
 */
void projState_rawCallback(const std_msgs::String &state){
  const std::string start = "Start Task 3";
  if(!start.compare(state.data)){
    taskActive = true;
  } else {
    taskActive = false;
  }
}


/**
 * @brief Interprets the joystick commands
 */
void joystick_rawCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  enum buttons{ // All binary either 0 or 1
    A=0, B=1, X=2, Y=3, LB=4, RB=5, BACK=6, START=7, LOGITECH=8, LEFT_STICK_PRESS=9, RIGHT_STICK_PRESS=10
  };
  enum axes{
    LEFT_STICK_HORIZ=0, LEFT_STICK_VERTIC=1, RIGHT_STICK_HORIZ=3, RIGHT_STICK_VERTIC=4, // -1.0 - 0.0 (std value) - 1.0
    LT=2, RT=5, // 1.0 (std value) - -1.0
    NUM_PAD_HORIZ=6, NUM_PAD_VERTIC=7 // either -1, 0 (std value) or 1
  };
  if(msg->buttons[LB] && !msg->buttons[RB]){
    if(msg->buttons[X]){
      taskActive = true;
    }
    if(msg->buttons[B] || msg->buttons[A]){
      taskActive = false;
    }
  }
  else if(msg->buttons[RB] && !msg->buttons[LB] && (msg->buttons[A] || msg->buttons[B] || msg->buttons[X] || msg->buttons[Y])){
    taskActive = false;
  }
}

/**
 * @brief Initialises everything necessary for the project integration
 * @param nh ROS Nodehandle of the current node
 */
void projInteg_init(ros::NodeHandle nh, int argc, char** argv){
  for(int i=0; i<argc; i++){  // Search for trigger term
    if(!strcmp(argv[i], "useActivation")){
      isInitialised = true;
      break;
    }
  }
  if(isInitialised){  // Only initialise if requested
    joystick_sub = nh.subscribe("/joy",1,joystick_rawCallback);
    projState_sub = nh.subscribe("/ProjectState", 1, projState_rawCallback);
    projState_pub = nh.advertise<std_msgs::String>("/ProjectState", 1);
    taskActive = false;
  }
}

/**
 * @brief Function to be called when the task is finished
 */
void finished_task(){
  if(isInitialised){
    taskActive = false;
    std_msgs::String msg;
    msg.data = "Continue Task 2";
    projState_pub.publish(msg);
  }
}

/**
 * @brief Returns whether the task is currently active
 */
bool isActive(){
  return taskActive;
}
