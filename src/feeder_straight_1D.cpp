///
/// if force € [0, FORCE_F_1_2_THRESH] then state 1 : lower cup
/// if force € (FORCE_F_1_2_THRESH, FORCE_F_2_3_THRESH] then state 2 : do nothing
/// if force € (FORCE_F_2_3_THRESH, MAX] then state 3 : raise cup
///
/// Fallback velocity functions
///	Vel.x				                                  Vel.y 
/// -180 to -90	  y =  0.004444444*x + 0.8			  -180 to   0	y =  0.004444444*x + 0.4
///  -90 to  90	  y = -0.004444444*x 			           0 to 180	y = -0.004444444*x + 0.4
///   90 to 180	  y =  0.004444444*x - 0.8				
///
///
///
/// TODO: IMPLEMENT SAFETY ABOVE 1.5N

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"
#include "std_msgs/Int32.h"
#include "audio_emergency/AudioMessage.h"


#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <hri_package/Sens_Force.h>
#include <kinova_msgs/PoseVelocity.h>
#include "../include/projectintegration.hpp"


#define FORCE_F_1_2_THRESH 0.3
#define FORCE_F_2_3_THRESH 0.5
#define FORCE_SAFETY 5
#define UPPER_FEED_ANGLE_THRESH 170 // Do not raise this above 175, bad things happen.
#define ROTATION_STEP 5



#define ARC_DOWN 1
#define ARC_UP 2
#define TRANSLATE_BACK 3
#define RAISE_CUP 4
#define LOWER_CUP 5
#define TRANSLATE_UP 6
#define TRANSLATE_DOWN 7
#define TRANSLATE_FRONT 8
#define END_EFF_FRAME "j2s7s300_end_effector"
#define BASE_FRAME "j2s7s300_link_base"
#define SENSOR_FRAME "forcesensor"

#define MAX_STEPS 200
#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4
#define VEL_CMD_DURATION 0.8

double lower_angle_thresh = 85;
float thresh_lin = 0.02;
float thresh_ang = 0.05;

int step_count;

std::string result;
std::mutex lock_pose;
std::mutex lock_status;
std::mutex lock_proc;
std::mutex lock_force;
std::mutex lock_emerg;
bool emergency = false;
double force_f, force_b;
bool processing=false;
ros::Publisher cmd_pos;
ros::Publisher cmd_vel;
geometry_msgs::PoseStamped current_pose, initial_pose;

void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);
}

void waitForActionCompleted()
{
  std::string temp_res="";
  while(ros::ok())
  {
    ros::spinOnce();
    lock_status.lock();
    temp_res = result;
    lock_status.unlock();

    if (temp_res=="Stopped")
    {
      ROS_INFO_STREAM("Status : " << temp_res);
      //Wait a little extra for arm to settle due to inertia.
      ros::Time start = ros::Time::now();
      while(ros::Time::now()-start < ros::Duration(0.2)) ros::spinOnce();
      break;
    }
  }
}

void setPoseForDirection(int direction, geometry_msgs::PoseStamped& start_pose,double  distance)
{
  geometry_msgs::Quaternion quat;
  switch(direction)
  {
  /// END EFFECTOR FRAME  Z-forward-blue, Y-up-green, X-left-red
  case ARC_DOWN :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0)); //Turn down
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0.01;  // Slightly up
    start_pose.pose.position.z=-0.01; // Slightly back
    start_pose.pose.orientation = quat;
    break;
  case ARC_UP :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(-ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=-0.01;
    start_pose.pose.position.z=0.01;
    start_pose.pose.orientation = quat;
    break;
  case TRANSLATE_BACK :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=-distance;
    start_pose.pose.orientation = quat;
    break;
  case TRANSLATE_FRONT :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=distance;
    start_pose.pose.orientation = quat;
    break;

  /// SENSORFRAME  X-forward-red, Y-left-green, Z-up-blue
  }
}

geometry_msgs::TwistStamped getTwistForDirection(int direction)
{
  geometry_msgs::TwistStamped twist;
  twist.header.stamp=ros::Time::now();
  twist.header.frame_id="j2s7s300_link_7";
  switch(direction)
  {
    case ARC_DOWN:
      ROS_INFO("Arc down");
      twist.twist.linear.z=VEL_LIN_MAX;
      twist.twist.angular.x=VEL_ANG_MAX;
    break;

    case ARC_UP:
      ROS_INFO("Arc up");
      twist.twist.linear.z=-VEL_LIN_MAX;
      twist.twist.angular.x=-VEL_ANG_MAX;
    break;

    case TRANSLATE_BACK:
      ROS_INFO("Translate back");
      twist.twist.linear.x=-VEL_LIN_MAX;
    break;

    case RAISE_CUP:
      ROS_INFO("Raise Cup");
      twist.twist.angular.x=VEL_ANG_MAX;
    break;

    case LOWER_CUP:
      ROS_INFO("Lower Cup");
      twist.twist.angular.x=-VEL_ANG_MAX;
    break;

    case TRANSLATE_UP:
      ROS_INFO("Translate up");
      twist.twist.linear.z=VEL_LIN_MAX;
    break;

    case TRANSLATE_DOWN:
      ROS_INFO("Translate down");
      twist.twist.linear.z=-VEL_LIN_MAX;
    break;

  }
  return twist;
}

bool checkUpperAngleThreshold();
bool checkLowerAngleThreshold();
void fallback(bool emerg=false);

bool isForceSafe()
{
  double local_force_f;
  lock_force.lock();
  local_force_f=force_f;
  lock_force.unlock();

  if (local_force_f >= FORCE_SAFETY)
  {
    ROS_WARN_STREAM("PAIN THRESHOLD BREACHED!!");
    return false;
  }
  else
    return true;
}

bool isAudioSafe()
{
  lock_emerg.lock();
  bool emerg = emergency;
  lock_emerg.unlock();

  if (emerg) ROS_WARN_STREAM("AUDIO EMERGENCY DETECTED!!");
  return !emerg;
}

void publishTwistForDuration(geometry_msgs::TwistStamped twist_msg, double duration)
{
  ros::Time time_start = ros::Time::now();
  while (ros::Time::now() - time_start < ros::Duration(duration))
  {
    ros::spinOnce();
    cmd_vel.publish(twist_msg);
  }
}

void positionControlDriveForDirection(int direction, double distance)
{
  geometry_msgs::PoseStamped start_pose, pose_in_base;
  tf::TransformListener tf_listener;
  //Wait for TF buffer to fill up
  ros::Duration(0.25).sleep();
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    try
    {
      lock_pose.lock();
      start_pose = current_pose;
      lock_pose.unlock();

      setPoseForDirection(direction, start_pose, distance);

      tf_listener.transformPose(BASE_FRAME,start_pose,pose_in_base);
      break;

    }
    catch (tf::ExtrapolationException e)
    {
     /* ROS_INFO_STREAM("Waiting for transform "
                      //<< e.what()
                      );*/
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  cmd_pos.publish(pose_in_base);
}

void driveToRollGoalWithVelocity(int direction)
{
  if(direction == RAISE_CUP)
    ROS_INFO_STREAM("Raise Cup");
  else if (direction == LOWER_CUP)
    ROS_INFO_STREAM("Lower Cup");
  else
  {
    ROS_ERROR_STREAM("Wrong direction given!!");
    return;
  }

  geometry_msgs::PoseStamped start_pose, goal_pose, temp_pose;

  lock_pose.lock();
  start_pose=current_pose;
  lock_pose.unlock();

  double temp_rol, temp_pit, temp_yaw;
  double goal_rol, goal_pit, goal_yaw;
  double start_rol, start_pit, start_yaw;
  double del_rol;

  goal_pose = start_pose;

  tf::Quaternion q_rot = tf::createQuaternionFromRPY(angles::from_degrees( direction==RAISE_CUP?ROTATION_STEP:-ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0)); // Rotate about x by 20 degrees
  tf::Quaternion q_start; tf::quaternionMsgToTF(start_pose.pose.orientation, q_start);
  tf::Quaternion q_goal = q_start*q_rot;

  tf::quaternionTFToMsg(q_goal,goal_pose.pose.orientation);


  ros::Rate loop_rate(100);
  //ros::Time start = ros::Time::now();
  while(ros::ok())
  {
    lock_pose.lock();
    temp_pose = current_pose;
    lock_pose.unlock();

    getRPYFromQuaternionMSG(goal_pose.pose.orientation, goal_rol, goal_pit, goal_yaw);
    getRPYFromQuaternionMSG(temp_pose.pose.orientation, temp_rol, temp_pit, temp_yaw);
    getRPYFromQuaternionMSG(start_pose.pose.orientation, start_rol, start_pit, start_yaw);

    del_rol = goal_rol - temp_rol;

    if (std::fabs(del_rol)>=thresh_ang)
    {
      geometry_msgs::TwistStamped twist_msg;
      twist_msg.twist.linear.x = 0;
      twist_msg.twist.linear.y = 0;
      // Comment out next line ONLY if using end-effector mode ie: Mode 1 on jostick is active and 4th blue LED on far right is active.
      twist_msg.twist.linear.z = direction == LOWER_CUP? -thresh_lin : thresh_lin;
      twist_msg.twist.angular.x= del_rol>0?VEL_ANG_MAX:-VEL_ANG_MAX;
      twist_msg.twist.angular.y = 0;
      twist_msg.twist.angular.z= 0;

      if(isForceSafe() && isAudioSafe())
      {
        cmd_vel.publish(twist_msg);
      }
      else
      {
        ROS_WARN("ABORTING SEQUENCE!!!");
        fallback(true);
        ros::Duration dur(1);
        dur.sleep();
        finished_task();
        break;
      }
    }
    else
    {
      //ROS_INFO_STREAM("Goal reached, breaking loop");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}

void moveCup(int direction, double duration=VEL_CMD_DURATION, double distance=0.1)
{
  if (direction==TRANSLATE_BACK || direction==TRANSLATE_FRONT)
  {
    positionControlDriveForDirection(direction, distance);
    return;
  }
  else
  {
    geometry_msgs::TwistStamped twist_msg = getTwistForDirection(direction);
    publishTwistForDuration(twist_msg, duration);

  }
  return;
}


void poseGrabber(geometry_msgs::PoseStamped pose)
{
  lock_pose.lock();
  current_pose=pose;
  lock_pose.unlock();
}

void forceGrabber(const hri_package::Sens_Force::ConstPtr msg)
{
  lock_force.lock();
  force_f=msg->forceF;
  force_b=msg->forceB;
  lock_force.unlock();
}

void statusGrabber(std_msgs::String::ConstPtr status)
{
  lock_status.lock();
  result=status->data;
  lock_status.unlock();
}

void waitForPoseDataAvailable()
{
  // This loop to wait until subscriber starts returning valid poses.
  ros::Rate loop_rate(9);
  geometry_msgs::PoseStamped temp_pose;
  while(ros::ok())
  {
    lock_pose.lock();
    temp_pose = current_pose;
    lock_pose.unlock();

    ros::spinOnce();
    loop_rate.sleep();

    if (temp_pose.pose.position.x != 0) break;
  }
  ROS_INFO("Pose data available");
}
double getCurrentRollDegrees();
bool checkUpperAngleThreshold()
{
  geometry_msgs::PoseStamped temp_pose;

  lock_pose.lock();
  temp_pose=current_pose;
  lock_pose.unlock();

  double temp_roll, temp_pitch, temp_yaw;/*
  getRPYFromQuaternionMSG(temp_pose.pose.orientation,temp_roll, temp_pitch, temp_yaw);*/
  temp_roll = getCurrentRollDegrees();

//  ROS_INFO_STREAM("Checking current roll " << temp_roll <<
//                  " + " << ROTATION_STEP <<
//                  " vs limit " << UPPER_FEED_ANGLE_THRESH );

//  ros::spinOnce();
//  lock_force.lock();
//  double force=force_f;
//  lock_force.unlock();

//  if (!isAudioSafe() || !isForceSafe())
//  {
//    ROS_WARN_STREAM("ABORTING SEQUENCE!!. F = " << force);
//    fallback(true);
//    ros::Duration dur(1);
//    dur.sleep();
//    finished_task();
//  }


  if ( temp_roll + ROTATION_STEP >= UPPER_FEED_ANGLE_THRESH)
  {
//    geometry_msgs::TwistStamped zero;
//    zero.twist.linear.x = zero.twist.linear.y = zero.twist.linear.z = zero.twist.angular.x = zero.twist.angular.y = zero.twist.angular.z =0;
//    zero.header.stamp = ros::Time::now();
//    cmd_vel.publish(zero);
    ROS_WARN_STREAM("MAX UPPER FEED ANGLE REACHED. F = "
//                    << force <<
                    " R = " << temp_roll);
    return false;
  }
  else
  {
   return true;
  }
}

bool checkLowerAngleThreshold()
{
  geometry_msgs::PoseStamped temp_pose;

  lock_pose.lock();
  temp_pose=current_pose;
  lock_pose.unlock();

  double temp_roll, temp_pitch, temp_yaw;
//  getRPYFromQuaternionMSG(temp_pose.pose.orientation,temp_roll, temp_pitch, temp_yaw);
  temp_roll = getCurrentRollDegrees();

  if (temp_roll - (int)angles::to_degrees(lower_angle_thresh) < ROTATION_STEP/2 )
  {
    ROS_WARN_STREAM("MAX LOWER ANGLE REACHED. LIMIT: " << angles::to_degrees(lower_angle_thresh) << " CURRENT: " << temp_roll);
    return false;
  }
  else
  {
   ROS_INFO_STREAM("Current roll: " << (temp_roll));
   return true;
  }
}

double getCurrentRoll()
{
  lock_pose.lock();
  geometry_msgs::PoseStamped temp_pose = current_pose;
  lock_pose.unlock();

  double temp_rol, temp_pitch, temp_yaw;
  getRPYFromQuaternionMSG(temp_pose.pose.orientation, temp_rol, temp_pitch, temp_yaw);

  return temp_rol;
}

double getCurrentRollDegrees(){
  double temp_roll = getCurrentRoll();
  if (temp_roll >= 0)  return angles::to_degrees(getCurrentRoll());
  return angles::to_degrees(getCurrentRoll()) + 360;
}


void callFallbackTimer(double duration)
{

  ros::Time start = ros::Time::now();
  ros::Duration run_for(duration);
  ros::Rate loop_rate(10);
  ROS_WARN_STREAM("Starting fallback timeout for: " << duration << "s.");
  while(ros::Time::now() - start <= run_for)
  {
    lock_force.lock();
    double force = force_f;
    lock_force.unlock();

    if (force>= FORCE_F_2_3_THRESH)
    {
      ROS_INFO_STREAM("Force detected, breaking out of timer");
      return;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_WARN_STREAM("Time out! Initialising fallback");
 // moveCup(TRANSLATE_BACK);
  fallback();
  finished_task();
}


void fallback(bool emerg)
{
  lock_pose.lock();
  geometry_msgs::PoseStamped  start_pose=current_pose;
  lock_pose.unlock();

  double roll, pitch, yaw;
  getRPYFromQuaternionMSG(start_pose.pose.orientation, roll, pitch, yaw);

  int yaw_degrees = angles::to_degrees(yaw);

  // ROS_INFO_STREAM("Current yaw float: " << angles::to_degrees(yaw) << " int " << yaw_degrees);

  geometry_msgs::TwistStamped twist_cmd;
 
  // y = mx + c
  double m = 0.004444444;
  double velx_c = 0.8;
  double vely_c = 0.4;


  /// Fallback velocity functions
  ///	Vel.x				                                  Vel.y 
  /// -180 to -90	  y =  0.004444444*x + 0.8			  -180 to   0	y =  0.004444444*x + 0.4
  ///  -90 to  90	  y = -0.004444444*x 			           0 to 180	y = -0.004444444*x + 0.4
  ///   90 to 180	  y =  0.004444444*x - 0.8				

  if (yaw_degrees >= -180 && yaw_degrees < -90)
  {
    //ROS_INFO_STREAM("Sector 1");
    twist_cmd.twist.linear.x = m * yaw_degrees + velx_c;
    twist_cmd.twist.linear.y = m * yaw_degrees + vely_c;
  }  
  else if (yaw_degrees >= -90 &&  yaw_degrees <0)
  {
    //ROS_INFO_STREAM("Sector 2");
    twist_cmd.twist.linear.x = -m * yaw_degrees;
    twist_cmd.twist.linear.y = m * yaw_degrees + vely_c;
  }
  else if (yaw_degrees >= 0 &&  yaw_degrees <90)
  {
    //ROS_INFO_STREAM("Sector 3");
    twist_cmd.twist.linear.x = -m * yaw_degrees;
    twist_cmd.twist.linear.y = -m * yaw_degrees + vely_c;
  }
  else if (yaw_degrees >= 90 &&  yaw_degrees <=180)
  {
    //ROS_INFO_STREAM("Sector 4");
    twist_cmd.twist.linear.x = m * yaw_degrees - velx_c;
    twist_cmd.twist.linear.y = -m * yaw_degrees + vely_c;
  }
  //ROS_INFO_STREAM("Publishing twist : ");
  //std::cout << (twist_cmd) << std::endl;
  if (emerg==true) twist_cmd.twist.angular.x = -0.5;
  publishTwistForDuration(twist_cmd,1);
}


void audioGrabber(audio_emergency::AudioMessage::ConstPtr msg)
{
  lock_emerg.lock();
  if (msg->result == "talk") emergency = true;
  else emergency = false;
  lock_emerg.unlock();
}

void projState_rawCallback(std_msgs::String msg)
{
  if(msg.data=="Start Task 3")
  {
    lock_pose.lock();
    initial_pose=current_pose;
    lock_pose.unlock();

    double r,p,y;
    getRPYFromQuaternionMSG(initial_pose.pose.orientation,r,p,y);
    lower_angle_thresh = r;
    ROS_INFO_STREAM("Lower feed angle thresh set to " << angles::to_degrees(lower_angle_thresh));
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "feeder_straight_1D");
  ros::NodeHandle nh;

  projInteg_init(nh, argc, argv);

  tf::TransformListener tf_listener;

  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceGrabber);
  cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );
  ros::Publisher arm_pose_pub = nh.advertise<std_msgs::Int32>("/arm_state", 1000);
  ros::Subscriber audio_emerg = nh.subscribe("/audio_emergency",1000, audioGrabber );
  ros::Subscriber projState_sub = nh.subscribe("/ProjectState", 1, projState_rawCallback);

  waitForPoseDataAvailable();

  double local_force_f;
  bool print_once_only=true;

  // No initial rotation because Felix's code already takes care of this
//  ROS_INFO_STREAM("Initial rotation from " << angles::to_degrees(getCurrentRoll()));
//  moveCup(RAISE_CUP, VEL_CMD_DURATION);
//  ros::Duration(1.0).sleep(); // Allow inertial settlement
//  ros::spinOnce();

  // No longer initialising lower angle here, but from projState_rawCallback whenever this node is activated
//  lock_pose.lock();
//  initial_pose=current_pose;
//  lock_pose.unlock();

//  double r,p,y;
//  getRPYFromQuaternionMSG(initial_pose.pose.orientation,r,p,y);
//  lower_angle_thresh = r;
//  ROS_INFO_STREAM("Lower feed angle thresh set to " << angles::to_degrees(lower_angle_thresh));

  step_count = 0;
  int prev_step_count = 0;

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::spinOnce();

    if(isActive()){
      lock_force.lock();
      local_force_f=force_f;
      lock_force.unlock();



      if (local_force_f >= 0  && local_force_f <= FORCE_F_1_2_THRESH  && checkLowerAngleThreshold())
      {
        ROS_INFO("---------------------------------------------------------------------");
        driveToRollGoalWithVelocity(LOWER_CUP);
        prev_step_count = step_count--;
        ROS_WARN_STREAM("Step : " << prev_step_count << " -> " << step_count  << " @ roll : " << angles::to_degrees(getCurrentRoll()));
        print_once_only=true;
        ROS_INFO("---------------------------------------------------------------------");
        ROS_INFO(" ");

        if (prev_step_count > step_count &&
            std::fabs((int)angles::to_degrees(getCurrentRoll()) - (int)angles::to_degrees(lower_angle_thresh)) <=  ROTATION_STEP/2  )
        {
          callFallbackTimer(3);
        }
      }

      else if (local_force_f >= FORCE_F_2_3_THRESH
               && local_force_f <= FORCE_SAFETY
               && checkUpperAngleThreshold())
      {
        ROS_INFO("---------------------------------------------------------------------");
        driveToRollGoalWithVelocity(RAISE_CUP);
        prev_step_count =  step_count++;
        ROS_WARN_STREAM("Step : " << prev_step_count << " -> " << step_count  << " @ roll : " << angles::to_degrees(getCurrentRoll()));
        print_once_only=true;
        ROS_INFO("---------------------------------------------------------------------");
        ROS_INFO(" ");
      }

      else if (print_once_only)
      {
        ROS_INFO("---------------------------------------------------------------------");
        ROS_INFO_STREAM("Do nothing.");
        print_once_only=false;
        ROS_WARN_STREAM("Step : " << prev_step_count << " -> " << step_count  << " @ roll : " << angles::to_degrees(getCurrentRoll()));
        ROS_INFO("---------------------------------------------------------------------");
        ROS_INFO(" ");

      }


      loop_rate.sleep();
      std_msgs::Int32 arm_pose_msg;
      arm_pose_msg.data=step_count;
      arm_pose_pub.publish(arm_pose_msg);
      ros::spinOnce();

      if (!isAudioSafe())
      {
        ROS_WARN_STREAM("ABORTING SEQUENCE!!");
        fallback(true);
        ros::Duration dur(1);
        dur.sleep();
        finished_task();
        break;
      }
      else
      {
  //      ROS_INFO("Audio safe");
      }
    }
  }
  return 0;
}
