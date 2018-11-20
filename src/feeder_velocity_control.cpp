#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"


#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <hri_package/Sens_Force.h>
#include <kinova_msgs/PoseVelocity.h>


#define ARC_DOWN 1
#define ARC_UP 2
#define TRANSLATE_BACK 3
#define ROTATE_DOWN 4
#define END_EFF_FRAME "j2s7s300_end_effector"
#define BASE_FRAME "j2s7s300_link_base"
#define SENSOR_FRAME "forcesensor"
#define FORCE_F_TRIGGER_THRESH 0.5
#define FORCE_B_TRIGGER_THRESH 0.5
#define FORCE_F_PAIN_THRESH 1
#define FORCE_B_PAIN_THRESH 1
#define ROTATION_STEP 10
#define MAX_STEPS 2
#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4
#define VEL_CMD_DURATION 0.8

float thresh_lin = 0.01;
float thresh_ang = 0.15;

static int seq=0;

std::string result;
std::mutex lock_pose;
std::mutex lock_status;
std::mutex lock_proc;
std::mutex lock_force;
double force_f, force_b;
bool processing=false;
ros::Publisher cmd_pos;
ros::Publisher cmd_vel;
geometry_msgs::PoseStamped current_pose, initial_pose;
int current_step=0;


void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);

  //ROS_INFO_STREAM("Quaternion is : " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w());
  //ROS_INFO_STREAM("Converted R: " << roll << " P: " << pitch << " Y: " << yaw);
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
      //Wait 1.5s extra for arm to settle due to inertia.
      ros::Time start = ros::Time::now();
      while(ros::Time::now()-start < ros::Duration(1.5)) ros::spinOnce();
      break;
    }
  }
}


void setPoseForDirection(int direction, geometry_msgs::PoseStamped& start_pose)
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
    start_pose.pose.position.z=-0.1;
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
  twist.header.seq=seq++;
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
    case ROTATE_DOWN:
      ROS_INFO("Rotate down");
      twist.twist.angular.x=VEL_ANG_MAX;
  }
  return twist;
}

bool isForceSafe()
{
  double local_force_f,local_force_b;
  lock_force.lock();
  local_force_f=force_f;
  local_force_b=force_b;
  lock_force.unlock();

  if (local_force_f >= FORCE_F_PAIN_THRESH || local_force_b >= FORCE_F_PAIN_THRESH)
    return false;
  else
    return true;
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

void positionControlDriveForDirection(int direction)
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

      setPoseForDirection(direction, start_pose);

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

void moveCup(int direction, double duration=VEL_CMD_DURATION)
{
  ROS_INFO("Calling moveCup");
  if (direction!=TRANSLATE_BACK)
  {
    geometry_msgs::TwistStamped twist_msg = getTwistForDirection(direction);
    publishTwistForDuration(twist_msg, duration);
  }
  else
  {
    positionControlDriveForDirection(direction);
  }
  return;
}

void fallBack(int& current_step)
{
  while(current_step < 0)
  {
    moveCup(ARC_UP);
    current_step++;
  }
  while(current_step > 0)
  {
    moveCup(ARC_DOWN);
    current_step--;
  }
  if (current_step==0)
  {
    moveCup(TRANSLATE_BACK);
  }
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feeder_position_control");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener;

  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceGrabber);
  cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );

  ros::Rate loop_rate(9);

  waitForPoseDataAvailable();

  double local_force_f,local_force_b;
  bool print_once_only=true;

  lock_pose.lock();
  initial_pose=current_pose;
  lock_pose.unlock();

  moveCup(ROTATE_DOWN, 2*VEL_CMD_DURATION);

  while(ros::ok())
  {
    ros::spinOnce();

    lock_force.lock();
    local_force_b=force_f;  //Interchange sensors so that it makes practical sense
    local_force_f=force_b;
    lock_force.unlock();

    if (local_force_f >= FORCE_F_TRIGGER_THRESH && local_force_b <FORCE_B_TRIGGER_THRESH)
    {
      if(current_step < -MAX_STEPS)
      {
        ROS_WARN_STREAM("Max lower steps of " << MAX_STEPS << " reached.");
      }
      else
      {
        moveCup(ARC_DOWN);
        current_step--;
      }
      print_once_only=true;
    }
    else if (local_force_f <FORCE_F_TRIGGER_THRESH && local_force_b>=FORCE_B_TRIGGER_THRESH)
    {
      if (current_step > MAX_STEPS/2)
      {
        ROS_WARN_STREAM("Max upper steps of " << MAX_STEPS/2 << " reached.");
      }
      else
      {
        moveCup(ARC_UP);
        current_step++;
      }
      print_once_only=true;
    }
    else if (local_force_f >= FORCE_F_TRIGGER_THRESH && local_force_b >=FORCE_B_TRIGGER_THRESH)
    {
      fallBack(current_step);
      ROS_WARN("Closing node");
      break;
    }
    else if (print_once_only)
    {
      ROS_INFO_STREAM("Do nothing. Current step : " << current_step);
      print_once_only=false;
    }

    loop_rate.sleep();
  }

  ros::spinOnce();

  return 0;
}
