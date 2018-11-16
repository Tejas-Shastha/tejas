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


#define MOVE_DOWN 1
#define MOVE_UP 2
#define MOVE_BACK 3
#define END_EFF_FRAME "j2s7s300_end_effector"
#define BASE_FRAME "j2s7s300_link_base"


std::string result;
std::mutex lock_pose;
std::mutex lock_status;
std::mutex lock_proc;
std::mutex lock_force;
double force_f, force_b;
bool processing=false;
ros::Publisher cmd_pos;
geometry_msgs::PoseStamped current_pose;


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
    ROS_INFO_STREAM("Status : " << temp_res);
    if (temp_res=="Stopped")
    {
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
  case MOVE_DOWN :
    ROS_INFO("Move down");
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(20),angles::from_degrees(0),angles::from_degrees(0)); //Turn down
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0.01;  // Slightly up
    start_pose.pose.position.z=-0.01; // Slightly back
    start_pose.pose.orientation = quat;
    break;
  case MOVE_UP :
    ROS_INFO("Move up");
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(-20),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=-0.01;
    start_pose.pose.position.z=0.01;
    start_pose.pose.orientation = quat;
    break;
  case MOVE_BACK :
    ROS_INFO("Move back");
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(-20),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=-0.1;
    start_pose.pose.orientation = quat;
    break;
  }
}



void moveCup(int direction)
{
  ROS_INFO("Calling moveCup");
  geometry_msgs::PoseStamped start_pose, pose_in_base;
  tf::TransformListener tf_listener;
  ros::Duration(0.25).sleep();

  ros::Rate loop_rate(9);
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
      ROS_INFO_STREAM("Waiting for transform "
                      //<< e.what()
                      );
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("Start pose :" << start_pose);
  ROS_INFO_STREAM("pose_in_base pose :" << pose_in_base);
  ROS_INFO_STREAM("Publishing goal pose ");
  cmd_pos.publish(pose_in_base);

  waitForActionCompleted();
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
  ros::init(argc, argv, "force_reader");
  ros::NodeHandle nh;


  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceGrabber);
  cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );


  ros::Rate loop_rate(9);

  waitForPoseDataAvailable();

  double local_force_f,local_force_b;

  while(ros::ok())
  {
    ros::spinOnce();

    lock_force.lock();
    local_force_b=force_b;
    local_force_f=force_f;
    lock_force.unlock();

    if (local_force_f >= 0.5 && local_force_b <0.5)
    {
      moveCup(MOVE_DOWN);
    }
    else if (local_force_f <0.5 && local_force_b>=0.5)
    {
      moveCup(MOVE_UP);
    }
    else if (local_force_f >= 0.5 && local_force_b >=0.5)
    {
      moveCup(MOVE_BACK);
    }
    else
    {
      ROS_INFO_STREAM("Do nothing");
    }
    loop_rate.sleep();
  }

  ros::spinOnce();

  return 0;
}
