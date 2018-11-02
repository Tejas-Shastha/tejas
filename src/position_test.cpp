#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "std_msgs/String.h"

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/PoseVelocity.h>

geometry_msgs::PoseStamped current_pose;
std::string result;
std::mutex lock_pose;
std::mutex lock_status;

void poseGrabber(geometry_msgs::PoseStamped::ConstPtr pose)
{
  lock_pose.lock();
  current_pose.pose.position.x = pose->pose.position.x;
  current_pose.pose.position.y = pose->pose.position.y;
  current_pose.pose.position.z = pose->pose.position.z;
  current_pose.pose.orientation.w = pose->pose.orientation.w;
  current_pose.pose.orientation.x = pose->pose.orientation.x;
  current_pose.pose.orientation.y = pose->pose.orientation.y;
  current_pose.pose.orientation.z = pose->pose.orientation.z;
  lock_pose.unlock();
  //ROS_INFO_STREAM("Got current pose as x=" << pose->pose.position.x << " saved as " << current_pose.pose.position.x);
}

void statusGrabber(std_msgs::String::ConstPtr status)
{
  lock_status.lock();
  result=status->data;
  lock_status.unlock();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_test");
  ros::NodeHandle nh;

  ros::Publisher cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );

  ros::Rate loop_rate(9);
  geometry_msgs::PoseStamped temp_pose;

  // This loop to wait until subscriber starts returning valid poses.
  while(ros::ok())
  {
    lock_pose.lock();
    temp_pose.pose.position.x = current_pose.pose.position.x;
    lock_pose.unlock();
    //ROS_INFO_STREAM(" Temp pose x = " << temp_pose.pose.position.x);

    ros::spinOnce();
    loop_rate.sleep();

    if (temp_pose.pose.position.x != 0) break;
  }

  ROS_INFO("Pose data available");

  lock_pose.lock();
  geometry_msgs::PoseStamped start_pose = current_pose;
  lock_pose.unlock();
  ROS_INFO_STREAM("Start pos : " << start_pose.pose.position.x);

  geometry_msgs::PoseStamped goal_pose = start_pose;
  goal_pose.pose.position.x+=0.01;
  goal_pose.pose.position.y+=0.01;
  goal_pose.pose.position.z+=0.01;

  ROS_INFO_STREAM("Publishing goal pose ");
  cmd_pos.publish(goal_pose);

  std::string temp_res;
  while(ros::ok())
  {
    ros::spinOnce();
    lock_status.lock();
    temp_res = result;
    lock_status.unlock();
    if (temp_res!="") break;
  }
  ROS_INFO_STREAM("Goal status available : " << temp_res);
  ros::Duration sleeper(0.5);
  ros::Time start = ros::Time::now();
  while(ros::Time::now()-start < sleeper)
  {
    ros::spinOnce();
  }


  lock_pose.lock();
  geometry_msgs::PoseStamped end_pose = current_pose;
  lock_pose.unlock();


  ROS_INFO_STREAM("End pos : " << end_pose.pose.position.x);
  ROS_INFO_STREAM("Total displacement x : " << -start_pose.pose.position.x + end_pose.pose.position.x);

  ros::spinOnce();
  return 0;
}
