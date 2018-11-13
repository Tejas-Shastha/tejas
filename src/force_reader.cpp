#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"


#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"


#include <hri_package/Sens_Force.h>
#include <kinova_msgs/PoseVelocity.h>


geometry_msgs::PoseStamped current_pose;
std::string result;
std::mutex lock_pose;
std::mutex lock_status;
ros::Publisher cmd_pos;


bool moveEndEffectorFromPose (const int command,const geometry_msgs::PoseStamped start_pose)
{
  geometry_msgs::PoseStamped target_pose=start_pose;
  switch(command)
  {
    case 1 :   target_pose.pose.position.z-=0.01;
                  ROS_INFO("Move down");
                  cmd_pos.publish(target_pose);
                  break;
    case 2 : target_pose.pose.position.z+=0.01;
                  ROS_INFO("Move up");
                  cmd_pos.publish(target_pose);
                  break;
    case 3 : target_pose.pose.position.x-=0.1;
                  ROS_INFO("Move back");
                  cmd_pos.publish(target_pose);
                  break;
  }
  return true;
}


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


void forceCallBack(const hri_package::Sens_Force::ConstPtr msg)
{
  lock_pose.lock();
  geometry_msgs::PoseStamped start_pose = current_pose;
  lock_pose.unlock();
  if (start_pose.pose.position.x==0)  return;

  ROS_INFO_STREAM("Received Force F : " << msg->forceF << " Force B : " << msg->forceB);
  ROS_INFO_STREAM("Current Position (cm)  X: " << (int)(start_pose.pose.position.x*100)
                  << " Y: " << (int)(start_pose.pose.position.y*100)
                  << " Z: " << (int)(start_pose.pose.position.z*100));

  // Quaternion to RPY
  tf::Quaternion quat;
  quat.setX(start_pose.pose.orientation.x);
  quat.setY(start_pose.pose.orientation.y);
  quat.setZ(start_pose.pose.orientation.z);
  quat.setW(start_pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  ROS_INFO_STREAM("Current Orientation(rad) R: " << roll << " P: " << pitch << " Y: " << yaw);




  if (msg->forceF >= 0.5 && msg->forceB <0.5)
  {
    //ROS_INFO_STREAM("Move cup down");
    moveEndEffectorFromPose(1,start_pose);
  }
  else if (msg->forceF <0.5 && msg->forceB >=0.5)
  {
    //ROS_INFO_STREAM("Move cup up");
    moveEndEffectorFromPose(2,start_pose);
  }
  else if (msg->forceF >= 0.5 && msg->forceB >=0.5)
  {
    //ROS_INFO_STREAM("Move cup back");
    moveEndEffectorFromPose(3,start_pose);
  }
  else
  {
    ROS_INFO_STREAM("Do nothing");
  }

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "force_reader");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceCallBack);
  cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );


  // This loop to wait until subscriber starts returning valid poses.
  ros::Rate loop_rate(9);
  geometry_msgs::PoseStamped temp_pose;
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


  ros::spin();

  return 0;
}
