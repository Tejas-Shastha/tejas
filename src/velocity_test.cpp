#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"

geometry_msgs::PoseStamped current_pose;
std::mutex pose_lock;

void poseGrabber(geometry_msgs::PoseStamped::ConstPtr pose)
{

  pose_lock.lock();
  current_pose.pose.position.x = pose->pose.position.x;
  current_pose.pose.position.y = pose->pose.position.y;
  current_pose.pose.position.z = pose->pose.position.z;
  current_pose.pose.orientation.w = pose->pose.orientation.w;
  current_pose.pose.orientation.x = pose->pose.orientation.x;
  current_pose.pose.orientation.y = pose->pose.orientation.y;
  current_pose.pose.orientation.z = pose->pose.orientation.z;
  pose_lock.unlock();
  ROS_INFO_STREAM("Got current pose as x=" << pose->pose.position.x << " saved as " << current_pose.pose.position.x);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "drink_motion");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );

  ros::Rate loop_rate(9);
  geometry_msgs::PoseStamped temp_pose;

  geometry_msgs::TwistStamped vel_msg;
  vel_msg.twist.linear.x = 0.04;

  // This loop to wait until subscriber starts returning valid poses.
  while(ros::ok())
  {
    pose_lock.lock();
    temp_pose.pose.position.x = current_pose.pose.position.x;
    pose_lock.unlock();
    ROS_INFO_STREAM(" Temp pose x = " << temp_pose.pose.position.x);

    ros::spinOnce();
    loop_rate.sleep();

    if (temp_pose.pose.position.x > 0) break;
  }

  ROS_INFO("Broke out");

  pose_lock.lock();
  double start_pos_x = current_pose.pose.position.x;
  pose_lock.unlock();

  ros::Time start = ros::Time::now();
  ros::Duration runtime(1.0);

  while(ros::Time::now()-start < runtime)
  {
    pose_lock.lock();
    temp_pose.pose.position.x = current_pose.pose.position.x;
    pose_lock.unlock();

    ROS_INFO_STREAM("Publish x+0.04 @ "
                    << ros::Time::now()-start << " given current x = " << temp_pose.pose.position.x
                    );
    cmd_vel.publish(vel_msg);
    ros::spinOnce();
    //loop_rate.sleep();
  }

  pose_lock.lock();
  double end_pos_x = current_pose.pose.position.x;
  pose_lock.unlock();

  ROS_INFO_STREAM("Start pos : " << start_pos_x << " end pos : " << end_pos_x << " total displacement x : " << start_pos_x-end_pos_x);



  ros::spinOnce();
  return 0;
}
