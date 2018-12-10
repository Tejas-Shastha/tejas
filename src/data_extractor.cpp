#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"
#include <stdio.h>
#include <fstream>
#include "sstream"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <hri_package/Sens_Force.h>
#include <kinova_msgs/PoseVelocity.h>


std::mutex lock_pose;
std::mutex lock_force;
double force_f_received, force_b_received;
int USER_ID;
std::string FILE_NAME;
ros::Time start_time;


void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);
}

void forceGrabber(const hri_package::Sens_Force::ConstPtr msg)
{
  lock_force.lock();
  force_f_received=msg->forceF;
  force_b_received=msg->forceB;
  lock_force.unlock();



  std::ofstream force_writer;
  std::string force_filename = "/home/lukas/data/extracted_data/" + std::to_string(USER_ID) + "/" + FILE_NAME + "_max_rate_forces.ods";

  ROS_INFO_STREAM("Writing max rate force data to : " << force_filename);

  force_writer.open(force_filename, std::ios_base::app);
  force_writer << force_f_received << "\t"
               << force_b_received << "\t"
               << "\n";
  force_writer.close();
}

void poseGrabber(geometry_msgs::PoseStamped pose)
{

  double roll, pitch, yaw;
  getRPYFromQuaternionMSG(pose.pose.orientation, roll, pitch, yaw);
  std::ofstream tool_writer;
  std::string tool_filename="/home/lukas/data/extracted_data/" + std::to_string(USER_ID) + "/" + FILE_NAME + "_synchronised.ods";

  ROS_INFO_STREAM("Writing sync data to : /home/lukas/data/extracted_data/" << USER_ID << "/" << FILE_NAME << "_synchronised.ods" );

  lock_force.lock();
  double forceF=force_f_received;
  double forceB=force_b_received;
  lock_force.unlock();

  tool_writer.open(tool_filename, std::ios_base::app);
  tool_writer << (float)(ros::Time::now().toSec() - start_time.toSec())  << "\t"
              << pose.pose.position.x << "\t"
              << pose.pose.position.y << "\t"
              << pose.pose.position.z << "\t"
              << angles::to_degrees(roll) << "\t"
              << angles::to_degrees(pitch) << "\t"
              << angles::to_degrees(yaw) << "\t"
              << forceF << "\t"
              << forceB << "\t"
              << 0.4 << "\t"   // Trigg_thresh_F
              << 0.4 << "\t"   // Trigg_thresh_B
              << 2.5 << "\t"   // Pain_Thresh_F
              << 2.5 << "\t"   // Pain_Thresh_B
              << "\n";
  tool_writer.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_extractor");
  ros::NodeHandle nh;

  USER_ID=std::stoi(argv[1]);
  ROS_INFO_STREAM("User ID: " << USER_ID);

  FILE_NAME = argv[2];

  std::ofstream tool_writer;
  std::string tool_filename="/home/lukas/data/extracted_data/" + std::to_string(USER_ID) + "/" + FILE_NAME + "_synchronised.ods";

  ROS_INFO_STREAM("Writing data to" << tool_filename);

  tool_writer.open(tool_filename, std::ios_base::app);
  tool_writer << "TIME(sec)" << "\t"
              << "X" << "\t"
              << "Y" << "\t"
              << "Z" << "\t"
              << "ROLL" << "\t"
              << "PITCH" << "\t"
              << "YAW" << "\t"
              << "FORCE_F" << "\t"
              << "FORCE_B" << "\t"
              << "Trigg_Thresh_F" << "\t"
              << "Trigg_Thresh_B" << "\t"
              << "Pain_Thresh_F" << "\t"
              << "Pain_Thresh_B" << "\t"
              << "\n";
  tool_writer.close();

  std::ofstream force_writer;
  std::string force_filename = "/home/lukas/data/extracted_data/" + std::to_string(USER_ID) + "/" + FILE_NAME + "_max_rate_forces.ods";

  force_writer.open(force_filename, std::ios_base::app);
  force_writer << "ForceF" << "\t"
               << "ForceB" << "\t"
               << "\n";
  force_writer.close();

  sleep(0.25);
  ROS_INFO("Ready to extract");


  start_time = ros::Time::now();

  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceGrabber);

  ros::spin();

  return 0;
}
