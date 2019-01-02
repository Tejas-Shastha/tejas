/// DRIVE TO POSE


#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "mutex"
#include "kinova_msgs/PoseVelocity.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"

#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4

float thresh_lin = 0.01;
float thresh_ang = 0.05;

double temp_rol, temp_pit, temp_yaw;
double goal_rol, goal_pit, goal_yaw;
double start_rol, start_pit, start_yaw;
double del_rol, del_pit, del_yaw;


std::string result;
std::mutex lock_pose;
std::mutex lock_status;
std::mutex lock_proc;
std::mutex lock_force;
double force_f, force_b;
bool processing=false;
ros::Publisher cmd_pos;
ros::Publisher cmd_vel;


geometry_msgs::PoseStamped current_pose, start_pose, goal_pose;
std::mutex pose_lock;

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


void poseGrabber(geometry_msgs::PoseStamped pose)
{
  pose_lock.lock();
  current_pose = pose;
  pose_lock.unlock();
  //ROS_INFO_STREAM("Got current pose as x=" << pose << " saved as " << current_pose);
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_drive");
  ros::NodeHandle nh;

  //ros::Publisher cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  ros::Publisher pub_kinova_vel = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );

  ros::Rate loop_rate(100);
  geometry_msgs::PoseStamped temp_pose;

  geometry_msgs::TwistStamped vel_msg;


  // This loop to wait until subscriber starts returning valid poses.
  while(ros::ok())
  {
    pose_lock.lock();
    temp_pose = current_pose;
    pose_lock.unlock();
    ros::spinOnce();
    loop_rate.sleep();
    if (temp_pose.pose.position.x > 0) break;
  }

  ROS_INFO("Valid pose available");

  pose_lock.lock();
  start_pose=current_pose;
  pose_lock.unlock();

  goal_pose = start_pose;

  double roll, pitch, yaw;
  getRPYFromQuaternionMSG(goal_pose.pose.orientation,roll,pitch,yaw);

  tf::Quaternion q_rot = tf::createQuaternionFromRPY(angles::from_degrees(20),angles::from_degrees(0),angles::from_degrees(0)); // Rotate about x by 20 degrees
  tf::Quaternion q_start; tf::quaternionMsgToTF(start_pose.pose.orientation, q_start);
  tf::Quaternion q_goal = q_start*q_rot;

  goal_pose.pose.position.x+=0.0;
  goal_pose.pose.position.y+=0.0;
  goal_pose.pose.position.z+=0.0;

  tf::quaternionTFToMsg(q_goal,goal_pose.pose.orientation);

  getRPYFromQuaternionMSG(goal_pose.pose.orientation, goal_rol, goal_pit, goal_yaw);
  getRPYFromQuaternionMSG(start_pose.pose.orientation, start_rol, start_pit, start_yaw);

  ros::Time start = ros::Time::now();
  while(ros::ok())
  {
    pose_lock.lock();
    temp_pose = current_pose;
    pose_lock.unlock();

    float delta_lin_x = goal_pose.pose.position.x - temp_pose.pose.position.x;
    float delta_lin_y = goal_pose.pose.position.y - temp_pose.pose.position.y;
    float delta_lin_z = goal_pose.pose.position.z - temp_pose.pose.position.z;


    getRPYFromQuaternionMSG(goal_pose.pose.orientation, goal_rol, goal_pit, goal_yaw);
    getRPYFromQuaternionMSG(temp_pose.pose.orientation, temp_rol, temp_pit, temp_yaw);
    getRPYFromQuaternionMSG(start_pose.pose.orientation, start_rol, start_pit, start_yaw);

    del_rol = goal_rol - temp_rol;
    del_pit = goal_pit - temp_pit;
    del_yaw = goal_yaw - temp_yaw;

    if (std::fabs(del_rol)>=thresh_ang)
    {
      kinova_msgs::PoseVelocity kinova_vel;
      kinova_vel.twist_linear_x=  0;
      kinova_vel.twist_linear_y=  0;
      kinova_vel.twist_linear_z=  0;
      kinova_vel.twist_angular_x= del_rol>0?VEL_ANG_MAX:-VEL_ANG_MAX;
      kinova_vel.twist_angular_y= 0;
      kinova_vel.twist_angular_z= 0;

      geometry_msgs::TwistStamped twist_msg;
      twist_msg.twist.linear.x = 0;
      twist_msg.twist.linear.y = 0;
      twist_msg.twist.linear.z = 0;
      twist_msg.twist.angular.x= del_rol>0?VEL_ANG_MAX:-VEL_ANG_MAX;
      twist_msg.twist.angular.y = 0;
      twist_msg.twist.angular.z= 0;

      //ROS_INFO_STREAM("Deltas_ang MAINX: " << del_rol << " Y: " << del_pit << " Z: " << del_yaw);
      //std::cout << (kinova_vel) << std::endl;
      //pub_kinova_vel.publish(kinova_vel);

      cmd_vel.publish(twist_msg);
    }
    else
    {
      ROS_INFO_STREAM("Goal reached, breaking loop");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  ROS_INFO_STREAM("Start angles R: " << angles::to_degrees(start_rol) << " P: " << angles::to_degrees(start_pit) << " Y: " << angles::to_degrees(start_yaw));
  ROS_INFO_STREAM("goal angles R: " << angles::to_degrees(goal_rol) << " P: " << angles::to_degrees(goal_pit) << " Y: " << angles::to_degrees(goal_yaw));

  ros::spinOnce();
  return 0;
}
