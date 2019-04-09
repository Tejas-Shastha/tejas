#ifndef PROJECTINTEGRATION_HPP
#define PROJECTINTEGRATION_HPP

  #include "std_msgs/String.h"
  #include "ros/ros.h"

  /**
   * @brief Initialises everything necessary for the project integration
   * @param nh ROS Nodehandle of the current node
   * @param argc, argv direct input of the node start
   */
  void projInteg_init(ros::NodeHandle nh, int argc, char** argv);

  /**
   * @brief Returns whether the task is currently active
   */
  bool isActive();

  /**
   * @brief Function to be called when the task is finished
   */
  void finished_task();

#endif // PROJECTINTEGRATION_HPP
