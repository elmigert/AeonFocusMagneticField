#include <ros/ros.h>

#include "a_vector_field/avectorfield.h"
#include "a_vector_field/adebug.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "a_vector_field");
  ros::NodeHandle nodeHandle("~");



  ROS_INFO_STREAM("Node loaded");







  a_vector_field::AVectorField AVectorField(nodeHandle);

  //husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}
