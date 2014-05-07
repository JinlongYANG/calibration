#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include "ros/ros.h"
#include <calibration/calibration_node.hpp>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "calibration",ros::init_options::NoRosout);
  ros::NodeHandle nh;

    Calibration_Node Calibration_Node(nh);

  // Main loop.
  while (nh.ok())
  {
    ros::spinOnce();
  }
  return 0;
}
