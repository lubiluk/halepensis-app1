// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

// PCL headers
#include <pcl/point_cloud.h>

// Std C++ headers
#include <string>
#include "node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "halepensis_app1");

  ros::NodeHandle nh, pnh("~");

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  cvp::Node node(nh, pnh);

  node.run();

  return 0;
}