#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

void markersCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  ROS_INFO("I hear");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "markers_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("visualization_marker", 1000, markersCallback);
  ros::spin();

  return 0;
}
