#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include <fstream>
#include "using_markers/json.hpp"
#include <visualization_msgs/Marker.h>

using json = nlohmann::json;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  // Read from Json file
  std::ifstream ifs("/home/azucena/ros_ws/src/Barcodes/using_markers/barcodes_data.json", std::ifstream::in);
  json j = json::array({});
  ifs >> j;
  std::cout << std::setw(4) << j << "\n\n";

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    
    // Set marker parameters
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration();

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.017;
    marker.scale.y = 0.0001;
    marker.scale.z = 0.0045;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set the pose of each marker and publish it
    for(int i = 0; i <= 1; i++)
    {
      marker.id = i;

      marker.pose.position.x = j["barcodes"][i]["pose"]["position"]["x"].get<float>();
      marker.pose.position.y = j["barcodes"][i]["pose"]["position"]["y"].get<float>();
      marker.pose.position.z = j["barcodes"][i]["pose"]["position"]["z"].get<float>();
      marker.pose.orientation.x = j["barcodes"][i]["pose"]["orientation"]["x"].get<float>();
      marker.pose.orientation.y = j["barcodes"][i]["pose"]["orientation"]["y"].get<float>();
      marker.pose.orientation.z = j["barcodes"][i]["pose"]["orientation"]["z"].get<float>();
      marker.pose.orientation.w = j["barcodes"][i]["pose"]["orientation"]["w"].get<float>();

      marker_pub.publish(marker);
    }

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_WARN_ONCE("Subscriber created");
    r.sleep();
  }
}
