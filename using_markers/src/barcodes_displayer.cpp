#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include "using_markers/json.hpp"
#include <visualization_msgs/Marker.h>

using json = nlohmann::json;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "barcodes_displayer");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  // Read from Json file
  std::ifstream ifs("/home/azucena/ros_ws/src/Barcodes/using_markers/barcodes_data.json", std::ifstream::in);
  json j_array = json::array({});
  ifs >> j_array;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    
    // Set marker parameters
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "barcodes_displayer";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration();

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    //marker.scale.x = 0.017;
    //marker.scale.y = 0.0001;
    //marker.scale.z = 0.0045;
    marker.scale.x = 0.022;
    marker.scale.y = 0.0001;
    marker.scale.z = 0.0095;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set the pose of each marker and publish it
    for(int i = 0; i <= j_array["shelf"].size() - 1; i++) //Shelves
    {
      for(int j = 0; j <= j_array["shelf"][i]["barcode"].size() - 1; j++) //Barcodes
      {
        std:: string id_str = std::to_string(i) + std::to_string(j);
        int id = std::stoi(id_str);
        marker.id = id;

        marker.pose.position.x = j_array["shelf"][i]["barcode"][j]["pose"]["position"]["x"].get<float>();
        marker.pose.position.y = -(j_array["shelf"][i]["barcode"][j]["pose"]["position"]["y"].get<float>());
        marker.pose.position.z = j_array["shelf"][i]["barcode"][j]["pose"]["position"]["z"].get<float>();
        marker.pose.orientation.x = j_array["shelf"][i]["barcode"][j]["pose"]["orientation"]["x"].get<float>();
        marker.pose.orientation.y = j_array["shelf"][i]["barcode"][j]["pose"]["orientation"]["y"].get<float>();
        marker.pose.orientation.z = j_array["shelf"][i]["barcode"][j]["pose"]["orientation"]["z"].get<float>();
        marker.pose.orientation.w = j_array["shelf"][i]["barcode"][j]["pose"]["orientation"]["w"].get<float>();

        marker_pub.publish(marker);
      } 
    }
    r.sleep();
  }
}
