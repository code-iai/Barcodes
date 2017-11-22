#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include "using_markers/json.hpp"

using json = nlohmann::json;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "json_reader");
  ros::NodeHandle n;
  //while (ros::ok())
  //{

  std::ifstream ifs("/home/azucena/ros_ws/src/Barcodes/using_markers/barcodes.json", std::ifstream::in);
  json j = json::array({});
  ifs >> j;
  std::cout << std::setw(4) << j << "\n\n";

std::cout << j.is_array() << '\n';
std::cout << j.at("/barcodes/1/code"_json_pointer) << '\n';

  //}
}
