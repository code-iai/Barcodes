#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv) {


  ros::init(argc, argv, "barcode_finder_node");
  
  ros::NodeHandle n;

  ros::Rate rate(1.0);

  while (ros::ok()) {

    //tuclase.procese_una_imagen();
    std::cout << "." << std::endl;
    rate.sleep();
  }



  return 0;




}
