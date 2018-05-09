// This node listens to the ROS image message topic "barcode/image",
// converts the sensor_msgs::Image to a HalconCpp::HImage,
// process the HalconCpp::HImage, publishes the barcode number and location on
// the " " topic. The image is then republished over ROS.

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <asr_halcon_bridge/halcon_image.h>

// Using image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>

#ifndef __APPLE__
#  include "HalconCpp.h"
#  include "HDevThread.h"
#  if defined(__linux__) && !defined(NO_EXPORT_APP_MAIN)
#    include <X11/Xlib.h>
#  endif
#else
#  ifndef HC_LARGE_IMAGES
#    include <HALCONCpp/HalconCpp.h>
#    include <HALCONCpp/HDevThread.h>
#  else
#    include <HALCONCppxl/HalconCpp.h>
#    include <HALCONCppxl/HDevThread.h>
#  endif
#  include <stdio.h>
#  include <HALCON/HpThread.h>
#  include <CoreFoundation/CFRunLoop.h>
#endif

using namespace HalconCpp;

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  // Local iconic variables
  HObject  ho_Image, ho_SymReg;
   
  // Local control variables
  HTuple  hv_BHandle, hv_Dec;
  HTuple  hv_Area, hv_Row, hv_Column;
  
  // Halcon image to process
  HImage *halcon_image;

public:
  ImageConverter()
    : it_(nh_){
    // Subscribe and  publish using image_transport
    image_sub_ = it_.subscribe("barcode/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("barcode/halcon_to_ROS_image", 1);
  }

  ~ImageConverter()
  {
    //
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg){
    halcon_bridge::HalconImagePtr halcon_ptr;

    try{
      halcon_ptr = halcon_bridge::toHalconCopy(msg);
    }
      catch (halcon_bridge::Exception& e){
      ROS_ERROR("halcon_bridge exception: %s", e.what());
      return;
    }
   
    // Process halcon_ptr->image using Halcon
    halcon_image = halcon_ptr->image;   
    Rgb1ToGray(*halcon_image, &ho_Image);

    // Find barcode in Halcon image
    CreateBarCodeModel(HTuple(), HTuple(), &hv_BHandle);
    FindBarCode(ho_Image, &ho_SymReg, hv_BHandle, "EAN-8", &hv_Dec);
    AreaCenter(ho_SymReg, &hv_Area, &hv_Row, &hv_Column);

    // Print barcode number and location               
    for (int i=1; i<= hv_Dec.Length(); i+=1){
      std::cout << "Barcode: " << hv_Dec[i-1].S() << "  Row: "
        << hv_Row[i-1].D() << "  Column: " << hv_Column[i-1].D() 
        << std::endl; 
    }
    std::cout << "." << std::endl;

    // Convert the Halcon image to a ROS image message and publish it
    image_pub_.publish(halcon_ptr->toImageMsg());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();

  return 0;
}
