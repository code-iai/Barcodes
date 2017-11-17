#include <iostream>
#include <ros/ros.h>
#include <halcon_image.h>

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

void action(){
  // Local iconic variables                                                   
  HObject  ho_Image2, ho_Image, ho_SymReg;

  // Local control variables                                                  
  HTuple  hv_i, hv_FileName, hv_BHandle, hv_Dec;
  HTuple  hv_Area, hv_Row, hv_Column;

  // This loop takes a defined number of images                                
  for (hv_i=1; hv_i<=9; hv_i+=1){
    // Read image from file and process it
    hv_FileName[hv_i] = ("/home/azucena/barcode/image_"+hv_i)+".jpg";
    ReadImage(&ho_Image2, HTuple(hv_FileName[hv_i]));
    Rgb1ToGray(ho_Image2, &ho_Image);
                                          
    // Find barcode in image             
    CreateBarCodeModel(HTuple(), HTuple(), &hv_BHandle);
    FindBarCode(ho_Image, &ho_SymReg, hv_BHandle, "EAN-8", &hv_Dec);   
    AreaCenter(ho_SymReg, &hv_Area, &hv_Row, &hv_Column);

    // Print barcode number and location
    std::cout << "Image " << hv_i.D() << std::endl;                                 for (int i=1; i<= hv_Dec.Length(); i+=1){
        std::cout << "Barcode: " << hv_Dec[i-1].S()<< "  Row: "<< 
        hv_Row[i-1].D()<< "  Column: "<< hv_Column[i-1].D() <<
        std::endl;
    }
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "barcode_finder_node");
  ros::NodeHandle n;
  ros::Rate rate(1.0);
  
  while (ros::ok()){
    action();
    std::cout << "." << std::endl;
    rate.sleep();
  }

  return 0;
}
