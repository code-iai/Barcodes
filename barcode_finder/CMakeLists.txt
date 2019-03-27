cmake_minimum_required(VERSION 2.8.3)
project(barcode_finder)

## Make sure HALCONROOT is set
if(NOT DEFINED ENV{HALCONROOT})
  message( FATAL_ERROR "PLEASE SET HALCONROOT" )
endif()

add_definitions(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  asr_halcon_bridge 
  cv_bridge
  image_transport
  sensor_msgs
  refills_msgs
  tf
)

catkin_package(
   CATKIN_DEPENDS roscpp
     asr_halcon_bridge
     sensor_msgs
     std_msgs
     refills_msgs
)

include_directories(
  $ENV{HALCONROOT}/include
  $ENV{HALCONROOT}/include/halconcpp
  ${catkin_INCLUDE_DIRS}
  )

#link_directories($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})

add_executable(barcode_finder src/barcode_finder.cpp)

target_link_libraries(barcode_finder
   halconcpp
   halcon
   ${catkin_LIBRARIES}
)

