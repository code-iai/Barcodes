#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

//Messages from the REFILLS project
#include <refills_msgs/Barcode.h>

#include <fstream>
using namespace std;
ofstream myfile;

int main(int argc, char** argv){

  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle node;
   

  myfile.open("/home/azucena/barcode/listener.txt", std::ofstream::out | std::ofstream::trunc);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
  
  ros::Time stamp_time;
  tf::StampedTransform transform;

  //shelf 5, 5 barcodes
    try{
       //listener.waitForTransform("/shelf", "/b22005632", ros::Time(0), ros::Duration(0.001));
      listener.lookupTransform("/shelf", "/b22005632", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 22005632 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
try{
      listener.lookupTransform("/shelf", "/b22005632", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 22005632 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b24476003", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 24476003 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b25011555", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 25011555 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b25011562", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 25011562 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b25349108", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 25349108 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    //shelf 4, 7 barcodes
    try{
      listener.lookupTransform("/shelf", "/b21240362", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 21240362 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b21766718", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 21766718 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b24227711", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 24227711 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b23313729", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 23313729 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b23779549", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 23779549 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b21923890", ros::Time(0), transform);
      stamp_time = transform.stamp_;

      myfile << stamp_time << " 21923890 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b22270306", ros::Time(0), transform);
      stamp_time = transform.stamp_;

      myfile << stamp_time << " 22270306 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    //shelf 3, 6 barcodes
    try{
      listener.lookupTransform("/shelf", "/b20047283", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 20047283 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b21960680", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 21960680 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b23323841", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 23323841 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b25231298", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 25231298 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b24249294", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 24249294 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b22355423", ros::Time(0), transform);
      stamp_time = transform.stamp_;

      myfile << stamp_time << " 22355423 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
//shelf 2, 7 barcodes
    try{
      listener.lookupTransform("/shelf", "/b25001839", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 25001839 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b20460884", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 20460884 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b22622891", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 22622891 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b20100551", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 20100551 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b20156527", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 20156527 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b25169379", ros::Time(0), transform);
      stamp_time = transform.stamp_;

      myfile << stamp_time << " 25169379 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b21254819", ros::Time(0), transform);
      stamp_time = transform.stamp_;

      myfile << stamp_time << " 21254819 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
// Shelf 1, 8 barcodes
    
    try{
      listener.lookupTransform("/shelf", "/b25442052", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 25442052 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }

    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b20279950", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 20279950 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b23841604", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 23841604 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b24573191", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 24573191 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b25348125", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 25348125 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b24026109", ros::Time(0), transform);
      stamp_time = transform.stamp_;
  
      myfile << stamp_time << " 24026109 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b24339612", ros::Time(0), transform);
      stamp_time = transform.stamp_;

      myfile << stamp_time << " 24339612 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }
    try{
      listener.lookupTransform("/shelf", "/b25079234", ros::Time(0), transform);
      stamp_time = transform.stamp_;

      myfile << stamp_time << " 25079234 ";
      myfile << transform.getOrigin().x()*100<<" "<< transform.getOrigin().y()*100 <<" "<< transform.getOrigin().z()*100<<"\n";
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(0.05).sleep();
    }

  rate.sleep();
  }

myfile.close();

  return 0;
};
