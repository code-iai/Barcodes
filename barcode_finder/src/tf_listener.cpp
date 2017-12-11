#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


//Messages from the REFILLS project
#include <refills_msgs/Barcode.h>

#include <fstream>
using namespace std;
ofstream myfile;

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;
  ros::Publisher barcode_pose = node.advertise<refills_msgs::Barcode>("/tf_listener/pose", 10);
  
  tf::TransformListener listener;
  ros::Time stamp_time;

//myfile.open ("/home/azucena/barcode/example.txt");
myfile.open("/home/azucena/barcode/example.txt", std::ofstream::out | std::ofstream::trunc);

std::vector<ros::Time> bar_code_vector;
std::vector<std::vector<double>> barcodes_vector;
ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("/tf_listener/marker", 1);   

// Create a barcode-marker to display it in rviz
    visualization_msgs::Marker marker;
    marker.ns = "/tf_listener";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "/my_frame";
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.022;
    marker.scale.y = 0.001;
    marker.scale.z = 0.0095;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;


  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/my_frame", "/new",  
                               ros::Time(0), transform);
      stamp_time = transform.stamp_;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }


// Filling the refills message
      refills_msgs::Barcode barcode_msg;
      barcode_msg.barcode_pose.header.stamp = stamp_time;     
      barcode_msg.barcode_pose.pose.position.x = transform.getOrigin().x();
      barcode_msg.barcode_pose.pose.position.y = transform.getOrigin().y();
      barcode_msg.barcode_pose.pose.position.z = transform.getOrigin().z();
/*
      barcode_msg.barcode_pose.pose.orientation.x = q[0];
      barcode_msg.barcode_pose.pose.orientation.y = q[1];
      barcode_msg.barcode_pose.pose.orientation.z = q[2];
      barcode_msg.barcode_pose.pose.orientation.w = q[3];
*/
      barcode_pose.publish(barcode_msg);

      marker.header.stamp = stamp_time;

// Fill a "row" vector with the barcode information
      std::vector<double> row(7);
      row[0] = transform.getOrigin().x();
      row[1] = transform.getOrigin().y();
      row[2] = transform.getOrigin().z();
/*
      row[3] = q[0];
      row[4] = q[1];
      row[5] = q[2];
      row[6] = q[3];
*/

      // Check if the barcode has already been detected
      int b = 0;
      for(int a = 0; a < barcodes_vector.size(); a++)
      {
        if (bar_code_vector[a] == stamp_time) //Update data
        {
std::cout << "updated"<<std::endl;
         myfile << "up" << " ";
          b = 1;
          // Update barcode   
          barcodes_vector[a][0] = (barcodes_vector[a][0] + row[0])/2;
          barcodes_vector[a][1] = (barcodes_vector[a][1] + row[1])/2;
          barcodes_vector[a][2] = (barcodes_vector[a][2] + row[2])/2;
          
          marker.id = a + 1;

        marker.pose.position.x = barcodes_vector[a][0];
        marker.pose.position.y = barcodes_vector[a][1];
        marker.pose.position.z = barcodes_vector[a][2];
        marker.pose.orientation.x = 0.0; //barcodes_vector[a - 1][3];
        marker.pose.orientation.y = 0.0; //barcodes_vector[a - 1][4];
        marker.pose.orientation.z = 0.0; //barcodes_vector[a - 1][5];
        marker.pose.orientation.w = 1.0; //barcodes_vector[a - 1][6];

        marker_pub.publish(marker);
        }
      }
      if(b==0) //New data
      {
        myfile << "no" << " ";
std::cout << "new"<<std::endl;
        barcodes_vector.push_back(row);
        bar_code_vector.push_back(stamp_time);

        // Publish marker
        marker.id = barcodes_vector.size();

        marker.pose.position.x = barcodes_vector[barcodes_vector.size() - 1][0];
        marker.pose.position.y = barcodes_vector[barcodes_vector.size() - 1][1];
        marker.pose.position.z = barcodes_vector[barcodes_vector.size() - 1][2];
        marker.pose.orientation.x = 0.0; //barcodes_vector[barcodes_vector.size() - 1][3];
        marker.pose.orientation.y = 0.0; //barcodes_vector[barcodes_vector.size() - 1][4];
        marker.pose.orientation.z = 0.0; //barcodes_vector[barcodes_vector.size() - 1][5];
        marker.pose.orientation.w = 1.0; //barcodes_vector[barcodes_vector.size() - 1][6];
 
        marker_pub.publish(marker);
      }

myfile << marker.id << " ";
myfile << stamp_time << " ";
myfile << marker.pose.position.x << " ";
myfile << marker.pose.position.y << " ";
myfile << marker.pose.position.z << "\n";
/*
myfile << row[3] << " ";
myfile << row[4] << " ";
myfile << row[5] << " ";
myfile << row[6] << "\n";
*/
    rate.sleep();
  }

myfile.close();

  return 0;
};
