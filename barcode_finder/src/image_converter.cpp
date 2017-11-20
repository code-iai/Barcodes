// This node listens to the ROS image message topic "barcode/image",
// converts the sensor_msgs::Image to a HalconCpp::HImage and processes it.
// The node reads from file a "Bar model" which was created in order to
// identify the bar where the labels are located.
// The node finds the barcodes and publishes the number and 
// location on the "/barcode/data" topic.

#include <ros/ros.h>
#include "std_msgs/String.h"

// Includes everything to publish and subscribe to images
#include <image_transport/image_transport.h> 

//Halcon libraries 
#include <halcon_image.h>
#include "HalconCpp.h"

// When working under UNIX/Linux it is necessary to turn on the support for
// multithreading in the Xlib in order to correctly use the Halcon graphics.
#include <X11/Xlib.h>
#include "HDevThread.h"

// Allow to load images using OpenCV and convert it to the ROS message format
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Messages from the REFILLS project
#include <refills_msgs/Barcode.h>


using namespace HalconCpp;

class ImageConverter
{
  private:
    ros::NodeHandle nh_;  
    ros::Publisher data_pub;                // Publisher for barcode information
    ros::Publisher barcode_pose_pub;        // Publisher for the estimated pose of the barcodes
    image_transport::ImageTransport it_;    // Create an ImageTransport instance
    image_transport::Subscriber image_sub_; // Subscriber to images
    image_transport::Publisher image_pub_;
    std::string bar_model;                  // .shm file from Param. Server
    HTuple internal_param;             // .cal file from Param. Server
    HTuple external_param;             // Pose file from Param. Server
  public:
    // Constructor
    ImageConverter(std::string, HTuple, HTuple);

    // Destructor
    ~ImageConverter();
  
    // Functions
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void converter(halcon_bridge::HalconImagePtr halcon_ptr);
    void barcodeFinder(HImage image_to_process, HTuple image_width, HTuple image_height, halcon_bridge::HalconImagePtr halcon_ptr);

    // Functions exported from Halcon
    void scaleImageRange (HObject ho_Image, HObject *ho_ImageScaled,
      HTuple hv_Min, HTuple hv_Max);
};

// Constructor is initialized with the 3 parameters in the Param. Server 
ImageConverter::ImageConverter(std::string _bar_model,
  HTuple _internal_param, HTuple _external_param) : it_(nh_)
{
  // Initialize class members
  bar_model = _bar_model;
  internal_param = _internal_param;
  external_param = _external_param;

  // Subscribe to images using "image_transport"
  //image_sub_ = it_.subscribe("barcode/image", 1,
    //&ImageConverter::imageCallback, this);
  image_sub_ = it_.subscribe("/refills_wrist_camera/image_mono", 1,
    &ImageConverter::imageCallback, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 5);
  // Advertise string data will be published
  data_pub = nh_.advertise<std_msgs::String>("barcode/data", 1);
  barcode_pose_pub = nh_.advertise<refills_msgs::Barcode>("barcode/pose", 5);
}


ImageConverter::~ImageConverter()
{
  //
}


// This function receives the published image and processes it
void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
//cambiar-----------------------------------------------------------------------------------
cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//------------------------------------------------------------------------------------


  // Convert the ROS image message to a Halcon image
  halcon_bridge::HalconImagePtr halcon_pointer;
  try
  {
    halcon_pointer = halcon_bridge::toHalconCopy(cv_ptr->toImageMsg());
    //halcon_pointer = halcon_bridge::toHalconCopy(msg);
  }
  catch (halcon_bridge::Exception& e)
  {
    ROS_ERROR("halcon_bridge exception: %s", e.what());
    return;
  }
  converter(halcon_pointer);
}


void ImageConverter::converter(halcon_bridge::HalconImagePtr halcon_ptr)
{ 
  // Halcon local iconic variables
  HObject  ho_gray_image;         // Graysaled and contrast-adjusted images
  HObject  ho_ROI;                // ROI according to "Bar model"

  HImage *halcon_image;                             // Halcon image to process  
  HImage ho_reduced_image, ho_scaled_image;
  
  // Halcon local control variables
  HTuple  hv_image_width, hv_image_height;                 // Image width and height  
  HTuple  hv_ClipRegion;                                   // 'clip_region' property
  HTuple  hv_bar_model, hv_model_ID, hv_model_height;      // Bar model properties
  HTuple  hv_row_center_model, hv_column_center_model;     // Bar model properties
  HTuple  hv_angle_model, hv_scale_model, hv_score_model;  // Bar model properties

try
  {
  // 'clip_region' property determines whether the regions of iconic objects
  // will be clipped to the currently used image size or not.
  GetSystem("clip_region", &hv_ClipRegion);
  SetSystem("clip_region", "false");

  // Process halcon_ptr->image using Halcon Software
  halcon_image = halcon_ptr->image; 
  GetImageSize(*halcon_image, &hv_image_width, &hv_image_height);
  
  // Adjust image contrast
  scaleImageRange(*halcon_image, &ho_scaled_image, 10, 220); 

  // Read "Bar model" from parameter in Param. Server
  hv_bar_model = bar_model.c_str();
  ReadShapeModel(hv_bar_model, &hv_model_ID);
  // hv_model_height is the height of the "Bar model" in pixels (180)
  hv_model_height = 180;
  // Find "Bar model" in image
  FindScaledShapeModel(ho_scaled_image, hv_model_ID, -0.2, 0.39, 0.6, 1.4, 0.8,
    1, 0, "interpolation", 0, 0.8, &hv_row_center_model, &hv_column_center_model,
    &hv_angle_model, &hv_scale_model, &hv_score_model);
}
catch (HException &except) {
    std::cout << "Exception while running Callback" << std::endl;
    std::cout << except.ErrorMessage() << std::endl;
  }  

  // If "Bar model" was found: search for the barcodes only in the ROI
  if (hv_score_model > 0)
  {
    // Generate a ROI according to the found model
    GenRectangle2(&ho_ROI, hv_row_center_model, hv_column_center_model,
      hv_angle_model, hv_image_width, hv_model_height*hv_scale_model);
   ReduceDomain(ho_scaled_image, ho_ROI, &ho_reduced_image);
    
    // Search for barcodes in the reduced image according to the ROI
    barcodeFinder(ho_reduced_image,hv_image_width, hv_image_height, halcon_ptr);
  }
  // Else: search for the barcodes in the whole image
  else
  {
    //ROS_WARN("Bar model not found");
    barcodeFinder(ho_scaled_image, hv_image_width, hv_image_height, halcon_ptr);
  }
  SetSystem("clip_region", hv_ClipRegion);
}

// This function finds the barcodes
void ImageConverter::barcodeFinder(HImage image_to_process, HTuple image_width, HTuple image_height, halcon_bridge::HalconImagePtr halcon_ptr) {

  // Halcon local control variables
  HTuple hv_window_handle;                               // Window to display image
  HTuple hv_pose_WCS;                                    // Pose of WCS
  HTuple hv_cam_H_wcs, hv_wcs_H_cam, hv_cam_H_obj;       // Homogeneous matrices
  HTuple hv_decoded_data;                                // Barcode number
  HTuple hv_region_area, hv_region_center_row;           // Barcode properties
  HTuple hv_region_center_column;                        // Barcode properties
  HTuple hv_region_width, hv_region_height;              // Barcode properties
  HTuple hv_control_point_X, hv_control_point_Y;         // Real barcode coordinates
  HTuple hv_control_point_Z;                             // Real barcode coordinates
  HTuple hv_barcode_height;                              // Proportional height
  HTuple hv_image_point_up, hv_image_point_down;         // Image barcode coordinates
  HTuple hv_image_point_left, hv_image_point_right;      // Image barcode coordinates
  HTuple hv_control_point_row, hv_control_point_column;  // Image barcode coordinates
  HTuple hv_pose_barcode, hv_pose_errors;                // Barcode pose
  HTuple hv_barcode, hv_barcode_label;                   // Barcode labels
  HTuple hv_x_trans, hv_y_trans, hv_z_trans;             // Barcode pose (Translation)
  HTuple hv_x_rot, hv_y_rot, hv_z_rot;                   // Barcode pose (Rotation)

  HImage image_regions;                                  // Image with barcode regions
  HBarCode hv_barcode_handle;                            // Barcode model ID
  HRegion ho_symbol_regions;                             // Region where the barcode is located 
  int number_barcodes;                                   // Number of barcodes in image
  std::string barcode_info;                              // Barcode information
  std_msgs::String msg;                                  // Message to publish 

  HTuple grayvalue(120.0, 120.0);

  try {
    // Modify the pose of the WCS, rotation based on homogenous transformation matrices
    PoseToHomMat3d(external_param, &hv_cam_H_wcs);
    HomMat3dRotateLocal(hv_cam_H_wcs, HTuple(180).TupleRad(), "x", &hv_cam_H_wcs);
    HomMat3dToPose(hv_cam_H_wcs, &hv_pose_WCS);
    HomMat3dInvert(hv_cam_H_wcs, &hv_wcs_H_cam);

    // Find barcode and get its properties
    hv_barcode_handle.CreateBarCodeModel(HTuple(), HTuple());
    ho_symbol_regions = image_to_process.FindBarCode(hv_barcode_handle, "EAN-8", &hv_decoded_data);
    number_barcodes = 0;
    number_barcodes = hv_decoded_data.Length();
    AreaCenter(ho_symbol_regions, &hv_region_area, &hv_region_center_row, &hv_region_center_column);
    RegionFeatures(ho_symbol_regions, "width", &hv_region_width);
    RegionFeatures(ho_symbol_regions, "height", &hv_region_height);
    //hv_barcode_handle->ClearBarCodeModel();

    // Define the real barcode/rectangle-corners coordinates in meters
    // (4 control points)
    hv_control_point_X = (((HTuple(1.5).Append(19)).Append(1.5)).Append(19)) / 1000.0;
    hv_control_point_Y = (((HTuple(6).Append(6)).Append(1)).Append(1)) / 1000.0;
    hv_control_point_Z = (((HTuple(0).Append(0)).Append(0)).Append(0));

    // Determine the image barcode/rectangle-corners coordinates in pixels.
    // Variable hv_barcode_height represents the proportional height
    // of the real barcode rectangle.
    hv_barcode_height = hv_region_width / 3.5;
    hv_image_point_up = hv_region_center_row - (hv_barcode_height / 2);
    hv_image_point_down = hv_region_center_row + (hv_barcode_height / 2);
    hv_image_point_left = hv_region_center_column - (hv_region_width / 2);
    hv_image_point_right = hv_region_center_column + (hv_region_width / 2);

    // Find pose of each barcode
    for (int i = 0; i < number_barcodes; i++) 
    {
      hv_barcode = ("BARCODE " + (i + 1));

      // Create two tuples with the values of the
      // image barcode/rectangle-corners coordinates in pixels
      hv_control_point_row.Clear();
      hv_control_point_row.Append(HTuple(hv_image_point_up[i]));
      hv_control_point_row.Append(HTuple(hv_image_point_up[i]));
      hv_control_point_row.Append(HTuple(hv_image_point_down[i]));
      hv_control_point_row.Append(HTuple(hv_image_point_down[i]));
      hv_control_point_column.Clear();
      hv_control_point_column.Append(HTuple(hv_image_point_left[i]));
      hv_control_point_column.Append(HTuple(hv_image_point_right[i]));
      hv_control_point_column.Append(HTuple(hv_image_point_left[i]));
      hv_control_point_column.Append(HTuple(hv_image_point_right[i]));

      // Determine pose and homogeneous matrix of the barcode
      VectorToPose(hv_control_point_X, hv_control_point_Y, hv_control_point_Z,
                   hv_control_point_row, hv_control_point_column, internal_param,
                   "analytic", "error", &hv_pose_barcode, &hv_pose_errors);
      PoseToHomMat3d(hv_pose_barcode, &hv_cam_H_obj);

      // Convert Barcode Pose to string
      TupleString(hv_pose_barcode[0] * 100, ".3f", &hv_x_trans);
      TupleString(hv_pose_barcode[1] * 100, ".3f", &hv_y_trans);
      TupleString(hv_pose_barcode[2] * 100, ".3f", &hv_z_trans);
      TupleString(hv_pose_barcode[3], ".3f", &hv_x_rot);
      TupleString(hv_pose_barcode[4], ".3f", &hv_y_rot);
      TupleString(hv_pose_barcode[5], ".3f", &hv_z_rot);
      barcode_info = hv_barcode.S().Text() + std::string(": ") +
                     hv_decoded_data[i].S().Text() +
                     std::string(" POSE: x: ") + hv_x_trans.S().Text() +
                     std::string(" cm, y: ") + hv_y_trans.S().Text() +
                     std::string(" cm, z: ") + hv_z_trans.S().Text() +
                     std::string(" cm, X: ") + hv_x_rot.S().Text() +
                     std::string("°, Y: ") + hv_y_rot.S().Text() +
                     std::string("°, Z: ") + hv_z_rot.S().Text() + std::string("°");

      // Publish the barcode pose
      msg.data = barcode_info;
      data_pub.publish(msg);

      refills_msgs::Barcode barcode_msg;
      barcode_msg.barcode = hv_decoded_data[i].S().Text();
      barcode_msg.barcode_pose.header.frame_id = halcon_ptr->header.frame_id;

      barcode_msg.barcode_pose.header.stamp = halcon_ptr->header.stamp;
      barcode_msg.barcode_pose.pose.position.x = hv_pose_barcode[0];
      barcode_msg.barcode_pose.pose.position.y = hv_pose_barcode[1];
      barcode_msg.barcode_pose.pose.position.z = hv_pose_barcode[2];
      barcode_msg.barcode_pose.pose.orientation.x = 0.0; //FIXME, convert halcon hv_rot to quaternion
      barcode_msg.barcode_pose.pose.orientation.y = 0.0;
      barcode_msg.barcode_pose.pose.orientation.z = 0.0;
      barcode_msg.barcode_pose.pose.orientation.w = 1.0;
      barcode_pose_pub.publish(barcode_msg);
    }

    // Publish image over ROS
    halcon_bridge::HalconImage *halcon_img_ptr(new halcon_bridge::HalconImage);
    halcon_img_ptr->header.frame_id = halcon_ptr->header.frame_id;
    halcon_img_ptr->header.stamp = halcon_ptr->header.stamp;
    halcon_img_ptr->encoding = halcon_ptr->encoding;
    if (number_barcodes>0)
    {
      image_regions = image_to_process.PaintRegion(ho_symbol_regions, grayvalue, HString("fill"));
      halcon_img_ptr->image = &image_regions;
    }
    else
    { 
      halcon_img_ptr->image = &image_to_process;
    }
    image_pub_.publish(halcon_img_ptr->toImageMsg());

  } catch (HException &except) {
    std::cout << "Exception while running FindBarCode" << std::endl;
    std::cout << except.ErrorMessage() << std::endl;
  }
}

// Halcon function: Scale the gray values of an image from the interval [Min,Max] to [0,255]
void ImageConverter::scaleImageRange (HObject ho_Image, HObject *ho_ImageScaled, HTuple hv_Min, 
    HTuple hv_Max)
{
  // Local iconic variables
  HObject  ho_SelectedChannel, ho_LowerRegion, ho_UpperRegion;

  // Local control variables
  HTuple  hv_LowerLimit, hv_UpperLimit, hv_Mult;
  HTuple  hv_Add, hv_Channels, hv_Index, hv_MinGray, hv_MaxGray;
  HTuple  hv_Range;

  //Convenience procedure to scale the gray values of the
  //input image Image from the interval [Min,Max]
  //to the interval [0,255] (default).
  //Gray values < 0 or > 255 (after scaling) are clipped.
  //
  //If the image shall be scaled to an interval different from [0,255],
  //this can be achieved by passing tuples with 2 values [From, To]
  //as Min and Max.
  //Example:
  //scale_image_range(Image:ImageScaled:[100,50],[200,250])
  //maps the gray values of Image from the interval [100,200] to [50,250].
  //All other gray values will be clipped.
  //
  //input parameters:
  //Image: the input image
  //Min: the minimum gray value which will be mapped to 0
  //     If a tuple with two values is given, the first value will
  //     be mapped to the second value.
  //Max: The maximum gray value which will be mapped to 255
  //     If a tuple with two values is given, the first value will
  //     be mapped to the second value.
  //
  //output parameter:
  //ImageScale: the resulting scaled image
  //
  if (0 != ((hv_Min.TupleLength())==2))
  {
    hv_LowerLimit = ((const HTuple&)hv_Min)[1];
    hv_Min = ((const HTuple&)hv_Min)[0];
  }
  else
  {
    hv_LowerLimit = 0.0;
  }
  if (0 != ((hv_Max.TupleLength())==2))
  {
    hv_UpperLimit = ((const HTuple&)hv_Max)[1];
    hv_Max = ((const HTuple&)hv_Max)[0];
  }
  else
  {
    hv_UpperLimit = 255.0;
  }
  //
  //Calculate scaling parameters
  hv_Mult = ((hv_UpperLimit-hv_LowerLimit).TupleReal())/(hv_Max-hv_Min);
  hv_Add = ((-hv_Mult)*hv_Min)+hv_LowerLimit;
  //
  //Scale image
  ScaleImage(ho_Image, &ho_Image, hv_Mult, hv_Add);
  //
  //Clip gray values if necessary
  //This must be done for each channel separately
  CountChannels(ho_Image, &hv_Channels);
  {
  HTuple end_val48 = hv_Channels;
  HTuple step_val48 = 1;
  for (hv_Index=1; hv_Index.Continue(end_val48, step_val48); hv_Index += step_val48)
  {
    AccessChannel(ho_Image, &ho_SelectedChannel, hv_Index);
    MinMaxGray(ho_SelectedChannel, ho_SelectedChannel, 0, &hv_MinGray, &hv_MaxGray, 
      &hv_Range);
    Threshold(ho_SelectedChannel, &ho_LowerRegion, (hv_MinGray.TupleConcat(hv_LowerLimit)).TupleMin(), 
      hv_LowerLimit);
    Threshold(ho_SelectedChannel, &ho_UpperRegion, hv_UpperLimit,
      (hv_UpperLimit.TupleConcat(hv_MaxGray)).TupleMax());
    PaintRegion(ho_LowerRegion, ho_SelectedChannel, &ho_SelectedChannel, hv_LowerLimit, 
      "fill");
    PaintRegion(ho_UpperRegion, ho_SelectedChannel, &ho_SelectedChannel, hv_UpperLimit, 
      "fill");
    if (0 != (hv_Index==1))
    {
      CopyObj(ho_SelectedChannel, &(*ho_ImageScaled), 1, 1);
    }
    else
    {
      AppendChannel((*ho_ImageScaled), ho_SelectedChannel, &(*ho_ImageScaled));
    }
  }
  }
  return;
}

int main(int argc, char **argv)
{
  int init_threads;
  std::string par_bar_model, par_internal_param, par_external_param;
  HTuple hv_internal_param, hv_external_param;               // Camera parameters
  HTuple hv_internal_param_file, hv_external_param_file;     // Camera parameters files
  
  // When working under UNIX/Linux it is necessary to turn on the support for
  // multithreading in the Xlib in order to correctly use the Halcon graphics.
  // This is achieved by calling the function XInitThreads() before any other
  // function of the Xlib library.
  if ((init_threads = XInitThreads()) =! 0)
  {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh("~");

    // Get Parameters from Parameter Server
    if (!nh.getParam("bar_model_file", par_bar_model))
    {
      ROS_ERROR("Could not find parameter 'bar_model_file' in namespace '%s'",
        nh.getNamespace().c_str());
      return 0;
    }
    else if (!nh.getParam("internal_param_file", par_internal_param))
    {
      ROS_ERROR("Could not find parameter 'internal_param_file' in namespace '%s'",
        nh.getNamespace().c_str());
      return 0;
    }
    else if (!nh.getParam("external_param_file", par_external_param))
    {
      ROS_ERROR("Could not find parameter 'external_param_file' in namespace '%s'",
        nh.getNamespace().c_str());
      return 0;
    }
    else 
    {
      { 
        // Read calibration parameters from file
        hv_internal_param_file = par_internal_param.c_str();
        ReadCamPar(hv_internal_param_file, &hv_internal_param);
        hv_external_param_file = par_external_param.c_str();
        ReadPose(hv_external_param_file, &hv_external_param);
 
        //ImageConverter ic(par_bar_model, par_internal_param, par_external_param);
        ImageConverter ic(par_bar_model, hv_internal_param, hv_external_param);
        ros::spin();
      }    
    }
  }
  else
  {
    ROS_ERROR("XInitThreads() initialization was not successful.");
  }
  return 0;
}
