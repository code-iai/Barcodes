// This node listens to the ROS image message topic "barcode/image",
// converts the sensor_msgs::Image to a HalconCpp::HImage,
// processes the HalconCpp::HImage and publishes the barcode number and 
// location on the "/data" topic.

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <halcon_image.h>
#include "HalconCpp.h"
#include "HDevThread.h"
#include <X11/Xlib.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace HalconCpp;

class ImageConverter
{
  private:
    ros::NodeHandle nh_;
    ros::Publisher data_pub;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    std::string bar_model;
    std::string internal_param;
    std::string external_param;
 
  public:
    // Constructor
    ImageConverter(std::string, std::string, std::string);

    // Destructor
    ~ImageConverter();
  
    // Functions
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void barcodeFinder(HObject image_to_process);

    // Halcon Functions
    void scaleImageRange (HObject ho_Image, HObject *ho_ImageScaled, HTuple hv_Min, HTuple hv_Max);
    void disp_message (HTuple hv_WindowHandle, HTuple hv_String, HTuple hv_CoordSystem, 
      HTuple hv_Row, HTuple hv_Column, HTuple hv_Color, HTuple hv_Box);
};

ImageConverter::ImageConverter(std::string _bar_model, std::string _internal_param, 
  std::string _external_param) : it_(nh_)
{
  // Subscribe and publish using "image_transport"
  image_sub_ = it_.subscribe("barcode/image", 1, &ImageConverter::imageCallback, this);
  image_pub_ = it_.advertise("barcode/results", 1);

  data_pub = nh_.advertise<std_msgs::String>("data", 1000);
  
  bar_model = _bar_model;
  internal_param = _internal_param;
  external_param = _external_param;
}

ImageConverter::~ImageConverter()
{
  //
}

void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Halcon local iconic variables
  HObject  ho_gray_image, ho_scaled_image;
  HObject  ho_ROI, ho_reduced_image;

  // Halcon local control variables
  HTuple  hv_ClipRegion, hv_bar_model;
  HTuple  hv_model_ID, hv_model_height, hv_file_name, hv_image_width;
  HTuple  hv_image_height, hv_row_center_model, hv_column_center_model;
  HTuple  hv_angle_model, hv_scale_model, hv_score_model;
    
  // Halcon image to process
  HImage *halcon_image;

  halcon_bridge::HalconImagePtr halcon_ptr;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    halcon_ptr = halcon_bridge::toHalconCopy(msg);
  }
  catch (halcon_bridge::Exception& e)
  {
    ROS_ERROR("halcon_bridge exception: %s", e.what());
    return;
  }
 
  // 'clip_region' property determines whether the regions of iconic objects will be clipped
  // to the currently used image size or not.
  GetSystem("clip_region", &hv_ClipRegion);
  SetSystem("clip_region", "true");

  // Process halcon_ptr->image using Halcon
  halcon_image = halcon_ptr->image;   
  GetImageSize(*halcon_image, &hv_image_width, &hv_image_height);
  
  // Adjust image contrast
  Rgb1ToGray(*halcon_image, &ho_gray_image);
  scaleImageRange(ho_gray_image, &ho_scaled_image, 50, 220);
  
  // Read bar model from file
  hv_bar_model = bar_model.c_str();
  ReadShapeModel(hv_bar_model, &hv_model_ID);
  // hv_model_height is the height of the bar model in pixels
  hv_model_height = 180;

  // Find bar model
  FindScaledShapeModel(ho_scaled_image, hv_model_ID, -0.2, 0.39, 0.6, 1.4, 0.8, 1, 0, "interpolation", 0,
    0.8, &hv_row_center_model, &hv_column_center_model, &hv_angle_model, &hv_scale_model, &hv_score_model);
  
  // If the bar model was found: search for the barcodes only in the region of interest (ROI),
  // else: search for the barcodes in the whole image
  if (hv_score_model>0)
  {
    // Generate a ROI according to the found model
    GenRectangle2(&ho_ROI, hv_row_center_model, hv_column_center_model, hv_angle_model, hv_image_width, 
      hv_model_height*hv_scale_model);
    ReduceDomain(ho_scaled_image, ho_ROI, &ho_reduced_image);
    
    // Search for barcodes in ROI
    barcodeFinder(ho_reduced_image);
  }
  else
  {
   // Search for barcodes in in the whole image
    ROS_WARN("Bar model not found");
    barcodeFinder(ho_scaled_image);
  }

  // Convert the Halcon image to a ROS image message and publish it
  image_pub_.publish(halcon_ptr->toImageMsg());
   
  SetSystem("clip_region", hv_ClipRegion);
}

void ImageConverter::barcodeFinder(HObject image_to_process)
{
  // Halcon local iconic variables
  HObject  ho_ROI, ho_symbol_regions, ho_cross;
	
  // Halcon local control variables
  HTuple  hv_internal_param, hv_external_param;
  HTuple  hv_internal_param_file, hv_external_param_file;
  HTuple  hv_image_to_process_height, hv_image_to_process_width, hv_window_handle;;
  HTuple  hv_cam_H_wcs, hv_pose_WCS, hv_wcs_H_cam;
  HTuple  hv_barcode_handle, hv_decoded_data, hv_region_area, hv_region_center_row;
  HTuple  hv_region_center_column, hv_region_width, hv_region_height;
  HTuple  hv_control_point_X, hv_control_point_Y, hv_control_point_Z;
  HTuple  hv_barcode_height, hv_image_point_up, hv_image_point_down;
  HTuple  hv_image_point_left, hv_image_point_right, hv_number_of_barcodes;
  HTuple  hv_i, hv_control_point_row, hv_control_point_column;
  HTuple  hv_pose_barcode, hv_pose_errors, hv_cam_H_obj;
  HTuple  hv_TransInX, hv_x;

   // Display image
  GetImageSize(image_to_process, &hv_image_to_process_width, &hv_image_to_process_height); //**************************** No la necesito?
  if (HDevWindowStack::IsOpen())
    CloseWindow(HDevWindowStack::Pop());
  ResetObjDb(hv_image_to_process_width, hv_image_to_process_height, 0); 
  SetWindowAttr("background_color","black");
  OpenWindow(0,0,800,800,0,"visible","",&hv_window_handle);
  HDevWindowStack::Push(hv_window_handle);
  DispObj(image_to_process, HDevWindowStack::GetActive());
    
  // Read calibration parameters from file
  hv_internal_param_file = internal_param.c_str();
  ReadCamPar(hv_internal_param_file, &hv_internal_param);
  hv_external_param_file = external_param.c_str();
  ReadPose(hv_external_param_file, &hv_external_param);
	
  // Modify the pose of the WCS, rotation based on homogenous transformation matrices
  PoseToHomMat3d(hv_external_param, &hv_cam_H_wcs);
  HomMat3dRotateLocal(hv_cam_H_wcs, HTuple(180).TupleRad(), "x", &hv_cam_H_wcs);
  HomMat3dToPose(hv_cam_H_wcs, &hv_pose_WCS);
  HomMat3dInvert(hv_cam_H_wcs, &hv_wcs_H_cam);
    
  // Find barcode
  CreateBarCodeModel(HTuple(), HTuple(), &hv_barcode_handle);
  FindBarCode(image_to_process, &ho_symbol_regions, hv_barcode_handle, "EAN-8", &hv_decoded_data);
  AreaCenter(ho_symbol_regions, &hv_region_area, &hv_region_center_row, &hv_region_center_column);
  RegionFeatures(ho_symbol_regions, "width", &hv_region_width);
  RegionFeatures(ho_symbol_regions, "height", &hv_region_height);
  ClearBarCodeModel(hv_barcode_handle);

  // Define the real barcode/rectangle-corners coordinates in meters (4 control points)
  hv_control_point_X = (((HTuple(1.5).Append(19)).Append(1.5)).Append(19))/1000.0;
  hv_control_point_Y = (((HTuple(6).Append(6)).Append(1)).Append(1))/1000.0;
  hv_control_point_Z = (((HTuple(0).Append(0)).Append(0)).Append(0));
  
  // Determine the image barcode/rectangle-corners coordinates in pixels
  // Variable hv_barcode_height represents the proportional height of the real barcode rectangle
  hv_barcode_height = hv_region_width/3.5;
  hv_image_point_up = hv_region_center_row-(hv_barcode_height/2);
  hv_image_point_down = hv_region_center_row+(hv_barcode_height/2);
  hv_image_point_left = hv_region_center_column-(hv_region_width/2);
  hv_image_point_right = hv_region_center_column+(hv_region_width/2);
  
  // Find pose of each barcode
  TupleLength(hv_decoded_data, &hv_number_of_barcodes);
  HTuple end_val = hv_number_of_barcodes-1;
  HTuple step_val = 1;  
  for (hv_i=0; hv_i.Continue(end_val, step_val); hv_i += step_val)
  {
    // Create two tuples with the values of the image barcode/rectangle-corners coordinates in pixels
    hv_control_point_row.Clear();
    hv_control_point_row.Append(HTuple(hv_image_point_up[hv_i]));
    hv_control_point_row.Append(HTuple(hv_image_point_up[hv_i]));
    hv_control_point_row.Append(HTuple(hv_image_point_down[hv_i]));
    hv_control_point_row.Append(HTuple(hv_image_point_down[hv_i]));
    hv_control_point_column.Clear();
    hv_control_point_column.Append(HTuple(hv_image_point_left[hv_i]));
    hv_control_point_column.Append(HTuple(hv_image_point_right[hv_i]));
    hv_control_point_column.Append(HTuple(hv_image_point_left[hv_i]));
    hv_control_point_column.Append(HTuple(hv_image_point_right[hv_i]));
      
    hv_TransInX = ("BARCODE "+(hv_i + 1));
    if (HDevWindowStack::IsOpen())
    {
      // Visualize image barcode/rectangle-corners points
      GenCrossContourXld(&ho_cross, hv_control_point_row, hv_control_point_column, 20, 0.785398);
      SetColor(HDevWindowStack::GetActive(),"green");
      DispObj(ho_cross, HDevWindowStack::GetActive());
      // Visualize barcode label
      hv_x = ((HTuple(hv_image_point_left[hv_i])*800)/hv_image_to_process_width).TupleInt();
      disp_message(hv_window_handle, hv_TransInX, "window", 12, hv_x, "green", "false");
    }
 
    // Determine the pose and homogeneous matrix of the barcode
    VectorToPose(hv_control_point_X, hv_control_point_Y, hv_control_point_Z, hv_control_point_row, 
        hv_control_point_column, hv_internal_param, "analytic", "error", &hv_pose_barcode, 
        &hv_pose_errors);
    PoseToHomMat3d(hv_pose_barcode, &hv_cam_H_obj);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO("%s: %s, Pose: x=%lf, y=%lf, z=%lf, X=%lf, Y=%lf, Z=%lf",
      hv_TransInX.S().Text(), hv_decoded_data[hv_i].S().Text(), hv_pose_barcode[0].D(),
      hv_pose_barcode[1].D(), hv_pose_barcode[2].D(), hv_pose_barcode[3].D(),
      hv_pose_barcode[4].D(), hv_pose_barcode[5].D());
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string barcode_info;

    HTuple  hv_x_trans, hv_y_trans, hv_z_trans;
    HTuple  hv_x_rot, hv_y_rot, hv_z_rot;

    TupleString(hv_pose_barcode[0]*100, ".3f", &hv_x_trans);
    TupleString(hv_pose_barcode[1]*100, ".3f", &hv_y_trans);
    TupleString(hv_pose_barcode[2]*100, ".3f", &hv_z_trans);
    TupleString(hv_pose_barcode[3], ".3f", &hv_x_rot);
    TupleString(hv_pose_barcode[4], ".3f", &hv_y_rot);
    TupleString(hv_pose_barcode[5], ".3f", &hv_z_rot);

    barcode_info = hv_TransInX.S().Text() + std::string(": ") +
      hv_decoded_data[hv_i].S().Text() + 
      std::string(" POSE: x: ") + hv_x_trans.S().Text() +
      std::string(" cm, y: ") + hv_y_trans.S().Text() +
      std::string(" cm, z: ") + hv_z_trans.S().Text() +
      std::string(" cm, X: ") + hv_x_rot.S().Text() +
      std::string("°, Y: ") + hv_y_rot.S().Text() +
      std::string("°, Z: ") + hv_z_rot.S().Text() + std::string("°");  

    std_msgs::String msg;
    msg.data = barcode_info;
    data_pub.publish(msg);                  ///////////////////____________-------------------------------
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


// Halcon function: This procedure writes a text message.
void ImageConverter::disp_message (HTuple hv_WindowHandle, HTuple hv_String, HTuple hv_CoordSystem, 
    HTuple hv_Row, HTuple hv_Column, HTuple hv_Color, HTuple hv_Box)
{

  // Local iconic variables

  // Local control variables
  HTuple  hv_GenParamName, hv_GenParamValue;

  //This procedure displays text in a graphics window.
  //
  //Input parameters:
  //WindowHandle: The WindowHandle of the graphics window, where
  //   the message should be displayed
  //String: A tuple of strings containing the text message to be displayed
  //CoordSystem: If set to 'window', the text position is given
  //   with respect to the window coordinate system.
  //   If set to 'image', image coordinates are used.
  //   (This may be useful in zoomed images.)
  //Row: The row coordinate of the desired text position
  //   A tuple of values is allowed to display text at different
  //   positions.
  //Column: The column coordinate of the desired text position
  //   A tuple of values is allowed to display text at different
  //   positions.
  //Color: defines the color of the text as string.
  //   If set to [], '' or 'auto' the currently set color is used.
  //   If a tuple of strings is passed, the colors are used cyclically...
  //   - if |Row| == |Column| == 1: for each new textline
  //   = else for each text position.
  //Box: If Box[0] is set to 'true', the text is written within an orange box.
  //     If set to' false', no box is displayed.
  //     If set to a color string (e.g. 'white', '#FF00CC', etc.),
  //       the text is written in a box of that color.
  //     An optional second value for Box (Box[1]) controls if a shadow is displayed:
  //       'true' -> display a shadow in a default color
  //       'false' -> display no shadow
  //       otherwise -> use given string as color string for the shadow color
  //
  //It is possible to display multiple text strings in a single call.
  //In this case, some restrictions apply:
  //- Multiple text positions can be defined by specifying a tuple
  //  with multiple Row and/or Column coordinates, i.e.:
  //  - |Row| == n, |Column| == n
  //  - |Row| == n, |Column| == 1
  //  - |Row| == 1, |Column| == n
  //- If |Row| == |Column| == 1,
  //  each element of String is display in a new textline.
  //- If multiple positions or specified, the number of Strings
  //  must match the number of positions, i.e.:
  //  - Either |String| == n (each string is displayed at the
  //                          corresponding position),
  //  - or     |String| == 1 (The string is displayed n times).
  //
  //
  //Convert the parameters for disp_text.
  if (0 != (HTuple(hv_Row==HTuple()).TupleOr(hv_Column==HTuple())))
  {
    return;
  }
  if (0 != (hv_Row==-1))
  {
    hv_Row = 12;
  }
  if (0 != (hv_Column==-1))
  {
    hv_Column = 12;
  }
  //
  //Convert the parameter Box to generic parameters.
  hv_GenParamName = HTuple();
  hv_GenParamValue = HTuple();
  if (0 != ((hv_Box.TupleLength())>0))
  {
    if (0 != (HTuple(hv_Box[0])==HTuple("false")))
    {
      //Display no box
      hv_GenParamName = hv_GenParamName.TupleConcat("box");
      hv_GenParamValue = hv_GenParamValue.TupleConcat("false");
    }
    else if (0 != (HTuple(hv_Box[0])!=HTuple("true")))
    {
      //Set a color other than the default.
      hv_GenParamName = hv_GenParamName.TupleConcat("box_color");
      hv_GenParamValue = hv_GenParamValue.TupleConcat(HTuple(hv_Box[0]));
    }
  }
  if (0 != ((hv_Box.TupleLength())>1))
  {
    if (0 != (HTuple(hv_Box[1])==HTuple("false")))
    {
      //Display no shadow.
      hv_GenParamName = hv_GenParamName.TupleConcat("shadow");
      hv_GenParamValue = hv_GenParamValue.TupleConcat("false");
    }
    else if (0 != (HTuple(hv_Box[1])!=HTuple("true")))
    {
      //Set a shadow color other than the default.
      hv_GenParamName = hv_GenParamName.TupleConcat("shadow_color");
      hv_GenParamValue = hv_GenParamValue.TupleConcat(HTuple(hv_Box[1]));
    }
  }
  //Restore default CoordSystem behavior.
  if (0 != (hv_CoordSystem!=HTuple("window")))
  {
    hv_CoordSystem = "image";
  }
  //
  if (0 != (hv_Color==HTuple("")))
  {
    //disp_text does not accept an empty string for Color.
    hv_Color = HTuple();
  }
  //
  DispText(hv_WindowHandle, hv_String, hv_CoordSystem, hv_Row, hv_Column, hv_Color, 
      hv_GenParamName, hv_GenParamValue);
  return;
}

int main(int argc, char **argv)
{
  int init_threads;
  std::string par_bar_model, par_internal_param, par_external_param;
  
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
      ImageConverter ic(par_bar_model, par_internal_param, par_external_param);
      ros::spin();
    }
  }
  else
  {
    ROS_ERROR("XInitThreads() initialization was not successful.");
  }
  return 0;
}
