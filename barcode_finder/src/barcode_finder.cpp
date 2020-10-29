// This node listens to the ROS image message topic "barcode/image",
// converts the sensor_msgs::Image to a HalconCpp::HImage and processes it.
// The node reads from file a "Shelf model" which was created in order to
// identify the shelf where the labels are located.

#include <ros/ros.h>
#include "std_msgs/String.h"

// Includes everything to publish and subscribe to images
#include <image_transport/image_transport.h>

//Halcon libraries 
#include <asr_halcon_bridge/halcon_image.h>
#include "HalconCpp.h"


//Messages from the REFILLS project
#include <refills_msgs/Barcode.h>

// Displaying Markers in Rviz
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <fstream>

//For usleep
#include <unistd.h>


using namespace HalconCpp;



struct HCBarcodeResult
{
  HalconCpp::HRegion regions_;
  HalconCpp::HTuple strings_;
  HalconCpp::HTuple orientations_;

  uint8_t num_of_barcodes_found_;
  std::vector<tf::Stamped<tf::Pose>> poses_;

  void reset()
  {
    num_of_barcodes_found_ = 0;
    regions_.Clear();
    strings_.Clear();
    poses_.clear();
  }
};

struct BarcodeRegion {
    sensor_msgs::RegionOfInterest roi;
    std::string code;
    double orientation;
};


class ImageConverter {
private:
    ros::NodeHandle nh_;
    ros::Publisher barcode_pose_pub;        // Publisher for the estimated pose of the barcodes
    ros::Publisher marker_pub;              // Publisher for barcode markers
    image_transport::ImageTransport it_;    // Create an ImageTransport instance
    image_transport::Subscriber image_sub_; // Subscriber to images
    image_transport::Publisher image_pub_;  // Publish images
    std::string bar_model;                  // .shm file from Param. Server
    HTuple internal_param;                  // .cal file from Param. Server
    HTuple external_param;                  // Pose file from Param. Server
    int id;                                 // Marker id
    HTuple hv_bar_model, hv_model_ID; //Hold the bar model

    HCBarcodeResult barcodes_;
    std::vector<BarcodeRegion> barcode_regions_;
public:
    // Constructor
    ImageConverter(std::string, HTuple, HTuple);

    // Destructor
    ~ImageConverter();

    // Functions    
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    void shelfFinder(halcon_bridge::HalconImagePtr halcon_ptr);

    void barcodeFinder(HImage image_to_process, HTuple image_width,
                       HTuple image_height, halcon_bridge::HalconImagePtr halcon_ptr);

    // Functions exported from Halcon
    void scaleImageRange(HObject ho_Image, HObject *ho_ImageScaled,
                         HTuple hv_Min, HTuple hv_Max);
};

// Constructor is initialized with the 3 parameters in the Param. Server 
ImageConverter::ImageConverter(std::string _bar_model,
                               HTuple _internal_param, HTuple _external_param) : it_(nh_) {
    // Initialize class members
    bar_model = _bar_model;
    internal_param = _internal_param;
    external_param = _external_param;

    // Subscribe to and publish images using "image_transport"
    image_sub_ = it_.subscribe("image_in",
                               1,  // This topic is named "/barcode/image" in the report
                               &ImageConverter::imageCallback, this);
    image_pub_ = it_.advertise("/barcode/output_images", 5);
    // Publish barcode information using the refills message
    barcode_pose_pub = nh_.advertise<refills_msgs::Barcode>("barcode/pose", 5);
    //Publish markers in rviz
    marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("barcode/markers", 1);


    // Read "Shelf model" from parameter in Param. Server
    hv_bar_model = bar_model.c_str();
    ReadShapeModel(hv_bar_model, &hv_model_ID);

}


ImageConverter::~ImageConverter() {
    //
}


// This function receives the published image and converts it to Halcon image
void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // Convert ROS image messages to Halcon images
    halcon_bridge::HalconImagePtr halcon_pointer;
    try {
        halcon_pointer = halcon_bridge::toHalconCopy(msg);
    }
    catch (halcon_bridge::Exception &e) {
        ROS_ERROR("halcon_bridge exception: %s", e.what());
        return;
    }

    shelfFinder(halcon_pointer);
}

// THis function finds the Shelf Model
void ImageConverter::shelfFinder(halcon_bridge::HalconImagePtr halcon_ptr) {
    // Halcon local iconic variables
    HObject ho_gray_image;         // Grayscaled and contrast-adjusted images
    HObject ho_ROI;                // ROI according to "Bar model"

    HImage *halcon_image;           // Halcon image to process
    HImage ho_reduced_image, ho_scaled_image;

    // Halcon local control variables
    HTuple hv_image_width, hv_image_height;                 // Image width and height
    HTuple hv_ClipRegion;                                   // 'clip_region' property
    HTuple hv_model_height;      // Bar model properties
    HTuple hv_row_center_model, hv_column_center_model;     // Bar model properties
    HTuple hv_angle_model, hv_scale_model, hv_score_model;  // Bar model properties

    try {
        // 'clip_region' property determines whether the regions of iconic objects
        // will be clipped to the currently used image size or not.
        GetSystem("clip_region", &hv_ClipRegion);
        SetSystem("clip_region", "false");


        // Process halcon_ptr->image using Halcon Software
        halcon_image = halcon_ptr->image;
        GetImageSize(*halcon_image, &hv_image_width, &hv_image_height);

        // Adjust image contrast
        //scaleImageRange(*halcon_image, &ho_scaled_image, 10, 180);
        ho_scaled_image = HObject(*halcon_image);

        // hv_model_height is the height of the "Shelf model" in pixels (180)
        //hv_model_height = 180;
        // Find "Shelf model" in image
        //FindScaledShapeModel(ho_scaled_image, hv_model_ID, -0.2, 0.39, 0.9, 1.1, 0.76,
        //  1, 0, "interpolation", 0, 0.8, &hv_row_center_model, &hv_column_center_model,
        //  &hv_angle_model, &hv_scale_model, &hv_score_model);
    }
    catch (HException &except) {
        std::cout << "Exception while running Callback" << std::endl;
        std::cout << except.ErrorMessage() << std::endl;
    }

    // If "Shelf model" was found: search for the barcodes only in the ROI
    //if (hv_score_model > 0)
    if (false) {
        // Generate a ROI according to the found model
        GenRectangle2(&ho_ROI, hv_row_center_model, hv_column_center_model,
                      hv_angle_model, hv_image_width, hv_model_height * hv_scale_model);
        ReduceDomain(ho_scaled_image, ho_ROI, &ho_reduced_image);

        // Search for barcodes in the reduced image according to the ROI
        barcodeFinder(ho_reduced_image, hv_image_width, hv_image_height, halcon_ptr);
    }
        // Else: search for the barcodes in the whole image
    else {
        barcodeFinder(ho_scaled_image, hv_image_width, hv_image_height, halcon_ptr);
    }
    SetSystem("clip_region", hv_ClipRegion);
}

// This function finds the barcodes
void ImageConverter::barcodeFinder(HImage image_to_process, HTuple image_width, HTuple image_height,
                                   halcon_bridge::HalconImagePtr halcon_ptr) {

    try {
        HalconCpp::HBarCode barcode_handle;
        barcode_handle.CreateBarCodeModel(HalconCpp::HTuple(), HalconCpp::HTuple());
        barcodes_.num_of_barcodes_found_ = 0;      
                   
        barcodes_.regions_ = image_to_process.FindBarCode(barcode_handle, "EAN-8", &barcodes_.strings_);
        barcodes_.orientations_ = barcode_handle.GetBarCodeResult("all", "orientation");       
        // will this assumption ever break? can we find more than 255 barcodes in a single image?;
        barcodes_.num_of_barcodes_found_ = static_cast<uint8_t>(barcodes_.strings_.Length());
 

        
        HalconCpp::HTuple hv_region_center_rows, hv_region_center_cols;
        barcodes_.regions_.AreaCenter(&hv_region_center_rows, &hv_region_center_cols);  // centers of the barcode regions

        // get the heights of the detected barcodes
        HalconCpp::HTuple region_widths = barcodes_.regions_.RegionFeatures(HalconCpp::HTuple("width"));
        HalconCpp::HTuple region_heights = barcodes_.regions_.RegionFeatures(HalconCpp::HTuple("height"));

        // get the corners of the detected barcodes
        HalconCpp::HTuple region_upper_left_row = barcodes_.regions_.RegionFeatures(HalconCpp::HTuple("row1"));
        HalconCpp::HTuple region_upper_left_col = barcodes_.regions_.RegionFeatures(HalconCpp::HTuple("column1"));
        HalconCpp::HTuple region_bottom_right_row = barcodes_.regions_.RegionFeatures(HalconCpp::HTuple("row2"));
        HalconCpp::HTuple region_bottom_right_col = barcodes_.regions_.RegionFeatures(HalconCpp::HTuple("column2"));

        double true_bc_ratio = 4.5/17.0;


        /// corrdinates estimated by vectorToPose will be relative to the 0,0,0 given here, which is the top left corner in
        /// the image; depending on oruentation of the camera (mono_0 is 180 deg flipped) this corresponds to the top right
        /// corner of the barcode
        HalconCpp::HTuple world_x = (((HalconCpp::HTuple(0).Append(17.0)).Append(0)).Append(17.0)) / 1000.0;
        HalconCpp::HTuple world_y = (((HalconCpp::HTuple(0).Append(0)).Append(4.5)).Append(4.5)) / 1000.0;
        HalconCpp::HTuple world_z = (((HalconCpp::HTuple(0).Append(0)).Append(0)).Append(0));
 
        visualization_msgs::MarkerArray markers;
        for (int i = 0; i < barcodes_.num_of_barcodes_found_; ++i) {
            // TODO: if width < height -> portrait otherwise different
            ROS_DEBUG_STREAM("Barcode: " << i);
            ROS_DEBUG_STREAM("Width: " << region_widths[i].D());
            ROS_DEBUG_STREAM("Height: " << region_heights[i].D());

            double adapted_width = region_heights[i].D() * true_bc_ratio;
            ROS_DEBUG_STREAM("Adapted width: " << adapted_width);

            ROS_DEBUG_STREAM("Ratio: " << region_widths[i].D() / region_heights[i].D());
            ROS_DEBUG_STREAM("Adapted Ratio: " << adapted_width / region_heights[i].D());

            //Extracting the barcode region values for price label ocr
            BarcodeRegion br;
            sensor_msgs::RegionOfInterest roi = sensor_msgs::RegionOfInterest();
            roi.width = region_widths[i].D();
            roi.height = region_heights[i].D();
            roi.x_offset = region_upper_left_col[i].D();
            roi.y_offset = region_upper_left_row[i].D();
            br.roi = roi;
            br.code = barcodes_.strings_[i].S();
            br.orientation = barcodes_.orientations_[i].D();
            barcode_regions_.push_back(br); 
         
            HalconCpp::HTuple image_cols, image_rows;
            /*if (barcodes_.orientations_[i].D() >= 0) {
                image_cols = ((HalconCpp::HTuple(region_upper_left_col[i]).Append(region_upper_left_col[i]))
                            .Append(region_upper_left_col[i] + adapted_width)).Append(region_upper_left_col[i].D() + adapted_width);
                image_rows = ((HalconCpp::HTuple(region_bottom_right_row[i]).Append(region_upper_left_row[i])).Append(region_bottom_right_row[i])).Append(region_upper_left_row[i]);
            }
            else {
                image_cols = ((HalconCpp::HTuple(region_upper_left_col[i] + adapted_width).Append(region_upper_left_col[i] + adapted_width)).Append(region_upper_left_col[i])).Append(region_upper_left_col[i]);
                image_rows = ((HalconCpp::HTuple(region_upper_left_row[i]).Append(region_bottom_right_row[i])).Append(region_upper_left_row[i])).Append(region_bottom_right_row[i]);
            }*/
            image_cols = ((HalconCpp::HTuple(region_bottom_right_col[i]).Append(region_upper_left_col[i]))
                            .Append(region_bottom_right_col[i])).Append(region_upper_left_col[i]);
            image_rows = ((HalconCpp::HTuple(region_bottom_right_row[i]).Append(region_bottom_right_row[i])).Append(region_bottom_right_row[i]+adapted_width)).Append(region_bottom_right_row[i]+adapted_width);

            HalconCpp::HTuple pose_errors, ho_pose;

            HalconCpp::VectorToPose(world_x, world_y, world_z, image_rows, image_cols, internal_param, "planar_analytic", "error", &ho_pose, &pose_errors);

            ROS_DEBUG_STREAM(ho_pose[0].D() << " " << ho_pose[1].D() << " " << ho_pose[2].D());
            tf::Stamped<tf::Pose> pose_stamped;
            pose_stamped.setOrigin(tf::Vector3(ho_pose[0].D(), ho_pose[1].D(), ho_pose[2].D()));
            tf::Quaternion q = tf::createQuaternionFromRPY((ho_pose[3].D() * M_PI / 180), (ho_pose[4].D() * M_PI / 180),
                                                  (ho_pose[5].D() * M_PI / 180));
            pose_stamped.setRotation(q);
            pose_stamped.stamp_ = halcon_ptr->header.stamp;
            pose_stamped.frame_id_ = halcon_ptr->header.frame_id;
            barcodes_.poses_.push_back(pose_stamped);
        
            refills_msgs::Barcode barcode_msg;
            barcode_msg.barcode = br.code;
            tf::poseStampedTFToMsg(pose_stamped, barcode_msg.barcode_pose);
            barcode_pose_pub.publish(barcode_msg);

            // Set marker parameters and publish


            visualization_msgs::Marker marker;
            marker.ns = "/barcode_detector";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.lifetime = ros::Duration(1);
            marker.header.stamp = halcon_ptr->header.stamp;
            marker.header.frame_id = halcon_ptr->header.frame_id;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.017;
            marker.scale.y = 0.0045;
            marker.scale.z = 0.01;
            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;

            marker.pose.position = barcode_msg.barcode_pose.pose.position;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            markers.markers.push_back(marker);

            // Create a tf broadcaster for the detected barcode
            static tf::TransformBroadcaster tf_br;
            tf::StampedTransform transform;
            transform.setOrigin(tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
            transform.setRotation(q);
            transform.stamp_ = halcon_ptr->header.stamp;
            tf_br.sendTransform(tf::StampedTransform(transform, halcon_ptr->header.stamp, halcon_ptr->header.frame_id,
                                                  "b" + barcode_msg.barcode));

        }//foir


        marker_pub.publish(markers);
    }//try
     catch (HException &except) {
         std::cout << "Exception while running barcodeFinder" << std::endl;
         std::cout << except.ErrorMessage() << std::endl;
    
    }

    try {

        HalconCpp::HImage image_regions;
        HalconCpp::HTuple greyvalue(120.0, 120.0);
        if (barcodes_.num_of_barcodes_found_ != 0)
          image_regions = halcon_ptr->image->PaintRegion(barcodes_.regions_, greyvalue, HalconCpp::HString("fill")); 
        else 
           image_regions = *halcon_ptr->image;

        // Publish image over ROS
        halcon_bridge::HalconImage *halcon_img_ptr(new halcon_bridge::HalconImage);
        halcon_img_ptr->header.frame_id = halcon_ptr->header.frame_id;
        halcon_img_ptr->header.stamp = halcon_ptr->header.stamp;
        halcon_img_ptr->encoding = halcon_ptr->encoding;
        halcon_img_ptr->image = &image_regions;
        image_pub_.publish(halcon_img_ptr->toImageMsg());

    }
    catch (HException &except) {
        std::cout << "Exception while converting to Halcon Image" << std::endl;
        std::cout << except.ErrorMessage() << std::endl;
    }
}


// Halcon function: Scale the gray values of an image from the interval [Min,Max] to [0,255]
void ImageConverter::scaleImageRange(HObject ho_Image, HObject *ho_ImageScaled, HTuple hv_Min,
                                     HTuple hv_Max) {
    // Local iconic variables
    HObject ho_SelectedChannel, ho_LowerRegion, ho_UpperRegion;

    // Local control variables
    HTuple hv_LowerLimit, hv_UpperLimit, hv_Mult;
    HTuple hv_Add, hv_Channels, hv_Index, hv_MinGray, hv_MaxGray;
    HTuple hv_Range;

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
    if (0 != ((hv_Min.TupleLength()) == 2)) {
        hv_LowerLimit = ((const HTuple &) hv_Min)[1];
        hv_Min = ((const HTuple &) hv_Min)[0];
    } else {
        hv_LowerLimit = 0.0;
    }
    if (0 != ((hv_Max.TupleLength()) == 2)) {
        hv_UpperLimit = ((const HTuple &) hv_Max)[1];
        hv_Max = ((const HTuple &) hv_Max)[0];
    } else {
        hv_UpperLimit = 255.0;
    }
    //
    //Calculate scaling parameters
    hv_Mult = ((hv_UpperLimit - hv_LowerLimit).TupleReal()) / (hv_Max - hv_Min);
    hv_Add = ((-hv_Mult) * hv_Min) + hv_LowerLimit;
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
        for (hv_Index = 1; hv_Index.Continue(end_val48, step_val48); hv_Index += step_val48) {
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
            if (0 != (hv_Index == 1)) {
                CopyObj(ho_SelectedChannel, &(*ho_ImageScaled), 1, 1);
            } else {
                AppendChannel((*ho_ImageScaled), ho_SelectedChannel, &(*ho_ImageScaled));
            }
        }
    }
    return;
}

int main(int argc, char **argv) {

    usleep(2000000);


    int init_threads;
    std::string par_bar_model, par_internal_param, par_external_param;
    HTuple hv_internal_param, hv_external_param;               // Camera parameters
    HTuple hv_internal_param_file, hv_external_param_file;     // Camera parameters files


    if (true) {
        ros::init(argc, argv, "barcode_finder");
        ros::NodeHandle nh("~");

        // Get Parameters from Parameter Server
        if (!nh.getParam("bar_model_file", par_bar_model)) {
            ROS_ERROR("Could not find parameter 'bar_model_file' in namespace '%s'",
                      nh.getNamespace().c_str());
            return 0;
        } else if (!nh.getParam("internal_param_file", par_internal_param)) {
            ROS_ERROR("Could not find parameter 'internal_param_file' in namespace '%s'",
                      nh.getNamespace().c_str());
            return 0;
        } else if (!nh.getParam("external_param_file", par_external_param)) {
            ROS_ERROR("Could not find parameter 'external_param_file' in namespace '%s'",
                      nh.getNamespace().c_str());
            return 0;
        } else {
            // Read calibration parameters from file
            hv_internal_param_file = par_internal_param.c_str();
            ReadCamPar(hv_internal_param_file, &hv_internal_param);
            hv_external_param_file = par_external_param.c_str();
            ReadPose(hv_external_param_file, &hv_external_param);

            ImageConverter ic(par_bar_model, hv_internal_param, hv_external_param);
            ros::spin();
        }
    }

    return 0;
}
