#include <stdlib.h>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include <px4_msgs/msg/landing_target_pose.hpp>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>

#include "airdrone_actions/action/offboard.hpp"



//*** Defines ***//
#define threshold_value 120
#define min_rect_area 1500
#define square_form_factor 16
#define delta 2

using namespace std;
using namespace cv;
using std::placeholders::_1;


class LandingDetector : public rclcpp::Node
{
public:

    LandingDetector();

    ~LandingDetector() { cv::destroyWindow(OPENCV_WINDOW); }

    //*** Public Methods ***//

    vector<Point2f> find_target_points(Mat image);

    vector<float> compute_pose(vector<Point2f> target_points);

    int check_target_error(vector<Point2f> target_points);

    geometry_msgs::msg::Point current_position;

    vector<Point2f> frame_points;

private:

    const std::string OPENCV_WINDOW = "Landing Target window";

//*** Target and Camera specification ***//

    // Landing target 3D position, z=0 always
    const float sq_size = 30;
    vector<Point2f> landing_points = { Point2f(-sq_size,-sq_size), Point2f(-sq_size,sq_size), Point2f(sq_size,sq_size), Point2f(sq_size,-sq_size)};

    // Camera  intrinsic parameters and distorsion coefficents 
    Mat camera_matrix = ( Mat_<double>(3,3) << 277.191, 0, 160.5, 0, 277.191, 120.5, 0, 0, 1) ;               //For simulation
    Mat dist_coeff = ( Mat_<double>(5,1) << 0, 0, 0, 0, 0) ;
    //Mat camera_matrix = ( Mat_<double>(3,3) << 947.3496, 0, 271.247, 0, 984.41283, 236.9789, 0, 0, 1) ;    //For real world
    //Mat dist_coeff = ( Mat_<double>(5,1) << 0.037894, 0.83262, -0.0036415, -0.0061397, -5.379875) ;
    
//*** PUBLISHERs/SUBSCRIBERs ***//
    
    image_transport::Subscriber Image_sub_;
    rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

//*** Flags ***//
    bool contourDetected;

//*** Callbacks ***//
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void timer_callback();
};
