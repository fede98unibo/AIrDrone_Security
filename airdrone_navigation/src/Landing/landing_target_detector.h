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


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Defines
#define threshold_value 150
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

private:


    const std::string OPENCV_WINDOW = "Landing Target window";
    
    //Flags 
    bool contourDetected;
    int errorCounter=0;
    bool StartStopRecognition;
    bool LandingTargetAquired;

//*** Target and Camera specification ***//

    //Landing target 3D position, z=0 always
    float sq_size = 37.5;
    std::vector<Point2f> landing_points = { Point2f(-sq_size,-sq_size), Point2f(-sq_size,sq_size), Point2f(sq_size,sq_size), Point2f(sq_size,-sq_size)};

    //Camera parameters

    //Mat camera_matrix = ( Mat_<double>(3,3) << 947.3496, 0, 271.247, 0, 984.41283, 236.9789, 0, 0, 1) ;    //For real world
    //Mat dist_coeff = ( Mat_<double>(5,1) << 0.037894, 0.83262, -0.0036415, -0.0061397, -5.379875) ;
    Mat camera_matrix = ( Mat_<double>(3,3) << 277.191, 0, 160.5, 0, 277.191, 120.5, 0, 0, 1) ;               //For simulation
    Mat dist_coeff = ( Mat_<double>(5,1) << 0, 0, 0, 0, 0) ;

    //** PUBLISHER/SUBSCRIBER **//
    
    image_transport::Subscriber Image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr StartStopRecognition_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //** Callbacks **//
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void timer_callback();
    void StartRecognition(const std_msgs::msg::Bool::SharedPtr msg);
};