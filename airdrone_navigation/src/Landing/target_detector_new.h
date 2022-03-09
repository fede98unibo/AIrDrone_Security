#include <stdlib.h>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include <px4_msgs/msg/landing_target_pose.hpp>

#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using std::placeholders::_1;


class TargetDetector : public rclcpp::Node
{
    public:

        TargetDetector();

    private:

        //*** PUBLISHERs/SUBSCRIBERs ***//
    
        image_transport::Subscriber Image_sub_;
        rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr position_pub_;

        //*** Private Attribute ***//

        const std::string OPENCV_WINDOW = "Landing Target window";
        std::string image_path;

        // Landing target 3D position, z=0 always
        float sq_size = 30;
        vector<Point2f> landing_points = { Point2f(sq_size,-sq_size), Point2f(sq_size,sq_size), Point2f(-sq_size,sq_size), Point2f(-sq_size,-sq_size)};
        //vector<Point2f> landing_points = { Point2f(9,-7), Point2f(9,7), Point2f(-9,7), Point2f(-9,-7)};
        // Camera  intrinsic parameters and distorsion coefficents 
        Mat camera_matrix = ( Mat_<double>(3,3) << 277.191, 0, 160.5, 0, 277.191, 120.5, 0, 0, 1) ;               //For simulation
        Mat dist_coeff = ( Mat_<double>(5,1) << 0, 0, 0, 0, 0) ;
        //Mat camera_matrix = ( Mat_<double>(3,3) << 947.3496, 0, 271.247, 0, 984.41283, 236.9789, 0, 0, 1) ;    //For real world
        //Mat dist_coeff = ( Mat_<double>(5,1) << 0.037894, 0.83262, -0.0036415, -0.0061397, -5.379875) ;
        vector<Point2f> frame_points;
        geometry_msgs::msg::Point current_position;

        // SURF Parameters
        Mat img_object;
        int minHessian;
        float ratio_thresh;
        int minGoodMatch;
        Ptr<SURF> detector;
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        
        //*** Private Methods ***//

        int Init();
        vector<Point2f> find_target_points(Mat img);
        vector<float> compute_pose(vector<Point2f> target_points);

        //*** Callbacks ***//

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        void timer_callback();
};
