#ifndef TARGET_DETECTOR_H
#define TARGET_DETECTOR_H

//#include <stdlib.h>
#include <functional>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/bool.hpp"
#include <px4_msgs/msg/landing_target_pose.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

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

#include "kalman_filter.h"

//using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using std::placeholders::_1;


class TargetDetector : public rclcpp::Node
{
    public:

        TargetDetector();

    private:
        //*** Private Attribute ***//
        
        //*** PUBLISHERs/SUBSCRIBERs ***//
        image_transport::Subscriber Image_sub_;
        rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr position_pub_;
        rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr detection_pub_;
        cv_bridge::CvImagePtr cv_ptr;

        //std::shared_ptr<KalmanFilter>;    // KALMAN
        KalmanFilter kalman;
        // counter used to count the number of frames without detection
        unsigned int no_detection_count = 0;
        bool detection_timed_out = false;

        bool simulation_;

        std::string image_path_;

        float target_dim_;
        std::vector<Point2f> landing_points_;

        Mat camera_matrix_;
        Mat dist_coeff_;

        // SURF Parameters
        Mat img_object_;
        int minHessian_;
        float ratio_thresh_;
        int minGoodMatch_;
        Ptr<SURF> detector_;
        Ptr<DescriptorMatcher> matcher_;
        std::vector<KeyPoint> keypoints_object_, keypoints_scene_;
        Mat descriptors_object_, descriptors_scene_;
        
        //*** Private Methods ***//

        int Init();
        std::vector<Point2f> find_target_points(Mat& img);
        std::vector<float> compute_pose(std::vector<Point2f> target_points);
        vision_msgs::msg::BoundingBox2D compute_bbox_info(std::vector<Point2f> detected_points);
        std::vector<Point2f> points_from_bbox(vision_msgs::msg::BoundingBox2D box); // compute vector of 4 vertexes points from a bounding box
        void display_points_lines(std::vector<Point2f> points, int R, int G, int B); // draw points and lines connecting them given a vector of points and RGB color

        //*** Callbacks ***//

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        void timer_callback();
};

#endif