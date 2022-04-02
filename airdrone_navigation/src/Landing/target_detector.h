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
        cv_bridge::CvImagePtr cv_ptr;

        //*** Private Attribute ***//
        bool simulation_;

        std::string image_path_;

        float target_dim_;
        vector<Point2f> landing_points_;

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
        vector<Point2f> find_target_points(Mat& img);
        vector<float> compute_pose(vector<Point2f> target_points);

        //*** Callbacks ***//

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        void timer_callback();
};
