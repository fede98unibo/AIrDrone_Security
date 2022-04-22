#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("fake_camera");

    image_transport::Publisher image_pub_;
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;

    image_pub_ = image_transport::create_publisher(node.get(),"/image_raw",custom_qos);

    cv::VideoCapture cap("udpsrc port=5600 ! application/x-rtp ! rtpjitterbuffer ! rtph264depay ! avdec_h264! videoconvert ! videoscale ! appsink",cv::CAP_GSTREAMER);

    if(!cap.isOpened())
    {
        std::cout<<"VideoCapture not opened"<<std::endl;
        exit(-1);
    }

    cv::Mat frame;
    cv_bridge::CvImage img_bridge;
    auto img_msg = sensor_msgs::msg::Image();
    std_msgs::msg::Header header;

    while(true) {

        cap.read(frame);

        std::cout << frame.size() << std::endl;

        if(frame.empty())
            break;
        
        header.stamp = rclcpp::Time();
        img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::RGB8, frame);
        img_bridge.toImageMsg(img_msg);
        image_pub_.publish(img_msg);

        std::chrono::nanoseconds();
        rclcpp::spin_some(node);
    }
    destroyWindow("Receiver");
    rclcpp::shutdown();
    return 0;
}
