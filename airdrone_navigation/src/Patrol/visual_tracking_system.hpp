#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/gimbal_manager_set_attitude.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


enum trackerState_t {IDLE,SEARCH,TRACKING,STABILIZE};

class VisualTracker : public rclcpp::Node
{
    public:

    VisualTracker()
    : Node("visual_tracker")
    {
      detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
                                "/detector_node/detections", 1, std::bind(&VisualTracker::detection_callback, this, _1));
      
      // get common timestamp
      timesync_sub_ =
        this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
          [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
            timestamp_.store(msg->timestamp);
          });

      gimbal_attitude_pub_ = this->create_publisher<GimbalManagerSetAttitude>("fmu/gimbal_manager_set_attitude/in", 1);

      timer_ = this->create_wall_timer(500ms, std::bind(&VisualTracker::run, this));
    }

    private:

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Publisher<GimbalManagerSetAttitude>::SharedPtr gimbal_attitude_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    trackerState_t trackerState_{SEARCH};

    std::atomic<uint64_t> timestamp_;

    bool isTracking_{false};
    int numDetectedObj_{0};
    std::vector<vision_msgs::msg::Detection2D> detectionList_;

    px4_msgs::msg::GimbalManagerSetAttitude gimbalAttitude_;

    void run_state_search();
    void run_state_tracking();
    void run_state_stabilize();

    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) const;
    void run();

};

