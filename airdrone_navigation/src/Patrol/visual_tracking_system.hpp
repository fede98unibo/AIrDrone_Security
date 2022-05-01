#define _USE_MATH_DEFINES

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <px4_msgs/msg/gimbal_manager_set_attitude.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

#define MARGIN_ERROR 0.10 // 10 deg error precision

using namespace std::chrono_literals;
using namespace Eigen;
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

      gimbal_attitude_pub_ = this->create_publisher<px4_msgs::msg::GimbalManagerSetAttitude>("fmu/gimbal_manager_set_attitude/in", 1);

      timer_ = this->create_wall_timer(100ms, std::bind(&VisualTracker::run, this));

      //Gimbal manager info
      gimbalAttitude_.target_system = 0;
      gimbalAttitude_.target_component = 0;
      gimbalAttitude_.gimbal_device_id = 0;

      //intrinsic camera matrix
      A << 100, 0, 320, 0, 100, 180, 0, 0, 1;
    }

    private:

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::GimbalManagerSetAttitude>::SharedPtr gimbal_attitude_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    trackerState_t trackerState_{TRACKING};

    std::atomic<uint64_t> timestamp_;

    Matrix<float,3,3>  A;
    Matrix<float,3,1>  boxPosition_;
    std::vector<vision_msgs::msg::Detection2D> detectionList_;

    px4_msgs::msg::GimbalManagerSetAttitude gimbalAttitude_;
    std::array<double,3> RPY_ = {0.0, 0.0, M_PI};
    std::array<double,3> gimbalSpeed_ = {0.0,0.0,0.0};
    tf2::Quaternion q_;
    const float dt{0.1}; //[s] = 1/timer_rate
    float t{0};

    //***CONTROLLER PARAMS***//
    const double K_p = 0.05;
    const double K_i = 0.2;
    std::chrono::time_point<std::chrono::high_resolution_clock> detTime_, last_detTime_;
    std::chrono::duration<double> det_interval;
    bool first_iter{true};

    void run_state_search();
    void run_state_tracking();

    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void run();

    void publish_gimbal_attitude(std::array<double,3> rpy, std::array<double,3> gimbal_speed);

};

