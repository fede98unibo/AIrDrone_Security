#define _USE_MATH_DEFINES

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <future>
#include <sstream>
#include <unistd.h>
#include "rclcpp/rate.hpp"
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <px4_msgs/msg/gimbal_manager_set_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "airdrone_actions/action/setpoint.hpp"
#include "airdrone_actions/action/offboard_request.hpp"

#define MARGIN_ERROR 0.05 // 10 deg error precision

using namespace std::chrono_literals;
using namespace Eigen;
using std::placeholders::_1;


enum trackerState_t {IDLE,SEARCH,TRACKING,STABILIZE};

class VisualTracker : public rclcpp::Node
{
    public:

    using Offboard = airdrone_actions::action::OffboardRequest;
    using GoalHandleOffboard = rclcpp_action::ClientGoalHandle<Offboard>;
    using Setpoint = airdrone_actions::action::Setpoint;
    using GoalHandleSetpoint = rclcpp_action::ClientGoalHandle<Setpoint>;

    VisualTracker()
    : Node("visual_tracker")
    {
      // yolo detection subscriber
      detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
                                "/detector_node/detections", 1, std::bind(&VisualTracker::detection_callback, this, _1));

      //local position subscriber
      local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("fmu/vehicle_local_position/out", 1, 
          [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
                  vehiclePosition_.x = msg->x;
                  vehiclePosition_.y = msg->y;
                  vehiclePosition_.z = msg->z;
          });

      // common timestamp subscriber
      timesync_sub_ =
        this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
          [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
            timestamp_.store(msg->timestamp);
          });

      // gimbal attitude publisher
      gimbal_attitude_pub_ = this->create_publisher<px4_msgs::msg::GimbalManagerSetAttitude>("fmu/gimbal_manager_set_attitude/in", 1);

      // timer -> Finite State Machine
      timer_ = this->create_wall_timer(100ms, std::bind(&VisualTracker::run, this));

      // offboard and setpoint clients
      offboard_client_ptr_ = rclcpp_action::create_client<Offboard>(this,"offboard_request");
      setpoint_client_ptr_ = rclcpp_action::create_client<Setpoint>(this,"setpoint_request");

      //Init send goal options
      send_goal_options_offboard = rclcpp_action::Client<Offboard>::SendGoalOptions();
      send_goal_options_setpoint = rclcpp_action::Client<Setpoint>::SendGoalOptions();
      
      using namespace std::placeholders;

      send_goal_options_offboard.goal_response_callback =
        std::bind(&VisualTracker::offboard_response_callback, this, _1);
      send_goal_options_offboard.feedback_callback =
        std::bind(&VisualTracker::offboard_feedback_callback, this, _1, _2);
      send_goal_options_offboard.result_callback =
        std::bind(&VisualTracker::offboard_result_callback, this, _1);

      send_goal_options_setpoint.goal_response_callback =
        std::bind(&VisualTracker::setpoint_response_callback, this, _1);
      send_goal_options_setpoint.feedback_callback =
        std::bind(&VisualTracker::setpoint_feedback_callback, this, _1, _2);
      send_goal_options_setpoint.result_callback =
        std::bind(&VisualTracker::setpoint_result_callback, this, _1);

      //Gimbal manager info
      gimbalAttitude_.target_system = 0;
      gimbalAttitude_.target_component = 0;
      gimbalAttitude_.gimbal_device_id = 0;

      //intrinsic camera matrix
      A << 100, 0, 320, 0, 100, 180, 0, 0, 1;
    }

    private:

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::GimbalManagerSetAttitude>::SharedPtr gimbal_attitude_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<Offboard>::SharedPtr offboard_client_ptr_;
    rclcpp_action::Client<Setpoint>::SharedPtr setpoint_client_ptr_;


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

    //*** GIMBAL CONTROLLER PARAMS***//
    const double K_p = 0.05;
    const double K_i = 0.2;
    std::chrono::time_point<std::chrono::high_resolution_clock> detTime_, last_detTime_;
    std::chrono::duration<double> det_interval;
    bool first_iter{true};

    //*** TARGET TRACKING PARAMS ***//
    px4_msgs::msg::VehicleLocalPosition vehiclePosition_;
    px4_msgs::msg::VehicleLocalPosition targetPosition_;
    px4_msgs::msg::VehicleLocalPosition last_targetPosition_;
    int has_moved();

    void run_state_search();
    void run_state_tracking();
    void run();

    void publish_gimbal_attitude(std::array<double,3> rpy, std::array<double,3> gimbal_speed);
    void check_controller_limits(double azimuth_error, double elevation_error);
    void compute_target_position(px4_msgs::msg::VehicleLocalPosition& vehicle_position, 
                                  px4_msgs::msg::VehicleLocalPosition& target_position);

    //*** OFFBOARD CLIENT ***//
    rclcpp_action::Client<Offboard>::SendGoalOptions send_goal_options_offboard;
    rclcpp_action::Client<Setpoint>::SendGoalOptions send_goal_options_setpoint;
    void offboard_response_callback(std::shared_future<GoalHandleOffboard::SharedPtr> future);
    void offboard_feedback_callback(GoalHandleOffboard::SharedPtr, const std::shared_ptr<const Offboard::Feedback> feedback);
    void offboard_result_callback(const GoalHandleOffboard::WrappedResult & result);
    void setpoint_response_callback(std::shared_future<GoalHandleSetpoint::SharedPtr> future);
    void setpoint_feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback> feedback);
    void setpoint_result_callback(const GoalHandleSetpoint::WrappedResult & result);
    bool offboard_active{false};
    long int good_detections{0};
    
    //Callbacks
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

};

