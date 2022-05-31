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
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "airdrone_actions/action/setpoint.hpp"
#include "airdrone_actions/action/offboard_request.hpp"

using namespace std::chrono_literals;
using namespace Eigen;
using std::placeholders::_1;

enum trackerState_t {IDLE,SEARCH,TRACKING,ERROR};

class VisualTracker : public rclcpp::Node
{
    public:

    using Offboard = airdrone_actions::action::OffboardRequest;
    using GoalHandleOffboard = rclcpp_action::ClientGoalHandle<Offboard>;
    using Setpoint = airdrone_actions::action::Setpoint;
    using GoalHandleSetpoint = rclcpp_action::ClientGoalHandle<Setpoint>;

    VisualTracker();

    private:

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::GimbalManagerSetAttitude>::SharedPtr gimbal_attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<Offboard>::SharedPtr offboard_client_ptr_;
    rclcpp_action::Client<Setpoint>::SharedPtr setpoint_client_ptr_;

    trackerState_t trackerState_{SEARCH};

    std::atomic<uint64_t> timestamp_;

    Matrix<float,3,3>  A;
    Matrix<float,3,1>  boxPosition_;
    std::vector<vision_msgs::msg::Detection2D> detectionList_;

    px4_msgs::msg::GimbalManagerSetAttitude gimbalAttitude_;
    std::array<double,3> RPY_ = {0.0, 0.0, M_PI};
    std::array<double,3> gimbalSpeed_ = {0.0,0.0,0.0};
    tf2::Quaternion q_;

    //*** GIMBAL CONTROLLER PARAMS***//
    double K_p;
    double K_i;
    double MARGIN_ERROR;
    std::chrono::time_point<std::chrono::high_resolution_clock> detTime_, last_detTime_;
    std::chrono::duration<double> det_interval;
    bool first_iter{true};

    //*** TRACKING CONTROLLER PARAMS ***//
    double Kp_track;
    double Ki_track;
    double T_ti;
    double TARGET_SIZE; //[pixel]
    double TRACKING_ERROR;
    int TIMEOUT; //[ms]
    double tracking_altitude;
    px4_msgs::msg::VehicleLocalPosition vehicle_position;
    geometry_msgs::msg::Point commanded_speed;
    double saturation_error{0};
    double saturation_error_elevation{0};
    double max_speed{0};
    //int has_moved();

    void run_state_idle(){}
    void run_state_search();
    void run_state_tracking();
    void run_state_error();
    void run();

    void publish_gimbal_attitude(std::array<double,3> rpy, std::array<double,3> gimbal_speed);
    void check_controller_limits(double azimuth_error, double elevation_error);
    //void compute_target_position(px4_msgs::msg::VehicleLocalPosition& vehicle_position, 
                                  //px4_msgs::msg::VehicleLocalPosition& target_position);

    //*** OFFBOARD CLIENT ***//
    rclcpp_action::Client<Offboard>::SendGoalOptions send_goal_options_offboard;
    rclcpp_action::Client<Setpoint>::SendGoalOptions send_goal_options_setpoint;
    void offboard_response_callback(std::shared_future<GoalHandleOffboard::SharedPtr> future);
    void offboard_feedback_callback(GoalHandleOffboard::SharedPtr, const std::shared_ptr<const Offboard::Feedback> feedback);
    void offboard_result_callback(const GoalHandleOffboard::WrappedResult & result);
    bool offboard_active{false};
    void setpoint_response_callback(std::shared_future<GoalHandleSetpoint::SharedPtr> future);
    void setpoint_feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback> feedback);
    void setpoint_result_callback(const GoalHandleSetpoint::WrappedResult & result);

    // Callbacks
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

};


