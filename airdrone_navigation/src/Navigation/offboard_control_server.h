#include <functional>
#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>

#include "airdrone_actions/action/offboard_request.hpp"
#include "airdrone_actions/action/setpoint.hpp"
#include "airdrone_actions/srv/offboard_session.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;


enum serverMode_t {MissionMode, LandingMode};
enum serverState_t {OffState, IdleState, BusyState, ErrorState};
enum controlType_t{CtrlPos, CtrlVel, CtrlPosVel}; 

namespace offboard_control_server
{

class OffboardServer : public rclcpp::Node
{
public:

    using Offboard = airdrone_actions::action::OffboardRequest;
    using GoalHandleOffboard = rclcpp_action::ServerGoalHandle<Offboard>;
    using Setpoint = airdrone_actions::action::Setpoint;
    using GoalHandleSetpoint = rclcpp_action::ServerGoalHandle<Setpoint>;

    explicit OffboardServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("offboard_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting Offboard Server");

        //Initilize private attibutes
        serverState = OffState;
        currentPosition.x = 0;
        currentPosition.y = 0;
        currentPosition.z = 0;
        setpntReached = false;

        using namespace std::placeholders;
        this->action_server = rclcpp_action::create_server<Offboard>(
            this,
            "offboard_request",
            std::bind(&OffboardServer::handle_offboard_request_goal, this, _1, _2),
            std::bind(&OffboardServer::handle_offboard_request_cancel, this, _1),
            std::bind(&OffboardServer::handle_offboard_request_accepted, this, _1));
        
        this->setpoint_server = rclcpp_action::create_server<Setpoint>(
            this,
            "setpoint_request",
            std::bind(&OffboardServer::handle_setpoint_request_goal, this, _1, _2),
            std::bind(&OffboardServer::handle_setpoint_request_cancel, this, _1),
            std::bind(&OffboardServer::handle_setpoint_request_accepted, this, _1));


		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});   
        vehicle_position_sub = 
			this->create_subscription<VehicleLocalPosition>("fmu/vehicle_local_position/out", 10, std::bind(&OffboardServer::local_position_callback, this, _1));
    }

private:

    /*** Server Private Attributes ***/
    serverMode_t serverMode;
    serverState_t serverState;

	TrajectorySetpoint trajectory{};
    geometry_msgs::msg::Point currentPosition;
    bool setpntReached;
    bool offboardActive;

    /***/

    rclcpp_action::Server<Offboard>::SharedPtr action_server;
    rclcpp_action::Server<Setpoint>::SharedPtr setpoint_server;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_sub;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    rclcpp_action::GoalResponse handle_offboard_request_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Offboard::Goal> goal);
    rclcpp_action::CancelResponse handle_offboard_request_cancel(const std::shared_ptr<GoalHandleOffboard> goal_handle);
    void handle_offboard_request_accepted(const std::shared_ptr<GoalHandleOffboard> goal_handle);
    void execute_offboard(const std::shared_ptr<GoalHandleOffboard> goal_handle);

    rclcpp_action::GoalResponse handle_setpoint_request_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Setpoint::Goal> goal);
    rclcpp_action::CancelResponse handle_setpoint_request_cancel(const std::shared_ptr<GoalHandleSetpoint> goal_handle);
    void handle_setpoint_request_accepted(const std::shared_ptr<GoalHandleSetpoint> goal_handle);
    void execute_setpoint_request(const std::shared_ptr<GoalHandleSetpoint> goal_handle);


	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
    void local_position_callback(VehicleLocalPosition::SharedPtr local_position);

    vector<vector<float>> generate_polynomial(geometry_msgs::msg::Point dest, float time);

};


void OffboardServer::local_position_callback(VehicleLocalPosition::SharedPtr local_position)
{
    currentPosition.x = local_position->x;
    currentPosition.y = local_position->y;
    currentPosition.z = local_position->z;
}

}

