#include <functional>
#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include "airdrone_actions/action/offboard_request.hpp"
#include "airdrone_actions/action/setpoint.hpp"
#include "airdrone_actions/srv/offboard_session.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_groundtruth.hpp>

#include "trajectory_generator.hpp"

#define TIMEOUT 500

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;


namespace offboard_control_server
{

enum serverState_t {OffState, IdleState, BusyState, ErrorState};

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
        lastTrajectoryUpdate_ = std::chrono::high_resolution_clock::now();
        currentPosition.x = 0;
        currentPosition.y = 0;
        currentPosition.z = 0;
        cmdAck_.result = -1;
        cmdAck_.command = -1;

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
        
        //update current position
        vehicle_position_sub = 
			this->create_subscription<VehicleLocalPosition>("fmu/vehicle_local_position/out", 10, 
                [this](const VehicleLocalPosition::UniquePtr msg){
                        currentPosition.x = msg->x;
                        currentPosition.y = msg->y;
                        currentPosition.z = msg->z;
                        
                        currentSpeed.x = msg->vx;
                        currentSpeed.y = msg->vy;
                        currentSpeed.z = msg->vz;
                });

        vehicle_attitude_sub_ = 
            this->create_subscription<VehicleAttitudeGroundtruth>("fmu/vehicle_attitude_groundtruth/out", 10, 
                [this](const VehicleAttitudeGroundtruth::UniquePtr msg){
                        std::cout << "attitude: " << msg->q[0] << std::endl;
                });

        command_ack_sub = 
        this->create_subscription<VehicleCommandAck>("fmu/vehicle_command_ack/out", 10, 
            [this](const VehicleCommandAck::UniquePtr msg){
                    cmdAck_.command = msg->command;
                    cmdAck_.result = msg->result;
            }); 

        trajectory_sub_ = 
        this->create_subscription<TrajectorySetpoint>("visual_tracker_trajectory", 1, 
            [this](const TrajectorySetpoint::UniquePtr msg){

                if(serverState == IdleState){
                    
                    speedcontrolActive = true;  //TODO: set a timeout to reset it to false... in the main loop
                    lastTrajectoryUpdate_ = std::chrono::high_resolution_clock::now();
                    
                    trajectory.vx = msg->vx;
                    trajectory.vy = msg->vy;
                    trajectory.vz = msg->vz;
                }
            });    
    }

private:

    /*** Server Private Attributes ***/
    serverState_t serverState{OffState};

	TrajectorySetpoint trajectory{};
    geometry_msgs::msg::Point currentPosition;
    geometry_msgs::msg::Point currentSpeed;
    geometry_msgs::msg::Point currentSetpoint;
    
    bool setpntReached{false};
    bool offboardActive{false};
    bool speedcontrolActive{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> lastTrajectoryUpdate_;
    VehicleCommandAck cmdAck_;

    /***/

    rclcpp_action::Server<Offboard>::SharedPtr action_server;
    rclcpp_action::Server<Setpoint>::SharedPtr setpoint_server;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleCommandAck>::SharedPtr command_ack_sub;
	rclcpp::Subscription<Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_sub;
    rclcpp::Subscription<VehicleAttitudeGroundtruth>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr trajectory_sub_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    TrajectoryGenerator TJ;   // Trajectory Generator;

    rclcpp_action::GoalResponse handle_offboard_request_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Offboard::Goal> goal);
    rclcpp_action::CancelResponse handle_offboard_request_cancel(const std::shared_ptr<GoalHandleOffboard> goal_handle);
    void handle_offboard_request_accepted(const std::shared_ptr<GoalHandleOffboard> goal_handle);
    void execute_offboard(const std::shared_ptr<GoalHandleOffboard> goal_handle);

    rclcpp_action::GoalResponse handle_setpoint_request_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Setpoint::Goal> goal);
    rclcpp_action::CancelResponse handle_setpoint_request_cancel(const std::shared_ptr<GoalHandleSetpoint> goal_handle);
    void handle_setpoint_request_accepted(const std::shared_ptr<GoalHandleSetpoint> goal_handle);
    void execute_setpoint_request(const std::shared_ptr<GoalHandleSetpoint> goal_handle);
    bool new_setpnt_received{false};

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
    
    //Auxiliary Functions
    vector<vector<float>> generate_polynomial(geometry_msgs::msg::Point dest, float time); //Deprecated
    int wait_for_command_ack(int command_ID, int timeout);
};

}

