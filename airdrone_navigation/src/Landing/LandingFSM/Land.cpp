#include "Land.h"
#include "ConcreteLandState.h"
#include <iostream>

//using namespace std;
using std::placeholders::_1;
using namespace std::chrono;
using namespace std::chrono_literals;

Land::Land() : Node("Landing_FSM") 
{
	
	// public variable intialization
	start_landing_sequence = false;
	target_error = false;
	minimum_high = false;

	// FSM Initial State
	currentState = &Idle::getInstance();


	StartLanding_subscription_ =
		this->create_subscription<std_msgs::msg::Bool>("StartLanding_fsm", 1, std::bind(&Land::StartLanding, this, _1));

	TargetState_subscription_ =
		this->create_subscription<std_msgs::msg::Bool>("TargetState_fsm", 1, std::bind(&Land::TargetState_Handler, this, _1));

	SetpointReached_subscription_ =
		this->create_subscription<geometry_msgs::msg::Point>("SetPointReached", 1, std::bind(&Land::SetpointReached_Handler, this, _1));

	LandingPos_subscription_ =
		this->create_subscription<geometry_msgs::msg::Point>("landing_position", 1, std::bind(&Land::landing_position_Handler, this, _1));

	TargetRecognition_publisher_ = 
		this->create_publisher<std_msgs::msg::Bool>("ActivateTargetRecognition_fsm",10);

	LandingSetpoint_publisher_ = 
		this->create_publisher<geometry_msgs::msg::Point>("NewSetpoint",10);

	vehicle_command_publisher_ =
		this->create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic",10);

	StopFiltering_publisher_ = 
		this->create_publisher<std_msgs::msg::Bool>("StartStopOffboard",10);

}

void Land::setState(LandState& newState)
{
	currentState->exit(this);
	currentState = &newState;
	currentState->enter(this);	
}


void Land::toggle()
{
	currentState->toggle(this);
}

//*****  FSM INTERFACE: the following functions are used to receive notification, update param and alert other node about the state of the machine and the corrispondents actions ******/

/**
 * @brief Callback: Start/Stop the landing sequence
 */
void Land::StartLanding(const std_msgs::msg::Bool::SharedPtr msg)
{
	
	if(msg->data==true)
	{ 
		RCLCPP_INFO(this->get_logger(), "Starting Landing Sequence");
		start_landing_sequence = true;
	}

	else
	{
		RCLCPP_WARN(this->get_logger(), "Stopping Landing Sequence");
		start_landing_sequence = false;
	}

	this->toggle();
}

/**
 * @brief Callback: Update about the state of the Landing Target, if it is not visible shows "ERROR"
 */
 void Land::TargetState_Handler(const std_msgs::msg::Bool::SharedPtr msg)
 {
	
	if(msg->data == true) 
	 	{
			 RCLCPP_INFO(this->get_logger(), "Landing Target Aquired");
			 target_error=false;
		}
	
	else 
		{
			RCLCPP_ERROR(this->get_logger(), "Landing Target NOT Visible");
			target_error=true;
		}

    this-> toggle();
 }

/**
 * @brief Callback: when the minimu high during landing is reached it togle state
 */
void Land::landing_position_Handler(const geometry_msgs::msg::Point::SharedPtr msg)
{
	if((msg->z < 1.5) && (minimum_high==false))
	{
		RCLCPP_ERROR(this->get_logger(), "Minimum Position Reached");
		minimum_high=true;

		this-> toggle();
	}
}


/**
 * @brief Callback: When landing setpoint is reached it toglle state
 */
 void Land::SetpointReached_Handler(const geometry_msgs::msg::Point::SharedPtr msg)
 {
	if(start_landing_sequence == true)
	{
		RCLCPP_INFO(this->get_logger(), "Landing Setpoint Reached");

		this-> toggle();
	}
 }


/**
 * @brief Alert: it is used to send a flag to the "landing_target_detector" node in order to start searching target 
 */
void Land::activate_target_recognition()
{
	std_msgs::msg::Bool StartRecognition;
	StartRecognition.data = true;
	this->TargetRecognition_publisher_->publish(StartRecognition);
}


/**
 * @brief Alert: send landing position setpoint 
 */
void Land::publish_landing_setpoint(geometry_msgs::msg::Point landing_setpoint)
{
	this->LandingSetpoint_publisher_->publish(landing_setpoint);
}

/**
 * @brief Alert: send vehicle cmd
 */
void Land::publish_vehicle_command(int cmd) const 
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
	if(cmd == 0) 
	{
		msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND; 
	}
	else 
	{
		msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	}
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Alert: stop filtering and stop offboard control mode
 */
void Land::stop_filtering()
{
	std_msgs::msg::Bool stop;
	stop.data = false;
	this->StopFiltering_publisher_->publish(stop);
}


int main(int argc, char* argv[]) {

	std::cout << "Starting landing FSM node..." << std::endl;
	
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Land>());

	rclcpp::shutdown();
	return 0;
}
