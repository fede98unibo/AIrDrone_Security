#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include "LandState.h"
#include "ConcreteLandState.h"

#include <chrono>
#include <px4_msgs/msg/timesync.hpp>



class LandState;

class Land : public rclcpp::Node
{
public:

	Land();

	/*** GET Methods ***/

	inline LandState* getCurrentState() const { return currentState; }
	
	/*** SET Methods ***/

	void setState(LandState& newState);
	
	/***/

	void toggle();

	void activate_target_recognition();

	void stop_filtering();

	void publish_landing_setpoint(geometry_msgs::msg::Point landing_setpoint);

	void publish_vehicle_command(int cmd) const;

	bool start_landing_sequence;

	bool target_error;

	bool minimum_high;
	
private:

	LandState* currentState;

	// Publisher/Subscriber definitions
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr StartLanding_subscription_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr TargetState_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr SetpointReached_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr LandingPos_subscription_;

	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr TargetRecognition_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr LandingSetpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr StopFiltering_publisher_;
	
	// Callbacks Prototypes
	void StartLanding(const std_msgs::msg::Bool::SharedPtr msg);
	void TargetState_Handler(const std_msgs::msg::Bool::SharedPtr msg);
	void SetpointReached_Handler(const geometry_msgs::msg::Point::SharedPtr msg);
	void landing_position_Handler(const geometry_msgs::msg::Point::SharedPtr msg);

};


