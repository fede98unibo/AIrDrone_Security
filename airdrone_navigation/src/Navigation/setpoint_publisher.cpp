/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

std::mutex mutex_; //mutex_lock

//Define flight modes used in offboard control
enum Mode_t {position_ctrl, velocity_ctrl};
Mode_t Mode= position_ctrl;  //Control mode variable
TrajectorySetpoint setpoint{}; //setpoint variable

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") 
	{
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);
		new_setpoint_sub_ = 
			this->create_subscription<geometry_msgs::msg::Point>("NewSetpoint", 10, std::bind(&OffboardControl::new_setpoint_callback, this, _1));
		landing_setpoint_sub_ = 
			this->create_subscription<geometry_msgs::msg::Point>("NewVelocitySetpoint", 10, std::bind(&OffboardControl::new_velocity_setpoint_callback, this, _1));
		stop_offboard_sub_ = 
			this->create_subscription<std_msgs::msg::Bool>("StartStopOffboard", 10, std::bind(&OffboardControl::start_stop_offboard_, this, _1));
		timer_ = 
			this->create_wall_timer(50ms, std::bind(&OffboardControl::timer_callback, this));
  		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10, [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp); });

		offboard_setpoint_counter_ = 0;

		offboard_control_ = true;
	}

	void arm() const;
	void disarm() const;


	bool offboard_control_;      //!< start/stop offboard control

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr new_setpoint_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr landing_setpoint_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_offboard_sub_;


	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent


	void timer_callback();
	void publish_offboard_control_mode(std::string param) const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0) const;
    void new_setpoint_callback(const geometry_msgs::msg::Point::SharedPtr setpoint_msg) const;
	void new_velocity_setpoint_callback(const geometry_msgs::msg::Point::SharedPtr setpoint_msg) const;
	void start_stop_offboard_(const std_msgs::msg::Bool::SharedPtr msg);
};

/**
 * @brief Timer based loop to send setpoint to px4 flight controller
 */
 void OffboardControl::timer_callback()
 {
	 if(offboard_control_ == true)
	 {
		if (offboard_setpoint_counter_ == 10) 
			{
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm(); 
			}

					// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) { offboard_setpoint_counter_++; }
			
			switch(Mode)
			{
				case position_ctrl:
					publish_offboard_control_mode("position");
					publish_trajectory_setpoint();   
				break;

				case velocity_ctrl:
					publish_offboard_control_mode("velocity");
					publish_trajectory_setpoint();   
				break;
		
			}
	 }
 }


/**
 * @brief Start/Stop offboard control
 */
void OffboardControl::start_stop_offboard_(const std_msgs::msg::Bool::SharedPtr msg)
{
	offboard_control_ = msg->data;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode(std::string param) const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();

	if(param == "position")
	{
		msg.position = true;
		msg.velocity = false;
	}

	else if(param == "velocity")
	{
		msg.position = true;
		msg.velocity = true;
	}
	
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() const {

	setpoint.timestamp = timestamp_.load();
	
	trajectory_setpoint_publisher_->publish(setpoint);
}


/**
 * @brief Callback: receive new position setpoint 
 */ 
void OffboardControl::new_setpoint_callback(const geometry_msgs::msg::Point::SharedPtr setpoint_msg) const
{
     RCLCPP_INFO(this->get_logger(), "New Setpoint received!");
     
     //Update current trajectory setpoint
     setpoint.x = setpoint_msg -> x;
     setpoint.y = setpoint_msg -> y;
     setpoint.z = setpoint_msg -> z;

	const std::lock_guard<std::mutex> lock(mutex_);

	 Mode = position_ctrl;
}

/**
 * @brief Callback: receive new velocity setpoint 
 */ 
void OffboardControl::new_velocity_setpoint_callback(const geometry_msgs::msg::Point::SharedPtr setpoint_msg) const
{
	RCLCPP_INFO(this->get_logger(), "Velocity Setpoint received!");

	 setpoint.vx = setpoint_msg -> x;
     setpoint.vy = setpoint_msg -> y;
     setpoint.vz = setpoint_msg -> z;

	 const std::lock_guard<std::mutex> lock(mutex_); 

	 Mode = velocity_ctrl;
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) const 
{
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	
	//Set initial takeoff position
	setpoint.x = 0;
	setpoint.y = 0;
	setpoint.z = -3;
	//
	
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
