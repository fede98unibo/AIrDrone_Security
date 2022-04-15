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
#include <px4_msgs/msg/mission_result.hpp>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <stdint.h>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <iostream>
#include <cmath>

//Define control modes used in offboard control
enum Mode_t {position_ctrl, velocity_ctrl};

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class OffboardManager : public rclcpp::Node {
public:
	OffboardManager() : Node("offboard_manager") 
	{
        timer_ = 
			this->create_wall_timer(50ms, std::bind(&OffboardManager::timer_callback, this));

        vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);

        offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);

        trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);

		mission_result_sub = 
			this->create_subscription<MissionResult>("MissionResult_PubSubTopic", 10, std::bind(&OffboardManager::mission_info_callaback, this, _1));
  		
          // get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10, [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp); });

        // Flags init
        instance_count = 0;
        seq_reached = -1;
        seq_current = 0;
        seq_total = 0;
        mission_finished = false;
        offboard_control = false;
        Mode_t Mode= position_ctrl;

        setpoint.x = 0;
        setpoint.y = 0;
        setpoint.z = 0;
	}

    // Reset flags value evry time a new mission is uploaded
    void reset_flags(MissionResult::SharedPtr mission_reset_info);

	//Flags
    bool offboard_control;      //!< start/stop offboard control

    TrajectorySetpoint setpoint{}; //setpoint variable

    Mode_t Mode;

    uint32_t instance_count;

    int32_t seq_reached;

    uint16_t seq_current;

    uint16_t seq_total;

    bool mission_finished;


private:  
    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<MissionResult>::SharedPtr mission_result_sub;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    // Callback for checking mission status
    void mission_info_callaback(MissionResult::SharedPtr mission_info_msg);

    //Timer callback used to perform offboard
    void timer_callback();

    //Advertizer used to publish Vehicle_command_msgs to px4-flight controller
    void publish_vehicle_command(uint16_t command, float param1, float param2) const;

    // Choose velocity or position control during offboard
    void publish_offboard_control_mode(std::string param) const;

    // Publish trajectory setpoint in NED to px4-flight controller
    void publish_trajectory_setpoint() ;


};


void OffboardManager::mission_info_callaback(MissionResult::SharedPtr mission_info_msg)
{
    if(mission_info_msg->instance_count != instance_count)
    {
        std::cout<<" New Mission Uploaded " << std::endl;

        reset_flags(mission_info_msg);

        return;
    }

    if((mission_info_msg->finished == false) && (mission_info_msg->seq_reached >= 1))
    {
        if(mission_info_msg->seq_current != seq_current)
        {
            RCLCPP_INFO(this->get_logger(), "New QGC setpoint reached, starting offboard...");

            offboard_control = true;

            seq_current = mission_info_msg->seq_current;
        }
    }

    else if (mission_info_msg->finished == true)
    {
        RCLCPP_INFO(this->get_logger(), "Mission Finished, Returning to launch");

        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0, 0);
    }

}

void OffboardManager::timer_callback()
 {
	 if(offboard_control == true)
	 {
		if (offboard_setpoint_counter_ == 10) 
			{
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}
			

		if (offboard_setpoint_counter_ == 1000) 
			{
				// Change to Offboard mission after 1000 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 6, 1);
				
				offboard_control = false;
				
			}

			// stop offboard control after a while and then continue mission
			if (offboard_setpoint_counter_ < 1001) { offboard_setpoint_counter_++; }
			
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

	 else if(offboard_control == false)
	 {
		 //reset setpoint counter
		 offboard_setpoint_counter_ = 0;
	 }
 }



void OffboardManager::publish_trajectory_setpoint() {

	setpoint.timestamp = timestamp_.load();
	
	trajectory_setpoint_publisher_->publish(setpoint);
}



void OffboardManager::publish_offboard_control_mode(std::string param) const {
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

void OffboardManager::publish_vehicle_command(uint16_t command, float param1, float param2) const 
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

 void OffboardManager::reset_flags(MissionResult::SharedPtr mission_reset_info)
{
    instance_count = mission_reset_info->instance_count;
    seq_reached = mission_reset_info->seq_reached;
    seq_current = mission_reset_info->seq_current;
    seq_total = mission_reset_info->seq_total;
    mission_finished = mission_reset_info->finished;

    std::cout<<" Number of mission uploaded: " << instance_count << std::endl;
    std::cout<<" Number of items in the mission:" << seq_total << std::endl;

    std::cout<<"--------------------------------" << seq_total << std::endl;


}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardManager>());
    rclcpp::shutdown();
    return 0;
}