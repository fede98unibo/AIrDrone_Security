/****************************************************************************
 *
 * Copyright 2018 PX4 Development Team. All rights reserved.
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
 * @brief Notify every time a setpoint is reached
 * @file setpoint_checker.cpp
 * @author Federico Soffritti <fedesoffritti@gmail.com>
 */

#include <chrono>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/landing_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include <stdlib.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

//Global Variables
std::mutex mutex_; //mutex_lock

class SetpointChecker : public rclcpp::Node
{
public:
    SetpointChecker() : Node("setpoint_checker") 
    {
        landing_position_sub =
			this->create_subscription<px4_msgs::msg::LandingPosition>("LAndingPosition_PubSubTopic", 10, std::bind(&SetpointChecker::landing_position_callback, this, _1));

        local_position_sub =
			this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("VehicleLocalPosition_PubSubTopic", 10, std::bind(&SetpointChecker::local_position_callback, this, _1));

        setpoint_sub =
			this->create_subscription<geometry_msgs::msg::Point>("NewSetpoint", 10, std::bind(&SetpointChecker::setpoint_callback, this, _1));	
        
        timer_ = 
            this->create_wall_timer(100ms, std::bind(&SetpointChecker::timer_callback, this));

        landing_phase = false;
        new_setpoint_avaiable=false;
	}

    void set_landing_phase(const bool OnOff) { landing_phase = OnOff; }
    inline bool get_landing_phase() const { return landing_phase; }

    void set_new_setpoint_avaiable(const bool OnOff) { new_setpoint_avaiable = OnOff; }
    inline bool get_new_setpoint_avaiable() const { return new_setpoint_avaiable; }

    geometry_msgs::msg::Point current_position;
    geometry_msgs::msg::Point setpoint;

private:

    bool landing_phase;
    bool new_setpoint_avaiable;

    rclcpp::Subscription<px4_msgs::msg::LandingPosition>::SharedPtr landing_position_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr setpoint_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);
    void landing_position_callback(const px4_msgs::msg::LandingPosition::UniquePtr msg);
    void setpoint_callback(const geometry_msgs::msg::Point::UniquePtr msg);

    void timer_callback();
};


void SetpointChecker::timer_callback()
{
    if(get_new_setpoint_avaiable() == true)
    {
        float dx = abs(current_position.x - setpoint.x);
        float dy = abs(current_position.y - setpoint.y);
        float dz = abs(current_position.z - setpoint.z);

        //TODO: Compute spherical distance

        if((dx < 0.1) && (dy < 0.1) && (dz < 0.15))
        {
            RCLCPP_INFO(this->get_logger(), "Setpoint reached");

            //TODO: Publish info about this event

            set_new_setpoint_avaiable(false);
        }
    }
}


void SetpointChecker::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{   
    const std::lock_guard<std::mutex> lock(mutex_);

    if(get_landing_phase()==false)
    {
        current_position.x = msg->x;
        current_position.y = msg->y;
        current_position.z = msg->z;
    }

    // The mutex is automaticaly release when lock goes out of the scope
}

void SetpointChecker::landing_position_callback(const px4_msgs::msg::LandingPosition::UniquePtr msg)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    if(msg->should_land == true)
    {
        set_landing_phase(true);

        current_position.x = msg->x;
        current_position.y = msg->y;
        current_position.z = msg->z;
    }

    else
    {
       set_landing_phase(false);
    }

    // The mutex is automaticaly release when lock goes out of the scope
}


void SetpointChecker::setpoint_callback(const geometry_msgs::msg::Point::UniquePtr msg)
{
    RCLCPP_INFO(this->get_logger(), " New setpoint to be reached !");

    set_new_setpoint_avaiable(true);

    setpoint.x = msg->x;
    setpoint.y = msg->y;
    setpoint.z = msg->z;
}


int main(int argc, char *argv[])
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"SetPoint Checker node started !");

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SetpointChecker>());

	rclcpp::shutdown();
	return 0;
}