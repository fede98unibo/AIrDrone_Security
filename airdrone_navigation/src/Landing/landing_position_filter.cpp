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
 * @brief Debug Vect uORB topic adverstiser example
 * @file debug_vect_advertiser.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/landing_position.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

#define MAX_SAFE_RANGE 3

using std::placeholders::_1;

//TODO: change from global variables to class members
geometry_msgs::msg::Point landing_points[3] ;
int counter=0;
using namespace std::chrono_literals;
//

class LandingPosFilter : public rclcpp::Node
{
public:
	LandingPosFilter() : Node("debug_vect_advertiser") {

		RCLCPP_INFO(this->get_logger(), "Landing Position filter node started !");

#ifdef ROS_DEFAULT_API
		current_position_pub_ = this->create_publisher<px4_msgs::msg::LandingPosition>("LandingPosition_PubSubTopic", 1);
		publisher1_ = this->create_publisher<px4_msgs::msg::LandingPosition>("LandingPosition_PubSubTopic_filtered", 1);
#else
		//current_position_pub_ = this->create_publisher<px4_msgs::msg::LandingPosition>("LandingPosition_PubSubTopic");
		//publisher1_ = this->create_publisher<px4_msgs::msg::LandingPosition>("LandingPosition_PubSubTopic_filtered");
#endif

    landingTargetPose_sub_ =
			this->create_subscription<geometry_msgs::msg::Point>("landing_position", 1, std::bind(&LandingPosFilter::position_filter, this, _1));

	timer_ = this->create_wall_timer(
		20ms, std::bind(&LandingPosFilter::timer_callback, this));
	}

private:

	void timer_callback(){}
		
	rclcpp::Publisher<px4_msgs::msg::LandingPosition>::SharedPtr current_position_pub_;
	rclcpp::Publisher<px4_msgs::msg::LandingPosition>::SharedPtr publisher1_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr landingTargetPose_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr FiteringStartStop_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
	
	//Calbacks
	void position_filter(const geometry_msgs::msg::Point::UniquePtr msg);
	void start_stop_filter(const std_msgs::msg::Bool::UniquePtr msg);

};

/*
/@brief:
*/
void LandingPosFilter::position_filter(const geometry_msgs::msg::Point::UniquePtr msg)
{
	auto current_position = px4_msgs::msg::LandingPosition();
	bool OUT_OF_RANGE_POSITION = false;
	
	try
	{
		current_position.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
		
		landing_points[0].x = msg->x;
		landing_points[0].y = msg->y;
		landing_points[0].z = msg->z;

		counter++;
		
		if (counter>3)
		{
			// Mean Filtering
			current_position.x = 0.333*(landing_points[0].x +landing_points[1].x +landing_points[2].x);
			current_position.y = 0.333*(landing_points[0].y +landing_points[1].y +landing_points[2].y);
			current_position.z = 0.333*(landing_points[0].z +landing_points[1].z +landing_points[2].z); 
			
			if((current_position.x < MAX_SAFE_RANGE) && (current_position.y < MAX_SAFE_RANGE))
			{		
				current_position.should_land = true;
				current_position_pub_->publish(current_position);
			}
					
			else 
			{  // Throw Exception
				OUT_OF_RANGE_POSITION = true;
				throw(OUT_OF_RANGE_POSITION);
			}
			
		}
		
		// Update filter array
		landing_points[2] = landing_points[1];
		landing_points[1] = landing_points[0];
	}

	catch(bool position_error)
	{	
		// Handle Exception
		if(position_error==true)
		{
			RCLCPP_ERROR(this->get_logger(),"Target position out of MAX_SAFE_RANGE !"); 

			current_position.should_land = false;
			current_position_pub_->publish(current_position);

			//TODO: Reset Filter
		}
	}
	
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LandingPosFilter>());

	rclcpp::shutdown();
	return 0;
}
