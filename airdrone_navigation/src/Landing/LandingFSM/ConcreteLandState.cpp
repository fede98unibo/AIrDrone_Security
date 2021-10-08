#include "ConcreteLandState.h"
#include <iostream>

using namespace std;

/*** PHASE1 ***/

void Idle::toggle(Land* land)
{
	if(land->start_landing_sequence == true) 
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Landing Sequence activated");

		land->setState(Init::getInstance());
		
	}

	// TODO: check what error happened and reset all Land Class parameter if needed
	else 
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Landing Sequence not started. Cannot change State");
	}

}

LandState& Idle::getInstance()
{
	static Idle singleton;
	
	return singleton;
}

/*** PHASE2 ***/

void Init::enter(Land* land)
{ 
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"IINIT-STATE - ENTER");

	land->activate_target_recognition(); 
}

void Init::toggle(Land* land)
{
	if(land->target_error == true)
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Target Error, back to IDLE-STATE");
		
		land->start_landing_sequence=false;
		//TODO: switch off landing detector
		land->setState(Idle::getInstance()); 
	}

	else if(land->target_error == false) 
	{
		land->setState(HorizontalApproach::getInstance()); 
	}

}

void Init::exit(Land* land)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"INIT-STATE: EXIT");
}

LandState& Init::getInstance()
{
	static Init singleton;
	
	return singleton;
}

/*** PHASE3 ***/

void HorizontalApproach::enter(Land* land)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"HORIZONTAL_APPROACH-STATE: ENTER");

	geometry_msgs::msg::Point setpoint;
	setpoint.x = 0;
	setpoint.y = 0;
	setpoint.z = -1.7;

	land->publish_landing_setpoint(setpoint);

}

void HorizontalApproach::toggle(Land* land)
{
	if(land->target_error == true) 
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Target Error, back to IDLE-STATE");
		
		land->start_landing_sequence=false;
		//TODO: switch off landing detector
		land->setState(Idle::getInstance()); 
	}
	
	else if(land->target_error == false) 
	{
		//TODO: sleep is used only to wait that the quadcopter is stabilized, we should set this condition in the setpoint checker node
		sleep(10);
		land->setState(Descent::getInstance()); 
	 }
}

LandState& HorizontalApproach::getInstance()
{
	static HorizontalApproach singleton;
	
	return singleton;
}


/*** PHASE 4 ***/

void Descent::enter(Land* land)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DESCENT-STATE: ENTER");

	/*geometry_msgs::msg::Point setpoint;
	setpoint.x = 0;
	setpoint.y = 0;
	setpoint.z = -1.4;

	//TODO: set also velocity setpoint

	land->publish_landing_setpoint(setpoint);*/


		land->publish_vehicle_command(0);
		land->stop_filtering();
}

void Descent::toggle(Land* land)
{
	/*if(land->minimum_high == true)
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Minimum Altitude Reached, LANDING!!!");
		land->publish_vehicle_command(0);
		land->stop_filtering();
		//TODO: switch off landing detector
		
		//TODO: check that vehicle is really landend

		land->start_landing_sequence = false;
		land->setState(Idle::getInstance());


	}

	else if(land->target_error == true) 
	{
		land->publish_vehicle_command(0);
		land->stop_filtering();
	 }*/
}

LandState& Descent::getInstance()
{
	static Descent singleton;
	
	return singleton;
}