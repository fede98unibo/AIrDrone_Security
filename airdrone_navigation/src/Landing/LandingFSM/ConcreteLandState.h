#pragma once 

#include <rclcpp/rclcpp.hpp>
#include "LandState.h"
#include "Land.h"
#include <iostream>
#include <unistd.h>
#include "geometry_msgs/msg/point.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>

/*
State description: ...
*/

class Idle : public LandState
{
public:

	void enter(Land* land) {RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"IDLE-STATE: ENTER");}

	void toggle(Land* land);
	
	void exit(Land* land) {RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"IDLE-STATE: EXIT");}
	
	static LandState& getInstance();
 
private:
	Idle(){}
	
	Idle(const Idle& other);
	
	Idle& operator=(const Idle& other);
	
};

/*
State description: ...
*/

class Init : public LandState
{
public:
	void enter(Land* land); 

	void toggle(Land* land);
	
	void exit(Land* land);
	
	static LandState& getInstance();

private:
	Init(){}
	
	Init(const Init& other);
	
	Init& operator=(const Init& other);
	
};

class HorizontalApproach : public LandState
{
public:
	void enter(Land* land);

	void toggle(Land* land);
	
	void exit(Land* land) { RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"PHASE3 - EXIT"); }
	
	static LandState& getInstance();

private:
	HorizontalApproach(){}
	
	HorizontalApproach(const HorizontalApproach& other);
	
	HorizontalApproach& operator=(const HorizontalApproach& other);
	
};

class Descent : public LandState
{
public:
	void enter(Land* land);

	void toggle(Land* land);
	
	void exit(Land* land) { RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"PHASE4 - EXIT"); }
	
	static LandState& getInstance();

private:
	Descent(){}
	
	Descent(const Descent& other);
	
	Descent& operator=(const Descent& other);
	
};