// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
//Gloabalvariables
auto message = geometry_msgs::msg::Point();

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  { 
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("NewSetpoint", 10);
    landing_publisher_ = this->create_publisher<std_msgs::msg::Bool>("StartLanding_fsm", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
int counter=0;
private:
  void timer_callback()
  {
    std::cout << "\n- Waiting for new command:" << std::endl;

    std::string cmd;
    //get input
    std::cin >> cmd;


    if (cmd == "precision_landing_start_cmd")
    {
        std_msgs::msg::Bool start;
        start.data = true;
        landing_publisher_->publish(start);

        std::cout << "\nPrecision Landing Sequence switched ON" << std::endl;

    }

    else if ( cmd == "new_setpoint_cmd")
    {
        std::cout << "\nEnter setpoint coordinate" << std::endl;
        std::cout << "X:" << std::endl;
        std::cin >> message.x;
        std::cout << "Y:" << std::endl;
        std::cin >> message.y;
        std::cout << "Z:" << std::endl;
        std::cin >> message.z;
        std::cout << "Sending new setpoint!" << std::endl;
    
        publisher_->publish(message);

    }

    else if ( cmd == "cmd_info")
    {
       std::cout << "\nList of avaiable cmd:" << std::endl;
       std::cout << "1- new_setpoint_cmd" << std::endl;
       std::cout << "2- precision_landing_start_cmd" << std::endl;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr landing_publisher_;
  
  size_t count_;
};

void print_interface_info()
{
  std::cout << "-----------------------" << std::endl;
  std::cout << "AIrDrone User-Interface" << std::endl;
  std::cout << "-----------------------" << std::endl;
  std::cout << "\n" << std::endl;

  std::cout << "Type 'cmd_info' to check the list of avaiable commands. \n " << std::endl;
}

int main(int argc, char * argv[])
{
  print_interface_info();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
