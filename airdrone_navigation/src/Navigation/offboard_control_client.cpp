#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <unistd.h>
#include "rclcpp/rate.hpp"

#include "airdrone_actions/action/offboard_request.hpp"
#include "airdrone_actions/action/setpoint.hpp"
#include <px4_msgs/msg/timesync.hpp>
#include "geometry_msgs/msg/point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace offboard_control_server
{

enum clientstate_t{Activation, Waiting, SetpointState, Travel};

class OffboardClient : public rclcpp::Node
{
public:
  using Offboard = airdrone_actions::action::OffboardRequest;
  using GoalHandleOffboard = rclcpp_action::ClientGoalHandle<Offboard>;
  using Setpoint = airdrone_actions::action::Setpoint;
  using GoalHandleSetpoint = rclcpp_action::ClientGoalHandle<Setpoint>;

  explicit OffboardClient(const rclcpp::NodeOptions & options )
  : Node("offboard_action_client", options)
  {

    RCLCPP_INFO(this->get_logger(), "Starting Offboard Client goal");

    clientState = Activation;

    //Create Path
    geometry_msgs::msg::Point stp;
    stp.x = 5.0;
    stp.y = 0.0;
    stp.z = -2.5;
    setpoints.push_back(stp);
    stp.x = 5.0;
    stp.y = 5.0;
    stp.z = -2.5;
    setpoints.push_back(stp);
    stp.x = 0.0;
    stp.y = 5.0;
    stp.z = -2.5;
    setpoints.push_back(stp);
    stp.x = 0.0;
    stp.y = 0.0;
    stp.z = -2.5;
    setpoints.push_back(stp);

    RCLCPP_INFO(this->get_logger(), "Setpoint sequence completed");
    for(int i=0; i< 4;i++)
    {
      RCLCPP_INFO(this->get_logger(),"Setpoint: %f, %f, %f", setpoints[i].x, setpoints[i].y, setpoints[i].z);
    }
    

    this->client_ptr_ = rclcpp_action::create_client<Offboard>(
      this,
      "offboard_request");
    this->setpoint_client_ptr_ = rclcpp_action::create_client<Setpoint>(
      this,
      "setpoint_request");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&OffboardClient::patrol_start, this));
  }

  void patrol_start()
  {
    using namespace std::placeholders;

    switch(clientState)
    {
      case Activation:
      {
        RCLCPP_INFO(this->get_logger(), "Starting offboard request");

        if (!this->client_ptr_->wait_for_action_server()) {
          RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
          rclcpp::shutdown();
        }

        auto goal_msg = Offboard::Goal();
        goal_msg.request_type = 0; //Useless for now

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Offboard>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&OffboardClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
          std::bind(&OffboardClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
          std::bind(&OffboardClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        if (!this->setpoint_client_ptr_->wait_for_action_server()) {
          RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
          rclcpp::shutdown();}
        
        clientState = Waiting;
      
      }
      break;

      case Waiting:
      {
  
      }
      break;

      case SetpointState:
      {
        RCLCPP_INFO(this->get_logger(), "Sending setpoint goal: %f, %f, %f", setpoints[count].x, setpoints[count].y, setpoints[count].z);

        sleep(2);

        auto goal_msg1 = Setpoint::Goal();
        goal_msg1.x = setpoints[count].x;
        goal_msg1.y = setpoints[count].y;
        goal_msg1.z = setpoints[count].z;

        auto send_goal_options1 = rclcpp_action::Client<Setpoint>::SendGoalOptions();
        send_goal_options1.goal_response_callback =
          std::bind(&OffboardClient::setpoint_response_callback, this, _1);
        send_goal_options1.feedback_callback =
          std::bind(&OffboardClient::setpoint_feedback_callback, this, _1, _2);
        send_goal_options1.result_callback =
          std::bind(&OffboardClient::setpoint_result_callback, this, _1);
        this->setpoint_client_ptr_->async_send_goal(goal_msg1, send_goal_options1);

        RCLCPP_INFO(this->get_logger(), "Switching to Travel");  

        clientState = Travel;  
      }
      break;

      case Travel:
      {

      }
      break;
    }
        //this->client_ptr_->async_cancel_all_goals(nullptr);

  }

private:
  rclcpp_action::Client<Offboard>::SharedPtr client_ptr_;
  rclcpp_action::Client<Setpoint>::SharedPtr setpoint_client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count = 0;
  clientstate_t clientState;
  std::vector<geometry_msgs::msg::Point> setpoints;
  bool active = false;

  void setpoint_response_callback(std::shared_future<GoalHandleSetpoint::SharedPtr> future1){
    auto goal_handle = future1.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Offboard Setpoint NOT accepted by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Offboard Setpoint accepted by server");
    }
  }
  void setpoint_feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback> feedback){

  }
  void setpoint_result_callback(const GoalHandleSetpoint::WrappedResult & result1){
     
    if(count<setpoints.size()){
      RCLCPP_INFO(this->get_logger(),"Setpoint Result received");
      count ++;
      clientState = SetpointState;}
    }

  void goal_response_callback(std::shared_future<GoalHandleOffboard::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Offboard request NOT accepted by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Offboard request accepted by server");
    }    
  }

  void feedback_callback(
    GoalHandleOffboard::SharedPtr,
    const std::shared_ptr<const Offboard::Feedback> feedback)
  {

    if((feedback->offboard_status == "Offboard Active") && (active==false))
    {
      active = true;
      clientState = SetpointState;
    }

  }

  void result_callback(const GoalHandleOffboard::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Offboard Session was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    rclcpp::shutdown();
  }


};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(offboard_control_server::OffboardClient)