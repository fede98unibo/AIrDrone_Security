#include "offboard_control_server.h"

namespace offboard_control_server
{
 
//****** OFFBOARD REQUEST SERVICE ******//

rclcpp_action::GoalResponse OffboardServer::handle_offboard_request_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Offboard::Goal> goal)
{
    //Check current ServerState, if it is already on this request is useless =>  Reject
    if(serverState != OffState){
        RCLCPP_WARN(this->get_logger(), "Offboard already active, request rejected...");
        return rclcpp_action::GoalResponse::REJECT;
    }

    //Check request type and set corresponding Server Mode
    if(goal->request_type == 0){
        serverMode = MissionMode;
        RCLCPP_INFO(this->get_logger(), "Received offboard request for Mission Task");}
    else if(goal->request_type == 1){
        serverMode = LandingMode;
        RCLCPP_INFO(this->get_logger(), "Received offboard request for Landing Task");}

    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

//TODO: andle canceling task in a smart way
rclcpp_action::CancelResponse OffboardServer::handle_offboard_request_cancel(const std::shared_ptr<GoalHandleOffboard> goal_handle)
{
    RCLCPP_WARN(this->get_logger(), "Received request to cancel offboard goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffboardServer::handle_offboard_request_accepted(const std::shared_ptr<GoalHandleOffboard> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&OffboardServer::execute_offboard, this, _1), goal_handle}.detach();
}

void OffboardServer::execute_offboard(const std::shared_ptr<GoalHandleOffboard> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting Offboard Session");
    
    int rate = 20;
    rclcpp::Rate loop_rate(rate);

    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<Offboard::Feedback>();

    auto result = std::make_shared<Offboard::Result>();

    int loop_counter = 0;

    //Initilize Trajectory: keep the current position -> HOVERING
    trajectory.x = currentPosition.x;
    trajectory.y = currentPosition.y;
    trajectory.z = currentPosition.z;
    trajectory.yaw = 1.57;

    while(rclcpp::ok()) 
    {
        //check if cancel request has been issued
        if (goal_handle->is_canceling()) 
        {
            //return to mission mode if the offboard goal has been canceled
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);

            //send back result
            result->offboard_status = "Offboard Result";
            goal_handle->canceled(result);

            RCLCPP_WARN(this->get_logger(), "Cenceling Offboard Session");

            //Set server state to off
            serverState = OffState;

            return;
        }
    
        loop_counter++; 

        //publish feedback based and current serverState
        if(serverState == OffState){ 
        feedback->offboard_status = "Offboard Not Active";
        goal_handle->publish_feedback(feedback);
        }
        else{
        feedback->offboard_status = "Offboard Active";
        goal_handle->publish_feedback(feedback);
        }
        
        if (loop_counter == 20) {

                    // Change to Offboard mode after 20 setpoints
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                    //Set server state to idle, now we can receive setpoint
                    serverState = IdleState;
             }

        // offboard_control_mode needs to be paired with trajectory_setpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        //RCLCPP_INFO(this->get_logger(), "Current trajectory: %f, %f, %f", trajectory.x, trajectory.y, trajectory.z);

        //TODO: check if error occur during the execution, if present exit the loop

        loop_rate.sleep();
    }

    // If we arrive here something went wrong
    if (rclcpp::ok()) {
          
        //Abort the goal
        result->offboard_status = "The offboard server is DESTROIED"; 
        goal_handle->abort(result);

        //switch back to some 'safe mode'
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);

        RCLCPP_ERROR(this->get_logger(), "Offboard Server Shutdown");    
    }
}

//****** SETPOINT REQUEST SERVICE ******//

rclcpp_action::GoalResponse OffboardServer::handle_setpoint_request_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Setpoint::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(this->get_logger(),"Received request of Setpoint");

    //if the serverState is IDLE we can accept a new setpoint, TODO: handle setpoint queue
    if(serverState==IdleState)
    {
        //switch to busyState
        serverState = BusyState;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else
        return rclcpp_action::GoalResponse::REJECT;  
}

rclcpp_action::CancelResponse OffboardServer::handle_setpoint_request_cancel(const std::shared_ptr<GoalHandleSetpoint> goal_handle1)
{
    RCLCPP_WARN(this->get_logger(), "Received request to cancel current setpoint goal... canceling");
    (void)goal_handle1;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffboardServer::handle_setpoint_request_accepted(const std::shared_ptr<GoalHandleSetpoint> goal_handle1)
{
    using namespace std::placeholders;
    std::thread{std::bind(&OffboardServer::execute_setpoint_request, this, _1), goal_handle1}.detach();
}

void OffboardServer::execute_setpoint_request(const std::shared_ptr<GoalHandleSetpoint> goal_handle1)
{
    RCLCPP_INFO(this->get_logger(),"Executing current setpoint");

    //Set loop rate
    int rate = 10;
    rclcpp::Rate loop_rate(rate);

    auto result = std::make_shared<Setpoint::Result>();

    float dt = 1/float(rate);
    float tt = 0;

    float distance2goal=0;

    const auto goal = goal_handle1->get_goal();
    geometry_msgs::msg::Point destination;
    destination.x = goal->x - currentPosition.x;
    destination.y = goal->y - currentPosition.y;
    destination.z = goal->z - currentPosition.z;

    //Compute 3D-trajectory to reach the Goal
    const float t_max = 10; //[s]
    vector<vector<float>> cubic_poly = generate_polynomial(destination, t_max);
    
    //Save initial position of vehicle before start sending the trajectory
    geometry_msgs::msg::Point initialPosition;
    initialPosition.x = currentPosition.x;
    initialPosition.y = currentPosition.y; 
    initialPosition.z = currentPosition.z;

    while(rclcpp::ok())
    {
        //check if cancel request has been issued
        if (goal_handle1->is_canceling()) 
        {
            goal_handle1->canceled(result);

            //Switch back to idle state
            serverState = IdleState;

            return;
        }

        //Compute distance from Goal
        distance2goal = sqrt(pow((destination.x+initialPosition.x)-currentPosition.x,2)+
                             pow((destination.y+initialPosition.y)-currentPosition.y,2)+
                             pow(abs(destination.z+initialPosition.z)-abs(currentPosition.z),2));
        //RCLCPP_INFO(this->get_logger(),"Destionation Z: %f, Current Z: %f", destination.z , currentPosition.z);
        //RCLCPP_INFO(this->get_logger(),"distance to goal: %f", distance2goal);
        
        //Publish feedback
        auto feed = std::make_shared<Setpoint::Feedback>();
        feed -> distance = distance2goal;
        goal_handle1->publish_feedback(feed);

        if(tt < t_max)
        {
            //Update trajectory based on current polynomial
            trajectory.x = -(cubic_poly[0][0]+cubic_poly[0][1]*tt+cubic_poly[0][2]*pow(tt,2)+cubic_poly[0][3]*pow(tt,3)) + initialPosition.x;
            trajectory.y = -(cubic_poly[1][0]+cubic_poly[1][1]*tt+cubic_poly[1][2]*pow(tt,2)+cubic_poly[1][3]*pow(tt,3)) + initialPosition.y;
            trajectory.z = -(cubic_poly[2][0]+cubic_poly[2][1]*tt+cubic_poly[2][2]*pow(tt,2)+cubic_poly[2][3]*pow(tt,3)) + initialPosition.z;
            
            //RCLCPP_INFO(this->get_logger(),"trajectory: %f, %f, %f", trajectory.x, trajectory.y, trajectory.z);

            //increment time until t_max
            tt += dt;
        }
        else 
        {
            tt = t_max;
        }

        //Check if we reached the goal
        if(distance2goal < 0.2)
        {
            //result->reached = true; //TODO: check if vehicle switched beck to missio mode before sending the result
            result->reached = true;
            goal_handle1->succeed(result);

            RCLCPP_INFO(this->get_logger(),"Setpoint reached");

            //Set server state to idle
            serverState = IdleState;

            return;
        }

        //TODO: check if error occurs

        loop_rate.sleep();
    }

    // If we arrive here something went wrong
    if (rclcpp::ok()) {
          
        //Abort the goal
        result->reached = false; 
        goal_handle1->abort(result);

        RCLCPP_ERROR(this->get_logger(), "Cannot reach the setpoint");    
    }


}

//*****************************************************************************************************************************//

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardServer::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
    msg.position = true;
    msg.velocity = false;
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
void OffboardServer::publish_trajectory_setpoint() {

	trajectory.timestamp = timestamp_.load();

	trajectory_setpoint_publisher_->publish(trajectory);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardServer::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
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

/**
 * @brief Generate 3D Cubic Polynomial
 * @param dest   Destination Point
 * @param time   Time needed to reach the destination
 */
vector<vector<float>> OffboardServer::generate_polynomial(geometry_msgs::msg::Point dest, float time)
{
    vector<vector<float>> poly;
    vector<float> poly_x;
    vector<float> poly_y;
    vector<float> poly_z;

    const float T = time;

    float a0, a1, a2, a3;

    a0 = 0;
    a1= 0;
    a2 = -3*dest.x/pow(T,2);
    a3 = 2*dest.x/pow(T,3);

    poly_x.push_back(a0);
    poly_x.push_back(a1);
    poly_x.push_back(a2);
    poly_x.push_back(a3);
    poly.push_back(poly_x);

    a0 = 0;
    a1= 0;
    a2 = -3*dest.y/pow(T,2);
    a3 = 2*dest.y/pow(T,3);

    poly_y.push_back(a0);
    poly_y.push_back(a1);
    poly_y.push_back(a2);
    poly_y.push_back(a3);
    poly.push_back(poly_y);

    a0 = 0;
    a1= 0;
    a2 = -3*dest.z/pow(T,2);
    a3 = 2*dest.z/pow(T,3);

    poly_z.push_back(a0);
    poly_z.push_back(a1);
    poly_z.push_back(a2);
    poly_z.push_back(a3);
    poly.push_back(poly_z);

    return poly;
}



}

RCLCPP_COMPONENTS_REGISTER_NODE(offboard_control_server::OffboardServer)



