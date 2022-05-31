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

    (void)uuid;  
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffboardServer::handle_offboard_request_cancel(const std::shared_ptr<GoalHandleOffboard> goal_handle)
{
    RCLCPP_WARN(this->get_logger(), "Received request to cancel current offboard session... stopping!");
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
    
    int rate = 10;
    rclcpp::Rate loop_rate(rate);

    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<Offboard::Feedback>();

    auto result = std::make_shared<Offboard::Result>();

    //Initilize Trajectory: keep the current position -> HOVERING
    // trajectory.x = currentPosition.x;
    // trajectory.y = currentPosition.y;
    // trajectory.z = currentPosition.z;
    // trajectory.yaw = 1.57;
    trajectory.x = NAN;             //Only velocty control.... this is a SHIT !
    trajectory.y = NAN;
    trajectory.z = NAN;
    trajectory.yaw = NAN;
    trajectory.vx = 0.0;
    trajectory.vy = 0.0;
    trajectory.vz = 0.0;
    trajectory.yawspeed = 0.0;

    long int loop_counter = 0; //TODO: Pay attention to overflow....

    while(rclcpp::ok()) 
    {
        // Check if some speed trajectory is still being published
        auto currentTime_ = std::chrono::high_resolution_clock::now();
        long int deltaT = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime_-lastTrajectoryUpdate_).count();

        if(deltaT > TIMEOUT){
            speedcontrolActive = false;
        }

        //check if cancel request has been issued
        if (goal_handle->is_canceling()) 
        {
            RCLCPP_WARN(this->get_logger(), "Canceling Offboard Session");

            //return to mission mode if the offboard goal has been canceled
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);

            if(wait_for_command_ack(176,1) != 0){
                RCLCPP_ERROR(this->get_logger(),"Ack not received: cannot change mode!");
                break;
            }

            //send back result
            result->offboard_status = "Offboard Result";
            goal_handle->canceled(result);

            //Set server state to off
            serverState = OffState;

            return;
        }

        if(loop_counter < 30)
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

            if(wait_for_command_ack(176,1) != 0){
                RCLCPP_ERROR(this->get_logger(),"Ack not received: cannot change mode!");
                break;
            }

            serverState = IdleState;
        }

        // offboard_control_mode needs to be paired with trajectory_setpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        
        RCLCPP_INFO(this->get_logger(), "Current trajectory: %f, %f, %f", trajectory.x, trajectory.y, trajectory.z);

        loop_rate.sleep();
    }

    // If we arrive here something went wrong
    if (rclcpp::ok()) {

        //switch back to some 'safe mode'
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);

        if(wait_for_command_ack(176,1) != 0){
            RCLCPP_ERROR(this->get_logger(),"Ack not received: cannot change mode!");
        }
         
        //Abort the goal
        result->offboard_status = "The offboard server is DESTROIED"; 
        goal_handle->abort(result);

        RCLCPP_ERROR(this->get_logger(), "Offboard Server Shutdown");    
    }
}

//****** SETPOINT REQUEST SERVICE ******//

rclcpp_action::GoalResponse OffboardServer::handle_setpoint_request_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Setpoint::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(this->get_logger(),"Received request of Setpoint");

    //TODO: check if the setpoint is valid before accepting it
    if(speedcontrolActive == true){
        return rclcpp_action::GoalResponse::REJECT;
    }
    //if the serverState is IDLE we can accept a new setpoint, TODO: handle setpoint queue
    else if(serverState == IdleState){
        //switch to busyState
        serverState = BusyState;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else if(serverState == BusyState){
        //switch to new setpoint
        new_setpnt_received = true;
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

    if((new_setpnt_received == true) && (serverState == BusyState)){

        auto goal = goal_handle1->get_goal();
        currentSetpoint.x = goal->x;
        currentSetpoint.y = goal->y;
        currentSetpoint.z = goal->z;

    }  
    else{
        std::thread{std::bind(&OffboardServer::execute_setpoint_request, this, _1), goal_handle1}.detach();
    }
        
}

void OffboardServer::execute_setpoint_request(const std::shared_ptr<GoalHandleSetpoint> goal_handle1)
{
    RCLCPP_INFO(this->get_logger(),"Executing current setpoint");

    serverState = BusyState;

    int rate = 10;
    rclcpp::Rate loop_rate(rate);

    const auto goal = goal_handle1->get_goal();
    auto result = std::make_shared<Setpoint::Result>();
    auto feed = std::make_shared<Setpoint::Feedback>();

    double dt = 1/double(rate);
    long double tt = 0;

    geometry_msgs::msg::Point distance2goal;

    geometry_msgs::msg::Point destination;
    destination.x = goal->x;
    destination.y = goal->y;
    destination.z = goal->z;

    RCLCPP_WARN(this->get_logger(),"Destination: (%f,%f,%f) ", destination.x,destination.y,destination.z);

    //Compute 3D-trajectory to reach the Goal
    array<array<double,4>,3> cubic_poly;

    TJ.cubic_poly_xyz(currentPosition,currentSpeed,destination,cubic_poly);

    const double WAYPOINT_PRECISION = 0.2;

    while(rclcpp::ok())
    {
        //check if cancel request has been issued
        if (goal_handle1->is_canceling()) 
        {
            result->reached = false;
            goal_handle1->canceled(result);

            // Switch back to idle state
            serverState = IdleState;

            return;
        }

        // Check if a new setpoint has been received
        if(new_setpnt_received == true){

            new_setpnt_received = false; // Attento che se arriva un ulteriore setpoint uno dei due potrebbe essere sfanculato

            destination.x = currentSetpoint.x;
            destination.y = currentSetpoint.y;
            destination.z = currentSetpoint.z;

            geometry_msgs::msg::Point source;
            source.x = trajectory.x;
            source.y = trajectory.y;
            source.z = trajectory.z;

            TJ.cubic_poly_xyz(source,currentSpeed,destination,cubic_poly);

            tt = 0;            
        }

        //Compute distance from Goal
        distance2goal.x = abs(destination.x-currentPosition.x);
        distance2goal.y = abs(destination.y-currentPosition.y);
        distance2goal.z = abs(destination.z-currentPosition.z);

        float Distance3D = sqrt(pow(distance2goal.x,2) + pow(distance2goal.y,2) + pow(distance2goal.z,2));

        // RCLCPP_INFO(this->get_logger(),"distance to goal: %f", distance2goal);
        
        //Publish feedback
        feed -> distance = Distance3D;
        goal_handle1->publish_feedback(feed);

        if(distance2goal.x > WAYPOINT_PRECISION){
            trajectory.x = (cubic_poly[0][0]+cubic_poly[0][1]*tt+cubic_poly[0][2]*pow(tt,2)+cubic_poly[0][3]*pow(tt,3));
        }
        if(distance2goal.y > WAYPOINT_PRECISION){
            trajectory.y = (cubic_poly[1][0]+cubic_poly[1][1]*tt+cubic_poly[1][2]*pow(tt,2)+cubic_poly[1][3]*pow(tt,3));
        }
        if(distance2goal.z > WAYPOINT_PRECISION){
            trajectory.z = (cubic_poly[2][0]+cubic_poly[2][1]*tt+cubic_poly[2][2]*pow(tt,2)+cubic_poly[2][3]*pow(tt,3));
        }

        tt += dt;

        //Check if we reached the goal
        if(Distance3D < WAYPOINT_PRECISION)
        {
            RCLCPP_INFO(this->get_logger(),"Setpoint reached");

            result->reached = true;
            goal_handle1->succeed(result);

            //Set server state to idle
            serverState = IdleState;

            return;
        }

        //TODO: check if error occurs
        loop_rate.sleep();
    }

    // If we arrive here something went wrong or another setpoint has been received
    if (rclcpp::ok()) {
          
        //Abort the goal
        result->reached = false; 
        goal_handle1->abort(result);

        RCLCPP_ERROR(this->get_logger(), "Cannot reach the setpoint for some reason");    
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
    msg.position = false;
    msg.velocity = true;
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

int OffboardServer::wait_for_command_ack(int command_ID, int timeout)
{
    int rate = 5;
    rclcpp::Rate loop_rate(rate);

    int max_count = timeout*rate;
    int n_iter = 0;

    while(rclcpp::ok() && (n_iter<max_count))
    {
        //check if is the right ack
        if(cmdAck_.command == (unsigned int)command_ID)
        {
            //save a copy
            auto ack = cmdAck_;

            //reset ack
            cmdAck_.command = -1;
            cmdAck_.result = -1;

            return ack.result;
        }

        n_iter++;

        loop_rate.sleep();
    }

    //reset ack
    cmdAck_.command = -1;
    cmdAck_.result = -1;

    return -1;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(offboard_control_server::OffboardServer)



