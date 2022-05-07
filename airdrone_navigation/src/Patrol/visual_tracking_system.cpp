#include "visual_tracking_system.hpp"

void VisualTracker::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    for(auto det : msg -> detections)
    {
        detectionList_.emplace_back(det);
        RCLCPP_INFO(this->get_logger(),"Detection ID: %s", det.tracking_id.c_str());

        detTime_ = std::chrono::high_resolution_clock::now();
    }
}

void VisualTracker::run()
{
    switch (trackerState_)
    {
    case IDLE:
        /* code */
        //TODO: set preset angle
        break;
    
    case SEARCH:
        run_state_search();
        break;

    case TRACKING:
        run_state_tracking();
        break;

    case STABILIZE:

        break;
    }
}

void VisualTracker::run_state_search()
{
    //go to state tracking if some obj has been detected
    if(detectionList_.size() > 0){
        trackerState_ = TRACKING;
        return;
    }

    const float omega = -M_PI/6; //[rad/s]

    RPY_[2] += omega * t;
    t += dt;

    if(RPY_[2] < -M_PI){
        RPY_[2] = M_PI;
        t = 0;
    }

    gimbalSpeed_[2] = omega;
    gimbalSpeed_[1] = -omega;

    publish_gimbal_attitude(RPY_,gimbalSpeed_);
}

void VisualTracker::run_state_tracking()
{
    if(detectionList_.size() > 0)
    {
        if(first_iter==true){

            // start and offboard session
            if (!this->setpoint_client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
                return;}

            auto offboard_goal = Offboard::Goal();

            offboard_client_ptr_->async_send_goal(offboard_goal,send_goal_options_offboard);

            first_iter = false;
            last_detTime_ = detTime_;
            return;
        }

        //*********************************** GIMBAL TRACKING ***********************************//

        //At the moment track always the first element in the list TODO: handle multiple detections
        boxPosition_ << detectionList_[0].bbox.center.x, detectionList_[0].bbox.center.y, 1;
        Matrix<float,3,1> target_position = A.inverse() * boxPosition_;

        double epsX = -std::atan2(target_position.coeff(0,0),1.0);
        double epsY = std::atan2(target_position.coeff(1,0),1.0);
        
        gimbalSpeed_ = {0.0,0.5,0.5}; //Gimbal tracking speed -- Costant

        //***  YAW PI CONTROLLLER ***//
        if(abs(epsX) > MARGIN_ERROR){

            det_interval = detTime_ - last_detTime_;

            RPY_[2] += K_p * epsX  + K_i * epsX * det_interval.count();
        
        }

        //***  PITCH PI CONTROLLLER ***//
        if(abs(epsY) > MARGIN_ERROR){

            det_interval = detTime_ - last_detTime_;

            RPY_[1] += K_p * epsY + K_i * epsY * det_interval.count();            
        }

        last_detTime_ = detTime_;

        check_controller_limits(epsX,epsY);
        
        publish_gimbal_attitude(RPY_,gimbalSpeed_);

        if((abs(epsX) > MARGIN_ERROR) && (abs(epsY) > MARGIN_ERROR))
            good_detections++;
        else
            good_detections = 0;
            
        //*********************************** TARGET POSITION TRACKING ***********************************//

        if(offboard_active == true)
        {
            compute_target_position(vehiclePosition_,targetPosition_);

            if(has_moved() && (good_detections>5))
            {
                if (!this->setpoint_client_ptr_->wait_for_action_server()) {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    rclcpp::shutdown();}

                auto setpoint_goal = Setpoint::Goal();
                setpoint_goal.x = targetPosition_.x;
                setpoint_goal.y = targetPosition_.y;
                setpoint_goal.z = -3.5;

                RCLCPP_INFO(this->get_logger(),"Sending setpoint: (%f,%f)", setpoint_goal.x, setpoint_goal.y);

                setpoint_client_ptr_->async_send_goal(setpoint_goal,send_goal_options_setpoint); 
            } 
        }    

        detectionList_.clear();
    }

    else
    {
        //If target is lost switch to stabilized mode
        // targetLostTime_ = rclcpp::Time());
        // trackerState_ = STABILIZE;
        return;
    }

}

//--------------------------------------- OFFBOARD SERVER CALLBACKS -------------------------------------------------//

void VisualTracker::offboard_response_callback(std::shared_future<GoalHandleOffboard::SharedPtr> future){
    //TODO: handle the rejected request
}

void VisualTracker::offboard_feedback_callback(GoalHandleOffboard::SharedPtr, const std::shared_ptr<const Offboard::Feedback> feedback){
   
    if(feedback->offboard_status == "Offboard Active")
        offboard_active = true;
    else 
        offboard_active = false;  
}

void VisualTracker::offboard_result_callback(const GoalHandleOffboard::WrappedResult & result){
    //TODO: check if the offboard session has terminated succesfully
}

void VisualTracker::setpoint_response_callback(std::shared_future<GoalHandleSetpoint::SharedPtr> future){
    //TODO: handle rejected request
}
void VisualTracker::setpoint_feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback> feedback){
    //TODO: check distance to goal
}
void VisualTracker::setpoint_result_callback(const GoalHandleSetpoint::WrappedResult & result){
    //TODO: check if setpoint has been reached
}

//-------------------------------------------------------------------------------------------------------------------//

void VisualTracker::publish_gimbal_attitude(std::array<double,3> rpy, std::array<double,3> gimbal_speed)
{
    q_.setRPY(rpy[2],rpy[1],rpy[0]);
    q_ = q_.normalize();
    
    //set gimbal attitude
    gimbalAttitude_.timestamp = timestamp_.load();
    gimbalAttitude_.q[0] = q_.x();
    gimbalAttitude_.q[1] = q_.y();
    gimbalAttitude_.q[2] = q_.z();
    gimbalAttitude_.q[3] = q_.w();
    gimbalAttitude_.angular_velocity_x = gimbal_speed[0];
    gimbalAttitude_.angular_velocity_y = gimbal_speed[1];
    gimbalAttitude_.angular_velocity_z = gimbal_speed[2];

    gimbal_attitude_pub_->publish(gimbalAttitude_);

}

void VisualTracker::check_controller_limits(double azimuth_error, double elevation_error)
{
    //Yaw speed sign
    if( azimuth_error > 0)
        gimbalSpeed_[2] = -gimbalSpeed_[2];

    //pitch speed sign
    if(elevation_error < 0)
        gimbalSpeed_[1] = -gimbalSpeed_[1];

    
    //Map yaw into [-PI,PI]
    if(RPY_[2]>M_PI)
        RPY_[2] = RPY_[2]-2*M_PI;
    else if(RPY_[2] <-M_PI)
        RPY_[2] = RPY_[2] +2*M_PI;
}

void VisualTracker::compute_target_position(px4_msgs::msg::VehicleLocalPosition& vehicle_position, px4_msgs::msg::VehicleLocalPosition& target_position)
{
    double altitude = -vehicle_position.z;

    double alpha = M_PI_2 - (M_PI_2 - RPY_[1]);

    double distance = altitude/tan(alpha);

    double dx,dy;

    dx = distance * sin(RPY_[2] + M_PI);
    dy = distance * cos(RPY_[2] + M_PI);

    //Target Position in Local NED Reference Frame
    target_position.x = vehicle_position.x + dx;
    target_position.y = vehicle_position.y + dy;
    target_position.z = 0.0;

}

int VisualTracker::has_moved()
{
    double target_displacement = sqrt(pow(last_targetPosition_.x-targetPosition_.x,2)+pow(last_targetPosition_.y-targetPosition_.y,2));
    
    const double DISTMAX = 3.0;

    if(target_displacement  > DISTMAX)
        return 1;
    else
        return 0;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualTracker>());
    rclcpp::shutdown();
    return 0;
}

