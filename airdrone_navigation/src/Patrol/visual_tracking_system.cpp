#include "visual_tracking_system.hpp"

VisualTracker::VisualTracker()
: Node("visual_tracker")
{
    // Parameters Initialization
    this->declare_parameter<double>("K_p_gimbal", 0.05);
    this->declare_parameter<double>("K_i_gimbal", 0.2);
    this->declare_parameter<double>("K_p_tracking", 0.01);
    this->declare_parameter<double>("K_i_tracking", 0.05);
    this->declare_parameter<double>("T_ti_tracking", 1.2);
    this->declare_parameter<double>("Target_detection_box_size", 75.0);
    this->declare_parameter<double>("Gimbal_error", 0.05);
    this->declare_parameter<double>("Tracking_error", 5.0);
    this->declare_parameter<int>("Target_lost_TimeOut", 3000);
    this->declare_parameter<double>("Max_tracking_speed",3.0);
    this->declare_parameter<double>("Tracking_altitude",-2.5);

    
    this->get_parameter("K_p_gimbal", K_p);
    this->get_parameter("K_i_gimbal", K_i);
    this->get_parameter("K_p_tracking", Kp_track);
    this->get_parameter("K_i_tracking", Ki_track);
    this->get_parameter("T_ti_tracking", T_ti);
    this->get_parameter("Target_detection_box_size", TARGET_SIZE);
    this->get_parameter("Gimbal_error", MARGIN_ERROR);
    this->get_parameter("Tracking_error", TRACKING_ERROR);
    this->get_parameter("Target_lost_TimeOut", TIMEOUT);
    this->get_parameter("Max_tracking_speed", max_speed);
    this->get_parameter("Tracking_altitude", tracking_altitude);

    // Yolo detection subscriber
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
                            "/detector_node/detections", 1, std::bind(&VisualTracker::detection_callback, this, _1));

    //local position subscriber
    local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("fmu/vehicle_local_position/out", 1, 
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
            vehicle_position.x = msg->x;
            vehicle_position.y = msg->y;
            vehicle_position.z = msg->z;
        });

    // common timestamp subscriber
    timesync_sub_ =
    this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
        [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
        timestamp_.store(msg->timestamp);
        });

    // gimbal attitude publisher
    gimbal_attitude_pub_ = this->create_publisher<px4_msgs::msg::GimbalManagerSetAttitude>("fmu/gimbal_manager_set_attitude/in", 1);

    trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("visual_tracker_trajectory",1);

    // timer -> Finite State Machine
    timer_ = this->create_wall_timer(100ms, std::bind(&VisualTracker::run, this));

    // offboard and setpoint clients
    offboard_client_ptr_ = rclcpp_action::create_client<Offboard>(this,"offboard_request");
    setpoint_client_ptr_ = rclcpp_action::create_client<Setpoint>(this,"setpoint_request");

    //Init send goal options
    send_goal_options_offboard = rclcpp_action::Client<Offboard>::SendGoalOptions();
    send_goal_options_setpoint = rclcpp_action::Client<Setpoint>::SendGoalOptions();
    
    using namespace std::placeholders;

    send_goal_options_offboard.goal_response_callback =
    std::bind(&VisualTracker::offboard_response_callback, this, _1);
    send_goal_options_offboard.feedback_callback =
    std::bind(&VisualTracker::offboard_feedback_callback, this, _1, _2);
    send_goal_options_offboard.result_callback =
    std::bind(&VisualTracker::offboard_result_callback, this, _1);

    send_goal_options_setpoint.goal_response_callback =
    std::bind(&VisualTracker::setpoint_response_callback, this, _1);
    send_goal_options_setpoint.feedback_callback =
    std::bind(&VisualTracker::setpoint_feedback_callback, this, _1, _2);
    send_goal_options_setpoint.result_callback =
    std::bind(&VisualTracker::setpoint_result_callback, this, _1);

    //Gimbal manager info
    gimbalAttitude_.target_system = 0;
    gimbalAttitude_.target_component = 0;
    gimbalAttitude_.gimbal_device_id = 0;

    //intrinsic camera matrix
    // TODO: put this in the yaml
    A << 100, 0, 320, 0, 100, 180, 0, 0, 1;

    RCLCPP_INFO(this->get_logger(),"Visual Tracker intialization completed");
}

void VisualTracker::run()
{
    switch (trackerState_)
    {
    case IDLE:
        run_state_idle();
        //TODO: set preset angle
        break;
    
    case SEARCH:
        run_state_search();
        break;

    case TRACKING:
        run_state_tracking();
        break;

    case ERROR:
        run_state_error();
        break;
    }
}

void VisualTracker::run_state_search()
{
    // Search constant gimbal position
    RPY_[1] = M_PI/6;
    RPY_[2] = M_PI;

    gimbalSpeed_ = {0.0,M_PI,M_PI};

    publish_gimbal_attitude(RPY_,gimbalSpeed_);

    //go to state tracking if some obj has been detected
    if(detectionList_.size() > 0){
        RCLCPP_WARN(this->get_logger(),"SEARCH -> TRACKING");
        trackerState_ = TRACKING;
        return;
    }
}

void VisualTracker::run_state_tracking()
{
    if(detectionList_.size() > 0)
    {
        if(first_iter==true){

            //start and offboard session
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
        
        gimbalSpeed_ = {0.0,2*M_PI,2*M_PI}; //Gimbal tracking speed -- Costant

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


        check_controller_limits(epsX,epsY);
        
        publish_gimbal_attitude(RPY_,gimbalSpeed_);


        //*********************************** Box Dimension Tracking ***********************************//
        
        if(offboard_active == true){
        
        double actual_size = detectionList_[0].bbox.size_x;
        double yaw = RPY_[2] + M_PI;

        double speed;
        double vertical_speed;
        double box_error = TARGET_SIZE - actual_size;
        //***  SPEED PI + Anti-Wind-up CONTROLLLER ***//
        if(abs(box_error) > TRACKING_ERROR){

            det_interval = detTime_ - last_detTime_;

            speed = Kp_track * box_error + Ki_track * box_error * det_interval.count() + 1/T_ti * saturation_error;
        
            if(speed > max_speed){
                saturation_error = max_speed - speed;
                speed = max_speed;
            }
            else{
                saturation_error = 0;
            }
        
        }
        else 
            speed = 0;
        
        commanded_speed.x = speed * sin(yaw);
        commanded_speed.y = speed * cos(yaw);
        
        //RCLCPP_INFO(this->get_logger(),"Target size: %f, actual size: %f, yaw: %f", TARGET_SIZE, actual_size, yaw);

        // Altitude speed controller

        double vertical_error = tracking_altitude - vehicle_position.z;
        if(vertical_error> 0.2){

            det_interval = detTime_ - last_detTime_;

            vertical_speed = Kp_track * vertical_error + Ki_track * vertical_error * det_interval.count() + 1/T_ti * saturation_error_elevation;
        
            if(vertical_speed > max_speed){
                saturation_error_elevation = max_speed - vertical_speed;
                vertical_speed = max_speed;
            }
            else{
                saturation_error = 0;
            }
        
        }
        else 
            vertical_speed = 0;

        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.vx = commanded_speed.x;
        setpoint.vy  = commanded_speed.y;
        setpoint.vz = vertical_speed;

        trajectory_pub_->publish(setpoint);
        
        RCLCPP_INFO(this->get_logger(),"Commanded_speed: (%f,%f,%f)", setpoint.vx, setpoint.vy, setpoint.vz);
        }

        last_detTime_ = detTime_;
        detectionList_.clear();
    }

    else
    {
        //If target is lost switch to stabilized mode
        auto currentTime_ = std::chrono::high_resolution_clock::now();

        long int deltaT = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime_-last_detTime_).count();

        //RCLCPP_INFO(this->get_logger(),"Target lost, ms from last seen: %ld", deltaT);

        if(deltaT > TIMEOUT){            
            offboard_client_ptr_->async_cancel_all_goals();
            trackerState_ = SEARCH;
            first_iter = true;
            RCLCPP_WARN(this->get_logger(),"Target Lost Timeout...TRACKING -> SEARCH");
        }
        
        return;
    }

}

void VisualTracker::run_state_error()
{
    bool exit_code = false;

    throw exit_code;

    return;
}


//--------------------------------------- OFFBOARD SERVER CALLBACKS -------------------------------------------------//

void VisualTracker::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    for(auto det : msg -> detections)
    {
        detectionList_.emplace_back(det);
        //RCLCPP_INFO(this->get_logger(),"Detection ID: %s", det.tracking_id.c_str());

        detTime_ = std::chrono::high_resolution_clock::now();
    }
}

void VisualTracker::offboard_response_callback(std::shared_future<GoalHandleOffboard::SharedPtr> future){
    
    auto goal_handle = future.get();

    if(!goal_handle){
        RCLCPP_ERROR(this->get_logger(),"Offboard Session request rejected by the server.");
        trackerState_ = ERROR;
    }
}

void VisualTracker::offboard_feedback_callback(GoalHandleOffboard::SharedPtr, const std::shared_ptr<const Offboard::Feedback> feedback){
   
    if(feedback->offboard_status == "Offboard Active")
        offboard_active = true;
    else 
        offboard_active = false;  
}

void VisualTracker::offboard_result_callback(const GoalHandleOffboard::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Offboard Session was aborted");
        trackerState_ = ERROR;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Offboard Session was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown Offboard Session result");
        return;
    }
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

// void VisualTracker::compute_target_position(px4_msgs::msg::VehicleLocalPosition& vehicle_position, px4_msgs::msg::VehicleLocalPosition& target_position)
// {
//     double altitude = -vehicle_position.z;

//     double alpha = M_PI_2 - (M_PI_2 - RPY_[1]);

//     double distance = altitude/tan(alpha);

//     double dx,dy;

//     dx = distance * sin(RPY_[2] + M_PI);
//     dy = distance * cos(RPY_[2] + M_PI);

//     //Target Position in Local NED Reference Frame
//     target_position.x = vehicle_position.x + dx;
//     target_position.y = vehicle_position.y + dy;
//     target_position.z = 0.0;
// }

// int VisualTracker::has_moved()
// {
//     double target_displacement = sqrt(pow(last_targetPosition_.x-targetPosition_.x,2)+pow(last_targetPosition_.y-targetPosition_.y,2));
    
//     const double DISTMAX = 3.0;

//     if(target_displacement  > DISTMAX)
//         return 1;
//     else
//         return 0;
// }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VisualTracker>();

    try{
        rclcpp::spin(node);
    }
    catch(...){
        RCLCPP_ERROR(node->get_logger(),"Excpetion occurred...closing Visual Tracker");
        node.reset();
    }
    
    rclcpp::shutdown();
    return 0;
}


