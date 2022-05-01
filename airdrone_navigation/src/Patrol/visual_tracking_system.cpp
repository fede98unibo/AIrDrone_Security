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
            first_iter = false;
            last_detTime_ = detTime_;
            return;
        }

        //At the moment track always the first element in the list TODO: handle multiple detections
        boxPosition_ << detectionList_[0].bbox.center.x, detectionList_[0].bbox.center.y, 1;
        Matrix<float,3,1> target_position = A.inverse() * boxPosition_;

        double epsX = -std::atan2(target_position.coeff(0,0),1.0);
        double epsY = std::atan2(target_position.coeff(1,0),1.0);
        
        gimbalSpeed_ = {0.0,0.5,0.5}; //Gimbal tracking speed

        //***  YAW PI CONTROLLLER ***//
        if(epsX > MARGIN_ERROR || epsX < -MARGIN_ERROR){

            det_interval = detTime_ - last_detTime_;

            RPY_[2] += K_p * epsX + K_i * epsX * det_interval.count();
        
        }

        //***  PITCH PI CONTROLLLER ***//
        if(epsY > MARGIN_ERROR || epsY < -MARGIN_ERROR){

            det_interval = detTime_ - last_detTime_;

            RPY_[1] += K_p * epsY + K_i * epsY * det_interval.count();            
        }

        last_detTime_ = detTime_;
        

        //Yaw speed sign
        if(epsX > 0)
            gimbalSpeed_[2] = -gimbalSpeed_[2];

        //pitch speed sign
        if(epsY < 0)
        {
            gimbalSpeed_[1] = -gimbalSpeed_[1];
        }
        
        //Map yaw into [-PI,PI]
        if(RPY_[2]>M_PI)
            RPY_[2] = RPY_[2]-2*M_PI;
        else if(RPY_[2] <-M_PI)
            RPY_[2] = RPY_[2] +2*M_PI;
        
        publish_gimbal_attitude(RPY_,gimbalSpeed_);
        
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualTracker>());
    rclcpp::shutdown();
    return 0;
}

