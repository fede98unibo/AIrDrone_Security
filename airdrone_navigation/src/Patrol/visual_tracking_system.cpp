#include "visual_tracking_system.hpp"

void VisualTracker::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) const
{
    numDetectedObj_ = 0;
    detectionList_.clear();

    for(auto det : msg -> detections)
    {
        detectionList_.emplace_back(det);

        if(det->isTracking == true)
            numDetectedObj_++;
    }
}

void VisualTracker::run()
{
    switch (trackerState)
    {
    case IDLE:
        /* code */
        break;
    
    case SEARCH:
        run_state_search();
        break;

    case TRACKING:
        run_state_tracking();
        break;

    case STABILIZE:
        run_state_stabilize();
        break;
    }
}

void VisualTracker::run_state_search()
{
    if(numDetectedObj_>0){
        trackerState_ = TRACKING;
        return;
    }

    gimbalAttitude_.timestamp = timestamp_.load();
    gimbalAttitude_.target_system = 0;
    gimbalAttitude_.target_component = 0;
    gimbalAttitude_.gimbal_device_id = 0;

    gimbalAttitude_.q = {0.,0.0,0.0,-0.7596879};
    gimbalAttitude_.angular_velocity_x = 0.0;
    gimbalAttitude_.angular_velocity_y = 0.0;
    gimbalAttitude_.angular_velocity_z = 0.1;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualTracker>());
    rclcpp::shutdown();
    return 0;
}

