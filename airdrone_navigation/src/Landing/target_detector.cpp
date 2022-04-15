#include "target_detector.h"

TargetDetector::TargetDetector() : Node("target_detector")
{
    RCLCPP_INFO(this->get_logger(), " Initializing SURF target detector...");

    // this->get_parameter("simulation", simulation_);
    // this->get_parameter("image_path", image_path_);
    // this->get_parameter("target_dimension", target_dim_);
    // this->get_parameter("min_hessian", minHessian_);
    // this->get_parameter("ratio_thresh", ratio_thresh_);
    // this->get_parameter("min_good_match", minGoodMatch_);

    simulation_ = true;
    image_path_ = "/home/fede/px4_ros_com_ros2/src/airdrone_navigation/src/Landing/reference_images/ReferenceImage.png";
    target_dim_ = 30.0;
    minHessian_ = 400;
    ratio_thresh_ = 0.55;
    minGoodMatch_ = 5;

    if(simulation_==true){
        camera_matrix_ = ( Mat_<double>(3,3) << 277.191, 0, 160.5, 0, 277.191, 120.5, 0, 0, 1) ;             
        dist_coeff_ = ( Mat_<double>(5,1) << 0, 0, 0, 0, 0) ;
    }
    else{
        camera_matrix_ = ( Mat_<double>(3,3) << 947.3496, 0, 271.247, 0, 984.41283, 236.9789, 0, 0, 1) ; 
        dist_coeff_ = ( Mat_<double>(5,1) << 0.037894, 0.83262, -0.0036415, -0.0061397, -5.379875) ;
    }

    landing_points_ = { Point2f(target_dim_,-target_dim_), Point2f(target_dim_,target_dim_), Point2f(-target_dim_,target_dim_), Point2f(-target_dim_,-target_dim_)};
    
    detector_ = SURF::create( minHessian_ );
    matcher_ = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    kalman = std::make_shared<KalmanFilter>();

    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    Image_sub_ = image_transport::create_subscription(this, "/camera/image_raw",
            std::bind(&TargetDetector::imageCallback, this, std::placeholders::_1), "raw", custom_qos);

    position_pub_ = 
        this->create_publisher<px4_msgs::msg::LandingTargetPose>("/fmu/landing_target_pose/in", 1);

    detection_pub_ =
        this->create_publisher<vision_msgs::msg::BoundingBox2D>("/landing_target_detection", 1);

    this-> Init();

    RCLCPP_INFO(this->get_logger(),"Surf Initialization completed");
}

/*
* @brief Callback: Initialize Node, find salient point on the target image using SURF algorithm
*/
int TargetDetector::Init()
{
    img_object_ = imread( samples::findFile(image_path_), IMREAD_GRAYSCALE );

    if (img_object_.empty())
    {
        cout << "Could not open or find the image!\n" << endl;
        return -1;
    }

    //Search keypoints in the reference image, do this once when the node is activated.
    detector_->detectAndCompute( img_object_, noArray(), keypoints_object_, descriptors_object_ );

    return 0;
}

/*
* @brief Callback: whenever an image is received it is stored as in order perform computation
*/
void TargetDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    auto target_pose = px4_msgs::msg::LandingTargetPose();
    auto detection_box = vision_msgs::msg::BoundingBox2D();

	try
	{
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

		vector<Point2f> detected_points = find_target_points(cv_ptr->image);

        detection_box = compute_bbox_info(detected_points);

        kalman->predict(detection_box);

        detection_box = kalman->update();

        //TODO: from detection box to filterd points
        // fare in modo che il kalman filter agisca anche quando non vengono detectati i frame
        // inserire un massimo numero di frame senza i quali la detection si interrompe --> reset filter

		if(detected_points.empty()){
			RCLCPP_WARN(this->get_logger(), "No-Detection in current frame");
		}

		else
		{			
			vector<float> quadcopter_pose = compute_pose(detected_points);

            target_pose.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            target_pose.is_static = true;
            target_pose.rel_pos_valid = true;
            target_pose.rel_vel_valid = false;
			target_pose.x_rel = (quadcopter_pose[1])/100;
			target_pose.y_rel = -(quadcopter_pose[0])/100;
			target_pose.z_rel = -(quadcopter_pose[2])/100;
            target_pose.abs_pos_valid = true;
			target_pose.x_abs = 0;
            target_pose.y_abs = 0;
            target_pose.z_abs = 0;

			position_pub_ -> publish(target_pose);

			std::cout<<"----------------"<<std::endl;
			std::cout<<"Target detected:"<<target_pose.rel_pos_valid<<std::endl;
			std::cout<<"x:"<<target_pose.x_rel<<std::endl;
			std::cout<<"y:"<<target_pose.y_rel<<std::endl;
			std::cout<<"z:"<<target_pose.z_rel<<std::endl;

            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( cv_ptr -> image, detected_points[0],
                detected_points[1], Scalar(0, 255, 0), 4 );
            line( cv_ptr -> image, detected_points[1],
                detected_points[2], Scalar( 0, 255, 0), 4 );
            line( cv_ptr -> image, detected_points[2],
                detected_points[3], Scalar( 0, 255, 0), 4 );
            line( cv_ptr -> image, detected_points[3],
                detected_points[0], Scalar( 0, 255, 0), 4 );

            //Publishing bounding box
            detection_pub_->publish(detection_box);

		}
	}
	 
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}		    

    cv::imshow("Downward Camera", cv_ptr->image);
    cv::waitKey(3);}


/*
* @brief: find target points from downcamera's image frame
*/
vector<Point2f> TargetDetector::find_target_points(Mat& img)
{  
    // Exception may occur in this process... if something goes wrong we simply return an empty vector
    try{

        detector_->detectAndCompute( img, noArray(), keypoints_scene_, descriptors_scene_ );

        std::vector< std::vector<DMatch> > knn_matches;
        matcher_->knnMatch( descriptors_object_, descriptors_scene_, knn_matches, 2 );

        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh_ * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        //We need more than 4 point to compute the homography
        if(good_matches.size()>= 5)
        {
            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;
            for( size_t i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object_[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene_[ good_matches[i].trainIdx ].pt );
            }
            
            Mat H = findHomography( obj, scene, RANSAC );
            //Check if the Homography was found, otherwise return an empty vector
            if (! H.empty())
            {   
            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(0, 0);
            obj_corners[1] = Point2f( (float)img_object_.cols, 0 );
            obj_corners[2] = Point2f( (float)img_object_.cols, (float)img_object_.rows );
            obj_corners[3] = Point2f( 0, (float)img_object_.rows );
            std::vector<Point2f> scene_corners(4);
            perspectiveTransform( obj_corners, scene_corners, H);

            return scene_corners;
            }

            else 
                return{};
        }
        else
            return {};  // if no points are found return an empty vector
    }
    catch(cv::Exception& excpt)
    {
        RCLCPP_INFO(this->get_logger(), "Some Exception..");
        return{};
    }
}

/*
* @brief: compute pose from target points
*/
vector<float> TargetDetector::compute_pose(vector<Point2f> target_points)
{
	// Undistort image
	std::vector<Point2f> detected_points_undistort;
	cv::undistortPoints(target_points, detected_points_undistort, camera_matrix_, dist_coeff_);

	// Homography
	Mat H = cv::findHomography(this->landing_points_, detected_points_undistort);
	
	///*** Get Pose and Orientation from Homography ***///
	
	double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) + H.at<double>(1,0)*H.at<double>(1,0) + H.at<double>(2,0)*H.at<double>(2,0));
	H /= norm;
	Mat tvec = H.col(2);
	
	//***end***//

    vector<float> pose = tvec;

	return pose;
}

vision_msgs::msg::BoundingBox2D TargetDetector::compute_bbox_info(vector<Point2f> detected_points)
{
    auto box = vision_msgs::msg::BoundingBox2D();
    box.center.x = 0;
    box.center.y = 0;
    float x_max = 0;
    float y_max = 0;
    float x_min = 0;
    float y_min = 0;

    //Baricenter
    for(auto pt : detected_points)
    {
        box.center.x += pt.x;
        if(pt.x>x_max)
            x_max = pt.x;
        else if(pt.x<x_min)
            x_min = pt.x;

        box.center.y += pt.y;
        if(pt.y>y_max)
            y_max = pt.y;
        else if(pt.y<y_min)
            y_min = pt.y;
    }

    box.center.x = box.center.x/4;
    box.center.y = box.center.y/4;
    box.size_x = abs(x_max-x_min);
    box.size_y = abs(y_max-y_min);

    return box;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetDetector>());
    rclcpp::shutdown();
    return 0;
}
