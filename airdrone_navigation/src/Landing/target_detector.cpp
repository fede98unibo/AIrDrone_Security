// TARGET_DETECTOR.CPP

#include "target_detector.h"

// -------------------------------------- NODE INIT --------------------------------------

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
    image_path_ = "/home/lorenzo/px4_ros_com_ros2/src/airdrone_navigation/src/Landing/reference_images/ReferenceImage.png"; // check path when working on a different pc!
    target_dim_ = 30.0;
    minHessian_ = 400;
    ratio_thresh_ = 0.55;
    minGoodMatch_ = 5;

    if (simulation_==true){
        camera_matrix_ = ( Mat_<double>(3,3) << 277.191, 0, 160.5, 0, 277.191, 120.5, 0, 0, 1) ;             
        dist_coeff_ = ( Mat_<double>(5,1) << 0, 0, 0, 0, 0) ;
    }
    else{
        camera_matrix_ = ( Mat_<double>(3,3) << 947.3496, 0, 271.247, 0, 984.41283, 236.9789, 0, 0, 1) ; 
        dist_coeff_ = ( Mat_<double>(5,1) << 0.037894, 0.83262, -0.0036415, -0.0061397, -5.379875) ;
    }

    landing_points_ = { Point2f(target_dim_,-target_dim_), Point2f(target_dim_,target_dim_), Point2f(-target_dim_,target_dim_), Point2f(-target_dim_,-target_dim_) };
   
    detector_ = SURF::create( minHessian_ );
    matcher_ = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    //auto kalman = std::make_shared<KalmanFilter>();     // KALMAN
    
    RCLCPP_INFO(this->get_logger(),"Intialized Kalman filter");


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


// -------------------------------------- TARGET DETECTOR INIT --------------------------------------
/*
* @brief Callback: Initialize Node, find salient point on the target image using SURF algorithm
*/
int TargetDetector::Init()
{
    img_object_ = imread( samples::findFile(image_path_), IMREAD_GRAYSCALE );

    if (img_object_.empty())
    {
        std::cout << "Could not open or find the image!\n" << std::endl;
        return -1;
    }

    //Search keypoints in the reference image, do this once when the node is activated.
    detector_->detectAndCompute( img_object_, noArray(), keypoints_object_, descriptors_object_ );

    return 0;
}


// -------------------------------------- IMAGE CALLBACK --------------------------------------
/*
* @brief Callback: whenever an image is received it is stored as in order perform computation
*/
void TargetDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    auto target_pose = px4_msgs::msg::LandingTargetPose();
    auto detection_box = vision_msgs::msg::BoundingBox2D();
    auto predicted_box = vision_msgs::msg::BoundingBox2D(); // box predicted in case of no detection

	try
	{
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

		std::vector<Point2f> detected_points = find_target_points(cv_ptr->image); // detect points
        // std::cout << "Detected " << detected_points.size() << " points." << std::endl; // 4 points
        
        //kalman->predict(detection_box); // KALMAN
        //kalman.predict(detection_box);
        //detection_box = kalman->update();   // KALMAN
        //detection_box = kalman.update();
        //detection_box = kalman.correct();

        //TODO: from detection box to filterd points
        // fare in modo che il kalman filter agisca anche quando non vengono detectati i frame
        // inserire un massimo numero di frame senza i quali la detection si interrompe --> reset filter

		if (detected_points.empty()) {

            if (!detection_timed_out) {
                ++no_detection_count; // increment the number of consecutive frames without detection
                predicted_box = kalman.predict(detection_box); // predict bounding box
                std::cout << "\npredicting box\n";
                RCLCPP_WARN(this->get_logger(), "No-Detection in current frame");

                // VISUALIZATION of Predicted points
                std::vector<Point2f> predicted_points = points_from_bbox(predicted_box);
                display_points_lines(predicted_points,true,255,255,0);
                //display_box(predicted_box,50,50,200);
            }

            // print an error after 50 frames without detection
            if (no_detection_count > 50) {
                detection_timed_out = true;
                kalman.reset(); // reset filter variables
                RCLCPP_ERROR(this->get_logger(), "Target detection timed out!");
                no_detection_count = 0;
            }


		}

		else // if there are detected points
		{
            detection_box = compute_bbox_info(detected_points); // compute bounding box based on detected points

            detection_timed_out = false;
            no_detection_count = 0;

            predicted_box = kalman.predict(detection_box); // predict bounding box
            detection_box = kalman.correct(); // correct prediction
            std::cout << "\npredicted and corrected box\n";

			std::vector<float> quadcopter_pose = compute_pose(detected_points);

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

			position_pub_ -> publish(target_pose); // publish target position on landing_target_pose topic

			std::cout<<"----------------"<<std::endl;
			std::cout<<"Target detected:"<<target_pose.rel_pos_valid<<std::endl;
			std::cout<<"x:"<<target_pose.x_rel<<std::endl;
			std::cout<<"y:"<<target_pose.y_rel<<std::endl;
			std::cout<<"z:"<<target_pose.z_rel<<std::endl;


            // DRAWINGS

            // Raw points
            display_points_lines(detected_points,false,255,0,0);

            // VISUALIZATION of Corrected (filtered) points
            std::vector<Point2f> filtered_points = points_from_bbox(detection_box);
            display_points_lines(filtered_points,true,0,255,0);
            

            //std::cout << filtered_points[0] << std::endl;
            //std::cout << filtered_points[1] << std::endl;
            //std::cout << filtered_points[2] << std::endl;
            //std::cout << filtered_points[3] << std::endl;


            //Publishing bounding box
            detection_pub_->publish(detection_box); // filtered box

		}
	}
	 
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}		    

    cv::namedWindow("Downward Camera", cv::WINDOW_NORMAL);
    cv::imshow("Downward Camera", cv_ptr->image);
    //cv::imshow("Downward Camera, filtered box", cv_ptr->image);
    cv::waitKey(3);
}

// -------------------------------------- FIND TARGET POINTS --------------------------------------
/*
* @brief: find target points from downcamera's image frame
*/
std::vector<Point2f> TargetDetector::find_target_points(Mat& img)
{  
    // Exception may occur in this process... if something goes wrong we simply return an empty vector
    try {

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

// -------------------------------------- COMPUTE POSE --------------------------------------
/*
* @brief: compute pose from target points
*/
std::vector<float> TargetDetector::compute_pose(std::vector<Point2f> target_points)
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

    std::vector<float> pose = tvec;

	return pose;
}

// -------------------------------------- COMPUTE BBOX --------------------------------------
/*
vision_msgs::msg::BoundingBox2D TargetDetector::compute_bbox_info(std::vector<Point2f> detected_points)
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
    // box.center.theta = 

    return box;
}
*/
vision_msgs::msg::BoundingBox2D TargetDetector::compute_bbox_info(std::vector<Point2f> detected_points) {

    auto box = vision_msgs::msg::BoundingBox2D();
    float distance_from_origin[4];
    float min_dist = 1e+10; // high nuber: for sure distance will be smaller
    int min_dist_idx;
    int i = 0;
    box.center.x = 0;
    box.center.y = 0;

    for (auto pt : detected_points) {
        box.center.x += pt.x;
        box.center.y += pt.y;

        distance_from_origin[i] = sqrt(pt.x*pt.x + pt.y*pt.y); // L2 norm of points
        if(distance_from_origin[i] < min_dist) { // search minimum distance
            min_dist = distance_from_origin[i]; 
            min_dist_idx = i;
            }
        i++;
    }

    std::cout << "min dist: " << min_dist << " index " << min_dist_idx << std::endl;
    box.center.x = box.center.x/4;
    box.center.y = box.center.y/4;
    float temp1 = detected_points[min_dist_idx].x - detected_points[(min_dist_idx+1)%4].x;
    float temp2 = detected_points[min_dist_idx].y - detected_points[(min_dist_idx+1)%4].y;
    box.size_x = sqrt(temp1*temp1 + temp2*temp2);
    box.size_y = box.size_x; // beacuse it's a square
    // box.center.theta = 
    return box;
}


std::vector<Point2f> TargetDetector::points_from_bbox(vision_msgs::msg::BoundingBox2D box) {
    std::vector<Point2f> predicted_points(5); // 4 corners and the center
    predicted_points[0].x = box.center.x + box.size_x/2;
    predicted_points[1].x = box.center.x + box.size_x/2;
    predicted_points[2].x = box.center.x - box.size_x/2;
    predicted_points[3].x = box.center.x - box.size_x/2;
    predicted_points[0].y = box.center.y + box.size_y/2;
    predicted_points[1].y = box.center.y - box.size_y/2;
    predicted_points[2].y = box.center.y - box.size_y/2;
    predicted_points[3].y = box.center.y + box.size_y/2;
    predicted_points[4].x = box.center.x; // center x
    predicted_points[4].y = box.center.y; // center y

    return predicted_points;
}

void TargetDetector::display_points_lines(std::vector<Point2f> points, bool center_required, int R, int G, int B) {
    const int radius = 5; // radius of circle
    // draw line between points
    /*
    line( cv_ptr -> image, points[0], points[1], Scalar(B, G, R), 3 );
    line( cv_ptr -> image, points[1], points[2], Scalar(B, G, R), 3 );
    line( cv_ptr -> image, points[2], points[3], Scalar(B, G, R), 3 );
    line( cv_ptr -> image, points[3], points[0], Scalar(B, G, R), 3 );
    */
    rectangle(cv_ptr -> image, points[0], points[2], Scalar(B, G, R), 3);
    
    // draw points
    circle(cv_ptr -> image, points[0], radius, Scalar(B, G, R), 4 );
    circle(cv_ptr -> image, points[1], radius, Scalar(B, G, R), 4 );
    circle(cv_ptr -> image, points[2], radius, Scalar(B, G, R), 4 );
    circle(cv_ptr -> image, points[3], radius, Scalar(B, G, R), 4 );
    if (center_required) // plot also the center (center does not exist if we are diplaying raw points)
        circle(cv_ptr -> image, points[4], radius, Scalar(B, G, R), -1 ); // center

}

void TargetDetector::display_box(vision_msgs::msg::BoundingBox2D box, int R, int G, int B) {

    const int radius = 5;
    Point2f center = Point2f(box.center.x, box.center.y);
    Point2f center2 = Point2f(box.center.x + box.size_x, box.center.y + box.size_y);
    // float theta = ...
    float size_x = box.size_x; 
    float size_y = box.size_y; 
    std::cout << "Box: center: " << box.center.x << "," << box.center.y << " Size_x: " << size_x << " Size_y: " << size_y << std::endl;
    circle(cv_ptr -> image, center, radius, Scalar(B, G, R), -1 ); // center
    line( cv_ptr -> image, center, center2, Scalar(B, G, R), 3 );

}

// -------------------------------------- MAIN --------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetDetector>());
    rclcpp::shutdown();
    return 0;
}
