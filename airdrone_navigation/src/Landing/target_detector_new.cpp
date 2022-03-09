#include "target_detector_new.h"

TargetDetector::TargetDetector() : Node("target_detector")
{
    RCLCPP_INFO(this->get_logger(), " Initializing SURF target detector...");

    std::this_thread::sleep_for(2000ms);

    cv::namedWindow(OPENCV_WINDOW);

    // Initialize node parameters, TODO: Use a file.yaml
    image_path =  "/home/fede/px4_ros_com_ros2/src/airdrone_navigation/src/Landing/reference_images/ReferenceImage1.png";
    minHessian = 400;
    ratio_thresh = 0.55;
    minGoodMatch = 5;
    //
    
    detector = SURF::create( minHessian );

    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    Image_sub_ = image_transport::create_subscription(this, "/image_raw",
            std::bind(&TargetDetector::imageCallback, this, std::placeholders::_1), "raw", custom_qos);

    position_pub_ = 
        this->create_publisher<px4_msgs::msg::LandingTargetPose>("/fmu/landing_target_pose/in", 1);

    this-> Init();

    RCLCPP_INFO(this->get_logger(),"Initialization completed");
}

/*
* @brief Callback: Initialize Node, find salient point on the target image using SURF algorithm
*/
int TargetDetector::Init()
{
    img_object = imread( samples::findFile(image_path), IMREAD_GRAYSCALE );

    if ( img_object.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        return -1;
    }

    //Search keypoints in the reference image, do this once when the node is activated.
    detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );

    return 0;
}



/*
* @brief Callback: whenever an image is received it is stored as in order perform computation
*/
void TargetDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    auto current_position = px4_msgs::msg::LandingTargetPose();

    RCLCPP_INFO(this->get_logger(),"image recieved");
   
	try
	{
		

        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

		vector<Point2f> detected_points = find_target_points(cv_ptr->image); 

		if(detected_points.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "No points detected!");
            throw 1;
		}

		else
		{
			cv::circle(cv_ptr -> image,detected_points[0], 10, CV_RGB(255,0,0));
			//cv::circle(cv_ptr -> image,detected_points[1], 10, CV_RGB(0,255,0));
			cv::circle(cv_ptr -> image,detected_points[2], 10, CV_RGB(0,0,255));
			//cv::circle(cv_ptr -> image,detected_points[3], 10, CV_RGB(255,255,255));
			
			vector<float> quadcopter_pose = compute_pose(detected_points);
			
			current_position.x_rel = (quadcopter_pose[1])/100;
			current_position.y_rel = -(quadcopter_pose[0])/100;
			current_position.z_rel = -0.1 - (quadcopter_pose[2])/100;
			current_position.rel_pos_valid = true;
			current_position.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

			position_pub_ -> publish(current_position);

			std::cout<<"----------------"<<std::endl;
			std::cout<<"Target detected:"<<current_position.rel_pos_valid<<std::endl;
			std::cout<<"x:"<<current_position.x_rel<<std::endl;
			std::cout<<"y:"<<current_position.y_rel<<std::endl;
			std::cout<<"z:"<<current_position.z_rel<<std::endl;

            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( cv_ptr -> image, detected_points[0],
                detected_points[1], Scalar(0, 255, 0), 4 );
            line( cv_ptr -> image, detected_points[1],
                detected_points[2], Scalar( 0, 255, 0), 4 );
            line( cv_ptr -> image, detected_points[2],
                detected_points[3], Scalar( 0, 255, 0), 4 );
            line( cv_ptr -> image, detected_points[3],
                detected_points[0], Scalar( 0, 255, 0), 4 );
			
			//update internal class state about current position
			this->current_position.x = current_position.x_rel;
			this->current_position.y = current_position.y_rel;
			this->current_position.z = current_position.z_rel;
		}
	}
	
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}
	catch(int target_error_number)
	{
		RCLCPP_ERROR(this->get_logger(), "target error number: %d", target_error_number);

		current_position.rel_pos_valid = false;
		current_position.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
	}  		    

cv::imshow(OPENCV_WINDOW, cv_ptr->image);
cv::waitKey(3);      
 	
}

/*
* @brief: find target points from downcamera's image frame
*/
vector<Point2f> TargetDetector::find_target_points(Mat img)
{  
    // Exception may occur in this process... if something goes wrong we simply return an empty vector
    try{
        Mat img_scene = img.clone();

        //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );

        //-- Step 2: Matching descriptor vectors with a FLANN based matcher
        // Since SURF is a floating-point descriptor NORM_L2 is used
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        std::vector< std::vector<DMatch> > knn_matches;
        matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
        //-- Filter matches using the Lowe's ratio test
        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        //We need more than 4 point to compute the homography
        if(good_matches.size()>= 5)
        {
            //-- Draw matches
            /*Mat img_matches;
            drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                        Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );*/
            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;
            for( size_t i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
            }
            
            Mat H = findHomography( obj, scene, RANSAC );
            //Check if the Homography was found, otherwise return an empty vector
            if (! H.empty())
            {   
            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(0, 0);
            obj_corners[1] = Point2f( (float)img_object.cols, 0 );
            obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
            obj_corners[3] = Point2f( 0, (float)img_object.rows );
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
	cv::undistortPoints(target_points, detected_points_undistort, this->camera_matrix, this->dist_coeff);

	// Homography
	Mat H = cv::findHomography(this->landing_points, detected_points_undistort);
	
	///*** Get Pose and Orientation from Homography ***///
	
	double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) + H.at<double>(1,0)*H.at<double>(1,0) + H.at<double>(2,0)*H.at<double>(2,0));
	H /= norm;
	Mat c1  = H.col(0);
	Mat c2  = H.col(1);
	Mat c3 = c1.cross(c2);
	Mat tvec = H.col(2);
	Mat R(3, 3, CV_64F);
	for (int i = 0; i < 3; i++)
	{
	R.at<double>(i,0) = c1.at<double>(i,0);
	R.at<double>(i,1) = c2.at<double>(i,0);
	R.at<double>(i,2) = c3.at<double>(i,0);
	}
	
	vector<Point3f> ContactPoints;
	ContactPoints.push_back(Point3f(0,0,0));
	
	ContactPoints.push_back(Point3f(0,0,10));
	ContactPoints.push_back(Point3f(10,0,0));
	ContactPoints.push_back(Point3f(0,10,0));
	projectPoints( ContactPoints, R, tvec, camera_matrix, dist_coeff, frame_points);

	Mat W, U, Vt;
	SVDecomp(R, W, U, Vt);
	R = U*Vt;
	
	//***end***//
	
    // Assign Pose value to proper data type to get published
    vector<float> pose = tvec;

	return pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetDetector>());
    rclcpp::shutdown();
    return 0;
}