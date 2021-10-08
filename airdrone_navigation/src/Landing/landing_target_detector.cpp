#include "landing_target_detector.h"


using namespace std::chrono_literals;
using namespace std;
using namespace cv;

LandingDetector::LandingDetector() : Node("landing_detector")
{
    cv::namedWindow(OPENCV_WINDOW);
	StartStopRecognition=false;
    LandingTargetAquired=false;


    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;

    Image_sub_ = image_transport::create_subscription(this, "/camera/image_raw",
            std::bind(&LandingDetector::imageCallback, this, std::placeholders::_1), "raw", custom_qos);

    position_pub_ = 
        this->create_publisher<geometry_msgs::msg::Point>("landing_position", 10);

	target_state_pub_ = 
        this->create_publisher<std_msgs::msg::Bool>("TargetState_fsm", 10);

    timer_ = 
        this->create_wall_timer(50ms,std::bind(&LandingDetector::timer_callback, this));

    StartStopRecognition_sub_ =
			this->create_subscription<std_msgs::msg::Bool>("ActivateTargetRecognition_fsm", 1, std::bind(&LandingDetector::StartRecognition, this, _1));

}



/*
* @brief Callback: whenever an image is received it is stored as in order perform computation
*/
void LandingDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding); 
        
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }    		    
 	
//*** CORE: IMAGE PROCESSING to DETECT target and compute POSE ***//

    //Clone  
    cv::Mat image_cloned = cv_ptr -> image.clone();

    //Convert RGB to Grayscale
    cv::cvtColor(image_cloned, image_cloned, cv::COLOR_RGB2GRAY);
        
    //Binarization
    cv::threshold(image_cloned, image_cloned, 150, 255, cv::THRESH_BINARY);

    //Find contours
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	cv::findContours(image_cloned, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Detect target points and compute pose
    for(int i=0; i<contours.size(); i++){
	
		int formFactor = pow(cv::arcLength(contours[i],true),2) / cv::contourArea(contours[i]);
		
		if((cv::contourArea(contours[i])> min_rect_area) && (formFactor > 16-delta) && (formFactor<16+delta))
        {
			contourDetected = true;
		
			// Find 4 corners of the Minimum Enclosing Rectangle
			cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
			Point2f rect_points[4];
			minRect.points(rect_points);

// ***TODO: Order the detected points to avoid ambiguity ***//

			std::vector<Point2f> detected_points = {Point2f(rect_points[0]),Point2f(rect_points[1]),Point2f(rect_points[2]), 							       Point2f(rect_points[3]) };
			
			//Undistort image
			std::vector<Point2f> detected_points_undistort;
			undistortPoints(detected_points, detected_points_undistort, this->camera_matrix, this->dist_coeff);


/* Ordering Algorithm ???
			cv::Moments mu = cv::moments(contours[i], false);

			Point2f Baricenter = Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);

			std::vector<Point2f> ordered_points=image_points;

			for(int i=0; i<4;i++)
			{
				if((image_points[i].x<Baricenter.x)&&(image_points[i].y>Baricenter.y)) {ordered_points[0]=image_points[i];}

				else if((image_points[i].x<Baricenter.x)&&(image_points[i].y<Baricenter.y)) {ordered_points[1]=image_points[i];}

				if((image_points[i].x>Baricenter.x)&&(image_points[i].y<Baricenter.y)) {ordered_points[2]=image_points[i];}

				if((image_points[i].x>Baricenter.x)&&(image_points[i].y>Baricenter.y)) {ordered_points[3]=image_points[i];}

			}
			
			std::cout<<"image points"<< image_points<< std::endl;

			std::cout<< "orderd points" << ordered_points << std::endl;
*/

			// Homography
			Mat H = findHomography( this->landing_points, detected_points_undistort );
			
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
		    
		    Mat W, U, Vt;
		    SVDecomp(R, W, U, Vt);
		    R = U*Vt;
		    
		//***end***//
		
		//Draw center of world reference frame	
		Mat R_rodrigues;
		Rodrigues(R,R_rodrigues);
        std::vector<Point3f> input = { Point3f(0,0,0)};
        std::vector<Point2f> RF_centre ;
        projectPoints(input, R_rodrigues, tvec, camera_matrix, dist_coeff, RF_centre, noArray());
        cv::circle(cv_ptr -> image,RF_centre[0], 10, CV_RGB(0,0,0));
		//Draw  corners points
		cv::circle(cv_ptr -> image,rect_points[0], 10, CV_RGB(255,0,0));
		cv::circle(cv_ptr -> image,rect_points[1], 10, CV_RGB(0,255,0));
		cv::circle(cv_ptr -> image,rect_points[2], 10, CV_RGB(0,0,255));
		cv::circle(cv_ptr -> image,rect_points[3], 10, CV_RGB(255,255,255));
            
	    	
        // Assign Pose value to proper data type to get published
        vector<float> z = tvec;

        geometry_msgs::msg::Point LandingPosition;
        LandingPosition.y = -(z[0])/100;
        LandingPosition.x = (z[1])/100;
        LandingPosition.z = -0.1  -(z[2])/100;
        

//** Send Pose IF AND ONLY IF The Landing Sequence was activated by the Landing State Machine
if(this->StartStopRecognition==true)
{

        if((contourDetected==true) && (LandingTargetAquired==false))
        {
			std_msgs::msg::Bool flag;
			flag.data=true;
			target_state_pub_->publish(flag);

			LandingTargetAquired=true;

			RCLCPP_INFO(this->get_logger(), "Target Aquired !");

        }

        position_pub_ -> publish(LandingPosition);

		cout << "X:" << LandingPosition.x<< endl;
        cout << "Y:" << LandingPosition.y<< endl;
        cout << "Z:" << LandingPosition.z<< endl;
		cout << "-----" << endl;

}
	
	
	}
	}

//*** ERROR DETECTION ***//
if(contourDetected == false) 
{

// if no contour are detected increment the error counter 
	errorCounter++;
	
	if(errorCounter >= 5)
	{
		cout << "Error: target not found for 5 consecutive frames" << endl;

		
		
		if(LandingTargetAquired==true)
		{
			std_msgs::msg::Bool flag1;
			flag1.data=false;
			target_state_pub_->publish(flag1);
			StartStopRecognition=false;
			LandingTargetAquired=false;

			RCLCPP_ERROR(this->get_logger(), "Cannot find Target !");
		}

		errorCounter = 0;
	}
}
	
else 
{
	errorCounter = 0;
	//reset
	contourDetected = false;  
}

	
cv::imshow(OPENCV_WINDOW, cv_ptr->image);
cv::waitKey(3);      
 	
}



/*
* @brief Callback: DUMMY
*/
void LandingDetector::timer_callback()
{}

/*
* @brief Callback: used to exchange data with landing_fsm, this flag is used to start, stop target detection
*/
void LandingDetector::StartRecognition(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data==true)
	{ 
		RCLCPP_INFO(this->get_logger(), "Start Target Recognition! ");
		StartStopRecognition=true; 
	}

    else 
	{ 
		StartStopRecognition=false; 
		RCLCPP_WARN(this->get_logger(), "Stopping Target Recognition");
	}
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandingDetector>());
    rclcpp::shutdown();
    return 0;
}