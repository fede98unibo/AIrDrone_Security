#include "landing_target_detector_copy.h"

using namespace std::chrono_literals;
using namespace std;
using namespace cv;

LandingDetector::LandingDetector() : Node("landing_detector")
{
    cv::namedWindow(OPENCV_WINDOW);

	contourDetected = false;

    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;

    Image_sub_ = image_transport::create_subscription(this, "/camera/image_raw",
            std::bind(&LandingDetector::imageCallback, this, std::placeholders::_1), "raw", custom_qos);

    position_pub_ = 
        this->create_publisher<px4_msgs::msg::LandingTargetPose>("/fmu/landing_target_pose/in", 10);

    timer_ = 
        this->create_wall_timer(50ms,std::bind(&LandingDetector::timer_callback, this));
}



/*
* @brief Callback: whenever an image is received it is stored as in order perform computation
*/
void LandingDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    auto current_position = px4_msgs::msg::LandingTargetPose();

   
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

		vector<Point2f> detected_points = find_target_points(cv_ptr->image); 

		if(detected_points.empty())
		{
			throw 1;
		}

		else
		{
			cv::circle(cv_ptr -> image,detected_points[0], 10, CV_RGB(255,0,0));
			//cv::circle(cv_ptr -> image,detected_points[1], 10, CV_RGB(0,255,0));
			cv::circle(cv_ptr -> image,detected_points[2], 10, CV_RGB(0,0,255));
			//cv::circle(cv_ptr -> image,detected_points[3], 10, CV_RGB(255,255,255));
			
			vector<float> quadcopter_pose = compute_pose(detected_points);
			
			current_position.x_rel = (quadcopter_pose[0])/100;
			current_position.y_rel = (quadcopter_pose[1])/100;
			current_position.z_rel = -0.1  -(quadcopter_pose[2])/100;
			
			current_position.rel_pos_valid = true;
			

			current_position.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

			position_pub_ -> publish(current_position);

			std::cout<<"----------------"<<std::endl;
			std::cout<<"Target detected:"<<current_position.rel_pos_valid<<std::endl;
			std::cout<<"x:"<<current_position.x_rel<<std::endl;
			std::cout<<"y:"<<current_position.y_rel<<std::endl;
			std::cout<<"z:"<<current_position.z_rel<<std::endl;

			
			line(cv_ptr -> image, frame_points[0], frame_points[1], Scalar(255,0 , 0),2);
			line(cv_ptr -> image, frame_points[0], frame_points[2], Scalar(0, 255, 0),2);
			line(cv_ptr -> image, frame_points[0], frame_points[3], Scalar(0, 0, 255),2);
			
			//update internal class state about current position
			this->current_position.x = current_position.x_rel;
			this->current_position.y = current_position.y_rel;
			this->current_position.z = current_position.z_rel;
		}
	}
	
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}
	catch(int target_error_number)
	{
		RCLCPP_ERROR(this->get_logger(), "target error number: %d", target_error_number);

		current_position.rel_pos_valid = false;
		current_position.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
		
		//position_pub_ -> publish(current_position);

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
* @brief: find target points from downcamera's image frame
*/
vector<Point2f> LandingDetector::find_target_points(Mat image)
{  
    if(this->current_position.z < -3.0)
	{	
	//Clone  
    Mat image_cloned = image.clone();

    //Convert RGB to Grayscale
    cv::cvtColor(image_cloned, image_cloned, cv::COLOR_RGB2GRAY);
        
    //Binarization
    cv::threshold(image_cloned, image_cloned, 150, 255, cv::THRESH_BINARY);

    //Find contours
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	cv::findContours(image_cloned, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	// Detect target points and compute pose
    for(int i=0; i<contours.size(); i++)
	{	
		int formFactor = pow(cv::arcLength(contours[i],true),2) / cv::contourArea(contours[i]);
		
		if((cv::contourArea(contours[i])> min_rect_area) && (formFactor > 16-delta) && (formFactor<16+delta))
        {
			contourDetected = true;
		
			// Find 4 corners of the Minimum Enclosing Rectangle
			cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
			Point2f rect_points[4];
			minRect.points(rect_points);

			// ***TODO: Order the detected points to avoid ambiguity 
			Point2f baricenter;

			baricenter.x = (rect_points[0].x+rect_points[1].x+rect_points[2].x+rect_points[3].x)/4;
			baricenter.y = (rect_points[0].y+rect_points[1].y+rect_points[2].y+rect_points[3].y)/4;

			Point2f rect_points_ordered[4];

			for(int i = 0; i<4 ; i++)
			{
				if((rect_points[i].x > baricenter.x) && (rect_points[i].y > baricenter.y)) rect_points_ordered[2] = rect_points[i];

				else if((rect_points[i].x > baricenter.x) && (rect_points[i].y < baricenter.y)) rect_points_ordered[3] = rect_points[i];

				else if((rect_points[i].x < baricenter.x) && (rect_points[i].y < baricenter.y)) rect_points_ordered[1] = rect_points[i];

				else rect_points_ordered[0] = rect_points[i];
			}


			std::vector<Point2f> detected_points = { Point2f(rect_points[0]),Point2f(rect_points[1]),Point2f(rect_points[2]),Point2f(rect_points[3]) };

			return detected_points;		
		}
	}
	}
	else
	{

	QRCodeDetector qrDecoder;
    
    Mat bbox, rectifiedImage;
    
    bool found = qrDecoder.detect(image, bbox);
	
	if(found == false)
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot detect QRcode");
		return {};
	}

	else
	{
		int n = bbox.rows;

		if(n >= 3)
		{
			std::vector<Point2f> detected_points;

			for(int i = 0 ; i < n ; i++)
			{
				line(image, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);

				detected_points.push_back(Point2f(bbox.at<float>(i,0),bbox.at<float>(i,1)));
			}

		return detected_points;
		}	
	}

	}
	return {};  // if no points are found return an empty vector

	
}


/*
* @brief: compute pose from target points
*/
vector<float> LandingDetector::compute_pose(vector<Point2f> target_points)
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
    rclcpp::spin(std::make_shared<LandingDetector>());
    rclcpp::shutdown();
    return 0;
}
