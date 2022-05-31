// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef V4L2_CAMERA__V4L2_CAMERA_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_HPP_

#include "v4l2_camera/v4l2_camera_device.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vision_msgs/msg/detection2_d_array.hpp"

#include <memory>
#include <string>
#include <map>
#include <vector>

#include "v4l2_camera/visibility_control.h"

using std::placeholders::_1;

namespace v4l2_camera
{

class V4L2Camera : public rclcpp::Node
{
public:
  explicit V4L2Camera(rclcpp::NodeOptions const & options);

  virtual ~V4L2Camera();

private:
  std::shared_ptr<V4l2CameraDevice> camera_;

  // Publisher used for intra process comm
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

  // Publisher used for inter process comm
  image_transport::CameraPublisher camera_transport_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  std::thread capture_thread_;
  std::atomic<bool> canceled_;

  std::string camera_frame_id_;
  std::string output_encoding_;

  std::map<std::string, int32_t> control_name_to_id_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_;

  /**GStreamer**/
  std::shared_ptr<sensor_msgs::msg::Image> frame_;
  const std::string encoding = "appsrc ! videoconvert ! x264enc noise-reduction=1000 speed-preset=superfast tune=zerolatency ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=5600";
  cv::VideoWriter videoOut;
  cv_bridge::CvImagePtr cv_ptr;
  //**Detection**//
  mutable std::vector<vision_msgs::msg::Detection2D> detections_;

  void createParameters();
  bool handleParameter(rclcpp::Parameter const & param);

  bool requestPixelFormat(std::string const & fourcc);
  bool requestImageSize(std::vector<int64_t> const & size);

  sensor_msgs::msg::Image::UniquePtr convert(sensor_msgs::msg::Image const & img) const;

  bool checkCameraInfo(
    sensor_msgs::msg::Image const & img,
    sensor_msgs::msg::CameraInfo const & ci);

  //Callbacks
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) const
  {    
    vision_msgs::msg::Detection2D tempDet;
    if(msg->detections.empty())
      return;

    for(auto det : msg->detections)
    {
      tempDet.bbox = det.bbox;
      tempDet.results = det.results;

      detections_.push_back(tempDet);    
    }
  }

};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_HPP_