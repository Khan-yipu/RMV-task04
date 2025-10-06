#ifndef HK_CAMERA_NODE_HPP_
#define HK_CAMERA_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "MvCameraControl.h"
#include "MvErrorDefine.h"
#include "PixelType.h"

namespace hk_camera
{

class HkCameraNode : public rclcpp::Node
{
public:
  explicit HkCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HkCameraNode();

private:
  // Camera initialization and connection
  bool initCamera();
  bool connectCamera();
  bool disconnectCamera();
  bool startGrabbing();
  bool stopGrabbing();
  
  // Camera parameter configuration
  bool setExposureTime(double exposure_time);
  bool setGain(double gain);
  bool setFrameRate(double frame_rate);
  bool setPixelFormat(const std::string & pixel_format);
  
  // Camera parameter reading
  double getActualFrameRate();
  double getCurrentFrameRate();
  
  // Image acquisition
  void getImageCallback();
  bool getOneImage(cv::Mat & image);
  void publishImage(const cv::Mat & image);
  
  // Parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  
  // ROS2 components
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actual_frame_rate_publisher_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Publisher image_transport_publisher_;
  
  // Parameter handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  
  // Camera parameters
  std::string device_ip_;
  std::string device_serial_;
  std::string camera_name_;
  std::string frame_id_;
  std::string topic_name_;
  double exposure_time_;
  double gain_;
  double frame_rate_;
  std::string pixel_format_;
  int image_width_;
  int image_height_;
  bool use_transport_;
  bool auto_reconnect_;
  
  // Camera handle
  void * device_handle_;
  bool is_connected_;
  bool is_grabbing_;
  
  // Threading
  std::thread image_thread_;
  std::thread frame_rate_thread_;
  std::mutex camera_mutex_;
  std::atomic<bool> should_stop_;
  
  // Utility functions
  bool enumDevices();
  bool openDevice();
  bool closeDevice();
  bool configureCamera();
  std::string getPixelFormatString(MvGvspPixelType pixel_format);
  MvGvspPixelType getPixelFormatEnum(const std::string & pixel_format_str);
  
  // Frame rate monitoring
  void frameRateMonitorCallback();
};

}  // namespace hk_camera

#endif  // HK_CAMERA_NODE_HPP_