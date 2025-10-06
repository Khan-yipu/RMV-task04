#include "hk_camera/hk_camera_node.hpp"

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

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

HkCameraNode::HkCameraNode(const rclcpp::NodeOptions & options)
: Node("hk_camera_node", options),
  device_handle_(nullptr),
  is_connected_(false),
  is_grabbing_(false),
  should_stop_(false)
{
  // Declare parameters
  this->declare_parameter("device_ip", "");
  this->declare_parameter("device_serial", "");
  this->declare_parameter("camera_name", "hk_camera");
  this->declare_parameter("frame_id", "camera_link");
  this->declare_parameter("topic_name", "image_raw");
  this->declare_parameter("exposure_time", 8000.0);
  this->declare_parameter("gain", 1.0);
  this->declare_parameter("frame_rate", 30.0);
  this->declare_parameter("pixel_format", "BGR8");
  this->declare_parameter("image_width", 1920);
  this->declare_parameter("image_height", 1080);
  this->declare_parameter("use_transport", true);
  this->declare_parameter("auto_reconnect", true);

  // Get parameters
  device_ip_ = this->get_parameter("device_ip").as_string();
  device_serial_ = this->get_parameter("device_serial").as_string();
  camera_name_ = this->get_parameter("camera_name").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  topic_name_ = this->get_parameter("topic_name").as_string();
  exposure_time_ = this->get_parameter("exposure_time").as_double();
  gain_ = this->get_parameter("gain").as_double();
  frame_rate_ = this->get_parameter("frame_rate").as_double();
  pixel_format_ = this->get_parameter("pixel_format").as_string();
  image_width_ = this->get_parameter("image_width").as_int();
  image_height_ = this->get_parameter("image_height").as_int();
  use_transport_ = this->get_parameter("use_transport").as_bool();
  auto_reconnect_ = this->get_parameter("auto_reconnect").as_bool();

  // Setup parameter callback first
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HkCameraNode::parametersCallback, this, std::placeholders::_1));

  // Initialize camera
  if (!initCamera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
    // 设置节点为失败状态，不创建发布器和定时器
    is_connected_ = false;
    return;
  }

  // Setup image publisher only after successful camera initialization
  image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 1);
  
  // Setup actual frame rate publisher only after successful camera initialization
  actual_frame_rate_publisher_ = this->create_publisher<std_msgs::msg::Float64>("~/actual_frame_rate", 10);
  
  // Image transport is disabled to avoid shared_from_this issues
  use_transport_ = false;

  RCLCPP_INFO(this->get_logger(), "HK Camera node initialized successfully");
}

HkCameraNode::~HkCameraNode()
{
  should_stop_ = true;
  if (image_thread_.joinable()) {
    image_thread_.join();
  }
  // Timer is automatically destroyed when node is destroyed
  
  stopGrabbing();
  disconnectCamera();
  
  RCLCPP_INFO(this->get_logger(), "HK Camera node destroyed");
}

bool HkCameraNode::initCamera()
{
  // Initialize MV SDK
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  
  // Enumerate devices
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Enum devices failed: %08x", nRet);
    return false;
  }
  
  if (device_list.nDeviceNum == 0) {
    RCLCPP_ERROR(this->get_logger(), "No devices found");
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Found %d devices", device_list.nDeviceNum);
  
  // Find target device
  int device_index = -1;
  for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
    MV_CC_DEVICE_INFO* device_info = device_list.pDeviceInfo[i];
    if (device_info->nTLayerType == MV_GIGE_DEVICE) {
      // Check IP address
      if (!device_ip_.empty()) {
        unsigned int nIp1 = ((device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        unsigned int nIp2 = ((device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        unsigned int nIp3 = ((device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        unsigned int nIp4 = (device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        
        char current_ip[32];
        sprintf(current_ip, "%d.%d.%d.%d", nIp1, nIp2, nIp3, nIp4);
        
        if (device_ip_ == std::string(current_ip)) {
          device_index = i;
          break;
        }
      }
    } else if (device_info->nTLayerType == MV_USB_DEVICE) {
      // Check serial number
      if (!device_serial_.empty()) {
        if (device_serial_ == std::string((char*)device_info->SpecialInfo.stUsb3VInfo.chSerialNumber)) {
          device_index = i;
          break;
        }
      }
    }
  }
  
  // If no specific device found, use the first one
  if (device_index == -1) {
    device_index = 0;
    RCLCPP_WARN(this->get_logger(), "No specific device found, using the first device");
  }
  
  // Open device
  nRet = MV_CC_CreateHandle(&device_handle_, device_list.pDeviceInfo[device_index]);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Create handle failed: %08x", nRet);
    return false;
  }
  
  // Connect device
  nRet = MV_CC_OpenDevice(device_handle_, MV_ACCESS_Exclusive, 0);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Open device failed: %08x", nRet);
    MV_CC_DestroyHandle(device_handle_);
    device_handle_ = nullptr;
    return false;
  }
  
  is_connected_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera connected successfully");
  
  // Configure camera
  if (!configureCamera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure camera");
    disconnectCamera();
    return false;
  }
  
  // Start grabbing
  if (!startGrabbing()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start grabbing");
    disconnectCamera();
    return false;
  }
  
  // Start image acquisition thread
  should_stop_ = false;
  image_thread_ = std::thread(&HkCameraNode::getImageCallback, this);
  
  // Create timer for frame rate monitoring (1 Hz)
  frame_rate_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&HkCameraNode::frameRateTimerCallback, this));
  
  // Initialize frame rate calculation
  last_time_ = std::chrono::steady_clock::now();
  
  return true;
}

bool HkCameraNode::configureCamera()
{
  int nRet;
  
  // Set pixel format
  nRet = MV_CC_SetEnumValueByString(device_handle_, "PixelFormat", pixel_format_.c_str());
  if (MV_OK != nRet) {
    RCLCPP_WARN(this->get_logger(), "Set pixel format '%s' failed: %08x", pixel_format_.c_str(), nRet);
    
    // For color cameras, always try to set RGB8 first to avoid color issues
  std::vector<std::string> color_formats = {"RGB8", "BGR8"};
    
    for (const auto& format : color_formats) {
      nRet = MV_CC_SetEnumValueByString(device_handle_, "PixelFormat", format.c_str());
      if (MV_OK == nRet) {
        pixel_format_ = format;
        RCLCPP_INFO(this->get_logger(), "Successfully set pixel format to: %s", format.c_str());
        break;
      }
    }
    
    if (MV_OK != nRet) {
      // If RGB8/BGR8 not supported, try Bayer formats
      std::vector<std::string> bayer_formats = {"BayerRG8", "BayerBG8", "BayerGB8", "BayerGR8"};
      for (const auto& format : bayer_formats) {
        nRet = MV_CC_SetEnumValueByString(device_handle_, "PixelFormat", format.c_str());
        if (MV_OK == nRet) {
          pixel_format_ = format;
          RCLCPP_INFO(this->get_logger(), "Using Bayer format: %s (will be converted to RGB8)", format.c_str());
          break;
        }
      }
    }
    
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set any pixel format");
      return false;
    }
  }
  
  // Set image size
  nRet = MV_CC_SetIntValue(device_handle_, "Width", image_width_);
  if (MV_OK != nRet) {
    RCLCPP_WARN(this->get_logger(), "Set width failed: %08x", nRet);
  }
  
  nRet = MV_CC_SetIntValue(device_handle_, "Height", image_height_);
  if (MV_OK != nRet) {
    RCLCPP_WARN(this->get_logger(), "Set height failed: %08x", nRet);
  }
  
  // Set exposure time
  nRet = MV_CC_SetFloatValue(device_handle_, "ExposureTime", exposure_time_);
  if (MV_OK != nRet) {
    RCLCPP_WARN(this->get_logger(), "Set exposure time failed: %08x", nRet);
  }
  
  // Set gain
  nRet = MV_CC_SetFloatValue(device_handle_, "Gain", gain_);
  if (MV_OK != nRet) {
    RCLCPP_WARN(this->get_logger(), "Set gain failed: %08x", nRet);
  }
  
  // Set frame rate
  nRet = MV_CC_SetFloatValue(device_handle_, "AcquisitionFrameRate", frame_rate_);
  if (MV_OK != nRet) {
    RCLCPP_WARN(this->get_logger(), "Set frame rate failed: %08x", nRet);
  }
  
  return true;
}

bool HkCameraNode::startGrabbing()
{
  int nRet = MV_CC_StartGrabbing(device_handle_);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Start grabbing failed: %08x", nRet);
    return false;
  }
  
  is_grabbing_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera started grabbing");
  return true;
}

bool HkCameraNode::stopGrabbing()
{
  if (!is_grabbing_) {
    return true;
  }
  
  int nRet = MV_CC_StopGrabbing(device_handle_);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Stop grabbing failed: %08x", nRet);
    return false;
  }
  
  is_grabbing_ = false;
  RCLCPP_INFO(this->get_logger(), "Camera stopped grabbing");
  return true;
}

bool HkCameraNode::disconnectCamera()
{
  if (!is_connected_) {
    return true;
  }
  
  int nRet = MV_CC_CloseDevice(device_handle_);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Close device failed: %08x", nRet);
    return false;
  }
  
  nRet = MV_CC_DestroyHandle(device_handle_);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Destroy handle failed: %08x", nRet);
    return false;
  }
  
  device_handle_ = nullptr;
  is_connected_ = false;
  RCLCPP_INFO(this->get_logger(), "Camera disconnected");
  return true;
}

void HkCameraNode::getImageCallback()
{
  cv::Mat image;
  int error_count = 0;
  const int max_errors = 10;
  
  while (!should_stop_ && rclcpp::ok()) {
    if (getOneImage(image)) {
      error_count = 0;  // Reset error count on success
      if (!image.empty()) {
        publishImage(image);
      }
    } else {
      error_count++;
      if (error_count >= max_errors) {
        RCLCPP_ERROR(this->get_logger(), "Too many consecutive errors (%d), stopping image acquisition", max_errors);
        break;
      }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool HkCameraNode::getOneImage(cv::Mat & image)
{
  if (!is_connected_ || !is_grabbing_) {
    return false;
  }
  
  std::lock_guard<std::mutex> lock(camera_mutex_);
  
  MV_FRAME_OUT_INFO_EX frame_info;
  memset(&frame_info, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  
  // Get image buffer size first
  MVCC_INTVALUE stIntValue;
  memset(&stIntValue, 0, sizeof(MVCC_INTVALUE));
  int nRet = MV_CC_GetIntValue(device_handle_, "PayloadSize", &stIntValue);
  if (MV_OK != nRet) {
    RCLCPP_WARN(this->get_logger(), "Get payload size failed: %08x", nRet);
    return false;
  }
  
  unsigned int nDataSize = stIntValue.nCurValue;
  unsigned char * pData = (unsigned char*)malloc(nDataSize);
  if (pData == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for image");
    return false;
  }
  
  nRet = MV_CC_GetOneFrameTimeout(device_handle_, pData, nDataSize, &frame_info, 1000);
  if (MV_OK != nRet) {
    free(pData);
    if (nRet == MV_E_GC_TIMEOUT) {
      // Timeout is normal when no image is available
      return false;
    } else if (nRet == 0x80000007) {
      // Buffer underrun or camera internal issue, try to recover
      RCLCPP_WARN(this->get_logger(), "Camera buffer underrun (0x80000007), trying to recover...");
      
      // Try to flush the camera buffer
      int flush_ret = MV_CC_ClearImageBuffer(device_handle_);
      if (flush_ret == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Camera buffer cleared successfully");
      }
      return false;
    } else {
      RCLCPP_WARN(this->get_logger(), "Get one frame failed: %08x", nRet);
      return false;
    }
  }
  
  // Convert to OpenCV Mat
               
  try {
    if (frame_info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
      image = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC3, pData).clone();
      RCLCPP_DEBUG(this->get_logger(), "BGR8 image: %dx%d, step: %zu", 
                   image.cols, image.rows, image.step[0]);
    } else if (frame_info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
      cv::Mat rgb_image(frame_info.nHeight, frame_info.nWidth, CV_8UC3, pData);
      cv::cvtColor(rgb_image, image, cv::COLOR_RGB2BGR);
      RCLCPP_DEBUG(this->get_logger(), "RGB8 converted to BGR8 image: %dx%d, step: %zu", 
                   image.cols, image.rows, image.step[0]);
    } else if (frame_info.enPixelType == PixelType_Gvsp_Mono8) {
      image = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC1, pData).clone();
      RCLCPP_DEBUG(this->get_logger(), "Mono8 image: %dx%d, step: %zu", 
                   image.cols, image.rows, image.step[0]);
    } else if (frame_info.enPixelType == PixelType_Gvsp_BayerRG8) {
      cv::Mat bayer_image(frame_info.nHeight, frame_info.nWidth, CV_8UC1, pData);
      // 先转换为BGR，然后手动交换R和B通道
      cv::Mat bgr_image;
      cv::cvtColor(bayer_image, bgr_image, cv::COLOR_BayerRG2BGR);
      
      // 创建RGB图像并交换通道
      cv::Mat channels[3];
      cv::split(bgr_image, channels);
      // 交换R和B通道: BGR -> RGB
      cv::Mat rgb_channels[3] = {channels[0], channels[1], channels[2]}; // B, G, R
      cv::merge(rgb_channels, 3, image);
      
      
    } else if (frame_info.enPixelType == PixelType_Gvsp_BayerBG8) {
      cv::Mat bayer_image(frame_info.nHeight, frame_info.nWidth, CV_8UC1, pData);
      cv::cvtColor(bayer_image, image, cv::COLOR_BayerBG2BGR);
      RCLCPP_DEBUG(this->get_logger(), "BayerBG8 converted to BGR8 image: %dx%d, step: %zu", 
                   image.cols, image.rows, image.step[0]);
    } else if (frame_info.enPixelType == PixelType_Gvsp_BayerGB8) {
      cv::Mat bayer_image(frame_info.nHeight, frame_info.nWidth, CV_8UC1, pData);
      cv::cvtColor(bayer_image, image, cv::COLOR_BayerGB2BGR);
      RCLCPP_DEBUG(this->get_logger(), "BayerGB8 converted to BGR8 image: %dx%d, step: %zu", 
                   image.cols, image.rows, image.step[0]);
    } else if (frame_info.enPixelType == PixelType_Gvsp_BayerGR8) {
      cv::Mat bayer_image(frame_info.nHeight, frame_info.nWidth, CV_8UC1, pData);
      cv::cvtColor(bayer_image, image, cv::COLOR_BayerGR2BGR);
      RCLCPP_DEBUG(this->get_logger(), "BayerGR8 converted to BGR8 image: %dx%d, step: %zu", 
                   image.cols, image.rows, image.step[0]);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported pixel format: %ld", frame_info.enPixelType);
      free(pData);
      return false;
    }
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    free(pData);
    return false;
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown exception during image conversion");
    free(pData);
    return false;
  }
  
  free(pData);
  return true;
}

void HkCameraNode::publishImage(const cv::Mat & image)
{
  if (image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty image received, not publishing");
    return;
  }
  
  // Increment frame counter for FPS calculation
  {
    std::lock_guard<std::mutex> lock(frame_rate_mutex_);
    frame_count_++;
  }
  
  // Create image message manually to ensure correct step
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = this->now();
  msg->header.frame_id = frame_id_;
  msg->height = image.rows;
  msg->width = image.cols;
  
  if (image.type() == CV_8UC3) {
    msg->encoding = "rgb8";
    msg->step = image.cols * 3; // 3 bytes per pixel for RGB8
  } else if (image.type() == CV_8UC1) {
    msg->encoding = "mono8";
    msg->step = image.cols; // 1 byte per pixel for Mono8
  } else {
    RCLCPP_WARN(this->get_logger(), "Unsupported OpenCV image type: %d", image.type());
    return;
  }
  
  // Copy image data
  size_t data_size = msg->step * msg->height;
  msg->data.resize(data_size);
  memcpy(msg->data.data(), image.data, data_size);
  
  RCLCPP_DEBUG(this->get_logger(), "Publishing image: %dx%d, step: %d, encoding: %s, data size: %zu", 
               msg->width, msg->height, msg->step, msg->encoding.c_str(), msg->data.size());
  
  // Always use regular publisher to avoid shared_from_this issues
  image_publisher_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult HkCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  
  for (auto param : parameters) {
    if (param.get_name() == "exposure_time") {
      exposure_time_ = param.as_double();
      if (is_connected_) {
        int nRet = MV_CC_SetFloatValue(device_handle_, "ExposureTime", exposure_time_);
        if (MV_OK != nRet) {
          result.successful = false;
          result.reason = "Failed to set exposure time";
        }
      }
    } else if (param.get_name() == "gain") {
      gain_ = param.as_double();
      if (is_connected_) {
        int nRet = MV_CC_SetFloatValue(device_handle_, "Gain", gain_);
        if (MV_OK != nRet) {
          result.successful = false;
          result.reason = "Failed to set gain";
        }
      }
    } else if (param.get_name() == "frame_rate") {
      frame_rate_ = param.as_double();
      if (is_connected_) {
        int nRet = MV_CC_SetFloatValue(device_handle_, "AcquisitionFrameRate", frame_rate_);
        if (MV_OK != nRet) {
          result.successful = false;
          result.reason = "Failed to set frame rate";
        }
      }
    }
  }
  
  return result;
}

std::string HkCameraNode::getPixelFormatString(MvGvspPixelType pixel_format)
{
  switch (pixel_format) {
    case PixelType_Gvsp_Mono8:
      return "Mono8";
    case PixelType_Gvsp_RGB8_Packed:
      return "RGB8";
    case PixelType_Gvsp_BGR8_Packed:
      return "BGR8";
    default:
      return "Unknown";
  }
}

MvGvspPixelType HkCameraNode::getPixelFormatEnum(const std::string & pixel_format_str)
{
  if (pixel_format_str == "Mono8") {
    return PixelType_Gvsp_Mono8;
  } else if (pixel_format_str == "RGB8") {
    return PixelType_Gvsp_RGB8_Packed;
  } else if (pixel_format_str == "BGR8") {
    return PixelType_Gvsp_BGR8_Packed;
  } else {
    return PixelType_Gvsp_BGR8_Packed;  // Default
  }
}

double HkCameraNode::getActualFrameRate()
{
  if (!is_connected_) {
    RCLCPP_WARN(this->get_logger(), "Camera not connected, cannot get actual frame rate");
    return 0.0;
  }
  
  MVCC_FLOATVALUE float_value;
  memset(&float_value, 0, sizeof(MVCC_FLOATVALUE));
  
  // Try to get the actual frame rate, also known as resulting frame rate
  int nRet = MV_CC_GetFloatValue(device_handle_, "ResultingFrameRate", &float_value);
  if (MV_OK != nRet) {
    // If ResultingFrameRate is not available, try AcquisitionFrameRateActual
    nRet = MV_CC_GetFloatValue(device_handle_, "AcquisitionFrameRateActual", &float_value);
    if (MV_OK != nRet) {
      RCLCPP_DEBUG(this->get_logger(), "Failed to get actual frame rate: %08x", nRet);
      return 0.0;
    }
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Actual frame rate: %.2f fps", float_value.fCurValue);
  return float_value.fCurValue;
}

double HkCameraNode::getCurrentFrameRate()
{
  if (!is_connected_) {
    RCLCPP_WARN(this->get_logger(), "Camera not connected, cannot get current frame rate");
    return 0.0;
  }
  
  MVCC_FLOATVALUE float_value;
  memset(&float_value, 0, sizeof(MVCC_FLOATVALUE));
  
  // Get the current frame rate setting
  int nRet = MV_CC_GetFloatValue(device_handle_, "AcquisitionFrameRate", &float_value);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Failed to get current frame rate: %08x", nRet);
    return 0.0;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Current frame rate setting: %.2f fps", float_value.fCurValue);
  return float_value.fCurValue;
}

void HkCameraNode::frameRateTimerCallback()
{
  if (is_connected_ && is_grabbing_ && actual_frame_rate_publisher_) {
    // Calculate frame rate based on actual frame count
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(frame_rate_mutex_);
    
    // Calculate time difference in seconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_);
    double time_diff = duration.count() / 1000.0;
    
    if (time_diff > 0.9) {  // Use 1 second intervals for more stable readings
      // Calculate FPS
      double instant_fps = frame_count_ / time_diff;
      
      // Apply simple low-pass filter to smooth out fluctuations
      const double alpha = 0.3; // Smoothing factor (0.0 = no change, 1.0 = no smoothing)
      calculated_fps_ = alpha * instant_fps + (1.0 - alpha) * calculated_fps_;
      
      // Reset counters
      frame_count_ = 0;
      last_time_ = now;
      
      // Publish calculated FPS
      auto msg = std::make_unique<std_msgs::msg::Float64>();
      msg->data = calculated_fps_;
      actual_frame_rate_publisher_->publish(std::move(msg));
      
      RCLCPP_DEBUG(this->get_logger(), "Published calculated frame rate: %.2f fps (instant: %.2f)", 
                   calculated_fps_, instant_fps);
    }
  } else if (!is_connected_ && actual_frame_rate_publisher_) {
    // If camera is not connected, publish 0 fps
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = 0.0;
    actual_frame_rate_publisher_->publish(std::move(msg));
    RCLCPP_DEBUG(this->get_logger(), "Camera not connected, publishing 0 fps");
  }
}

}  // namespace hk_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hk_camera::HkCameraNode)
