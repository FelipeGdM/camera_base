#pragma once

#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace camera_base {
/**
 * @brief The CameraRosBase class
 * This class implements a ros camera
 */
class CameraRosBase {
 public:
  explicit CameraRosBase(rclcpp::Node::SharedPtr node,
                         const std::string& frame_id_ = std::string(),
                         const std::string& identifier_ = std::string(),
                         const std::string& camera_name = std::string(),
                         const std::string& calib_url = std::string(),
                         const std::string& prefix = std::string())
      : it_(node),
        camera_pub_(it_.advertiseCamera("image_raw", 1)),
        cinfo_mgr_(node.get(), camera_name, calib_url),
        fps_(10.0),
        frame_id_(frame_id_),
        identifier_(identifier_),
        diagnostic_updater_(node),
        topic_diagnostic_(
            prefix.empty() ? "image_raw" : (prefix + "/image_raw"),
            diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-0.01, 0.1)){};

  CameraRosBase() = delete;
  CameraRosBase(const CameraRosBase&) = delete;
  CameraRosBase& operator=(const CameraRosBase&) = delete;
  virtual ~CameraRosBase() = default;

  const std::string& identifier() const { return identifier_; }
  const std::string& frame_id() const { return frame_id_; }

  double fps() const { return fps_; }
  void set_fps(double fps) { fps_ = fps; }

  /**
   * @brief SetHardwareId Set hardware id for diagnostic updater
   * @param id harware id
   */
  void SetHardwareId(const std::string& id) {
    diagnostic_updater_.setHardwareID(id);
  }

  /**
   * @brief PublishCamera Publish a camera topic with Image and CameraInfo
   * @param time Acquisition time stamp
   */
  void PublishCamera(const rclcpp::Time& time) {
    const auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    const auto cinfo_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
        cinfo_mgr_.getCameraInfo());
    image_msg->header.frame_id = frame_id_;
    image_msg->header.stamp = time;
    if (Grab(image_msg, cinfo_msg)) {
      // Update camera info header
      cinfo_msg->header = image_msg->header;
      camera_pub_.publish(image_msg, cinfo_msg);
      topic_diagnostic_.tick(image_msg->header.stamp);
    }

    // TODO! Should this be done?
    diagnostic_updater_.force_update();
  }

  void Publish(const sensor_msgs::msg::Image::SharedPtr& image_msg) {
    const auto cinfo_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
        cinfo_mgr_.getCameraInfo());
    // Update camera info header
    image_msg->header.frame_id = frame_id_;
    cinfo_msg->header = image_msg->header;
    camera_pub_.publish(image_msg, cinfo_msg);
    topic_diagnostic_.tick(image_msg->header.stamp);

    // TODO! Should this be done?
    diagnostic_updater_.force_update();
  }

  /**
   * @brief Grab Fill image_msg and cinfo_msg from low level camera driver
   * @param image_msg Ros message ImagePtr
   * @return True if successful
   */
  virtual bool Grab(
      const sensor_msgs::msg::Image::SharedPtr& image_msg,
      const sensor_msgs::msg::CameraInfo::SharedPtr& cinfo_msgs = nullptr) = 0;

 private:
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  camera_info_manager::CameraInfoManager cinfo_mgr_;
  double fps_;
  std::string frame_id_;
  std::string identifier_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::TopicDiagnostic topic_diagnostic_;
};

}  // namespace camera_base
