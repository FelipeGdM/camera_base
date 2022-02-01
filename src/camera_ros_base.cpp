#include "camera_base/camera_ros_base.hpp"

namespace camera_base {

CameraRosBase::CameraRosBase(rclcpp::Node::SharedPtr node,
                             const std::string& frame_id,
                             const std::string& identifier,
                             const std::string& camera_name,
                             const std::string& calib_url,
                             const std::string& prefix)
    : it_(node),
      camera_pub_(it_.advertiseCamera("image_raw", 1)),
      cinfo_mgr_(node.get(), camera_name, calib_url),
      fps_(10.0),
      frame_id_(frame_id),
      identifier_(identifier),
      diagnostic_updater_(node),
      topic_diagnostic_(
          prefix.empty() ? "image_raw" : (prefix + "/image_raw"),
          diagnostic_updater_,
          diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
          diagnostic_updater::TimeStampStatusParam(-0.01, 0.1)) {}

void CameraRosBase::SetHardwareId(const std::string& id) {
  diagnostic_updater_.setHardwareID(id);
}

void CameraRosBase::PublishCamera(const rclcpp::Time& time) {
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

void CameraRosBase::Publish(
    const sensor_msgs::msg::Image::SharedPtr& image_msg) {
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

}  // namespace camera_base
