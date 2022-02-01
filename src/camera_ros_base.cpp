#include "camera_base/camera_ros_base.hpp"

namespace camera_base {

CameraRosBase::CameraRosBase(const ros::NodeHandle& pnh,
                             const std::string& prefix = std::string())
    : pnh_(pnh),
      cnh_(pnh, prefix),
      it_(cnh_),
      camera_pub_(it_.advertiseCamera("image_raw", 1)),
      cinfo_mgr_(cnh_, getParam<std::string>(cnh_, "camera_name"),
                 getParam<std::string>(cnh_, "calib_url")),
      fps_(10.0),
      diagnostic_updater_(pnh_, cnh_),
      topic_diagnostic_(
          prefix.empty() ? "image_raw" : (prefix + "/image_raw"),
          diagnostic_updater_,
          diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
          diagnostic_updater::TimeStampStatusParam(-0.01, 0.1)) {
  cnh_.param<std::string>("frame_id", frame_id_, cnh_.getNamespace());
  cnh_.param<std::string>("identifier", identifier_, "");
}

void CameraRosBase::SetHardwareId(const std::string& id) {
  diagnostic_updater_.setHardwareID(id);
}

void CameraRosBase::PublishCamera(const ros::Time& time) {
  const auto image_msg = std::make_shared<sensor_msgs::Image>();
  const auto cinfo_msg =
      std::make_shared<sensor_msgs::CameraInfo>(cinfo_mgr_.getCameraInfo());
  image_msg->header.frame_id = frame_id_;
  image_msg->header.stamp = time;
  if (Grab(image_msg, cinfo_msg)) {
    // Update camera info header
    cinfo_msg->header = image_msg->header;
    camera_pub_.publish(image_msg, cinfo_msg);
    topic_diagnostic_.tick(image_msg->header.stamp);
  }
  diagnostic_updater_.update();
}

void CameraRosBase::Publish(const sensor_msgs::ImagePtr& image_msg) {
  const auto cinfo_msg =
      std::make_shared<sensor_msgs::CameraInfo>(cinfo_mgr_.getCameraInfo());
  // Update camera info header
  image_msg->header.frame_id = frame_id_;
  cinfo_msg->header = image_msg->header;
  camera_pub_.publish(image_msg, cinfo_msg);
  topic_diagnostic_.tick(image_msg->header.stamp);
  diagnostic_updater_.update();
}

}  // namespace camera_base
