#include "camera_base/camera_node_base.hpp"

namespace camera_base {

template <typename ConfigType>
CameraNodeBase<ConfigType>::CameraNodeBase(const std::string& node_name)
    : Node(node_name), is_acquire_(false) {}

template <typename ConfigType>
void CameraNodeBase<ConfigType>::Run() {
  ConfigCb();
}

template <typename ConfigType>
void CameraNodeBase<ConfigType>::End() {
  Stop();
}

template <typename ConfigType>
void CameraNodeBase<ConfigType>::Sleep() const {
  rate_->sleep();
}

template <typename ConfigType>
void CameraNodeBase<ConfigType>::ConfigCb(ConfigType& config) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing reconfigure server");
  if (is_acquire()) {
    Stop();
  }
  Setup(config);
  SetRate(config.fps);
  Start();
}

template <typename ConfigType>
void CameraNodeBase<ConfigType>::SetRate(double fps) {
  rate_.reset(new rclcpp::Rate(fps));
}

template <typename ConfigType>
void CameraNodeBase<ConfigType>::Start() {
  is_acquire_ = true;
  acquire_thread_.reset(new std::thread(&CameraNodeBase::Acquire, this));
}

template <typename ConfigType>
void CameraNodeBase<ConfigType>::Stop() {
  if (!is_acquire_) return;
  is_acquire_ = false;
  acquire_thread_->join();
}

}  // namespace camera_base
