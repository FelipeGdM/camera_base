#include "camera_base/camera_node_base.hpp"

namespace camera_base {

template <typename ConfigType>
class CameraNodeBase {
  CameraNodeBase(const ros::NodeHandle& pnh)
      : is_acquire_(false), pnh_(pnh), cfg_server_(pnh) {}

  void Run() {
    cfg_server_.setCallback(
        std::bind(&CameraNodeBase<ConfigType>::ConfigCb, this, _1, _2));
  }

  void End() { Stop(); }

  void Sleep() const { rate_->sleep(); }

  void ConfigCb(ConfigType& config, int level) {
    if (level < 0) {
      ROS_INFO("%s: %s", pnh().getNamespace().c_str(),
               "Initializing reconfigure server");
    }
    if (is_acquire()) {
      Stop();
    }
    Setup(config);
    SetRate(config.fps);
    Start();
  }

  void SetRate(double fps) { rate_.reset(new ros::Rate(fps)); }

  void Start() {
    is_acquire_ = true;
    acquire_thread_.reset(new std::thread(&CameraNodeBase::Acquire, this));
  }

  void Stop() {
    if (!is_acquire_) return;
    is_acquire_ = false;
    acquire_thread_->join();
  }

}  // class CameraNodeBase

}  // namespace camera_base
