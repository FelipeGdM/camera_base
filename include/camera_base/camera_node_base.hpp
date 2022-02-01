#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "camera_base/camera_ros_base.hpp"

namespace camera_base {

/**
 * @brief The CameraNodeBase class
 * A base class that implements a ros node for a camera
 */
template <typename ConfigType>
class CameraNodeBase : public rclcpp::Node {
 public:
  explicit CameraNodeBase(const std::string& node_name);

  CameraNodeBase() = delete;
  CameraNodeBase(const CameraNodeBase&) = delete;
  CameraNodeBase& operator=(const CameraNodeBase&) = delete;
  virtual ~CameraNodeBase() = default;

  bool is_acquire() const { return is_acquire_; }

  /**
   * @brief Run Run the node
   * This will setup the dynamic reconfigure server, this will start the
   * acquisition automatically when the server is initialized
   */
  void Run();

  /**
   * @brief End
   */
  void End();

  void Sleep() const;

  /**
   * @brief ConfigCb Dynamic reconfigure callback
   * @param config Config type
   * @param level Reconfigure level, not really used
   * Entering this callback will stop the acquisition thread, do the
   * reconfiguration and restart acquisition thread
   */
  void ConfigCb(ConfigType& config);

  /**
   * @brief Acquire Do acquisition here
   */
  virtual void Acquire() = 0;

  /**
   * @brief Setup Setup your camera here
   * @param config Config type
   */
  virtual void Setup(ConfigType& config) = 0;

 private:
  void SetRate(double fps);

  void Start();

  void Stop();

  bool is_acquire_;
  std::unique_ptr<rclcpp::Rate> rate_;
  std::unique_ptr<std::thread> acquire_thread_;
};

}  // namespace camera_base
