/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// ROS includes
// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// IMU Sensor message
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
namespace sensor_msgs {
typedef msg::Image Image;
}  // namespace sensor_msgs

// FSW includes
#include <config_reader/config_reader.h>

// STL includes
#include <string>

namespace gazebo {
FF_DEFINE_LOGGER("gazebo_sensor_plugin_dock_cam");
class GazeboSensorPluginDockCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginDockCam() : FreeFlyerSensorPlugin("dock_cam", "dock_cam", true), rate_(0.0) {}

  ~GazeboSensorPluginDockCam() {
    if (update_) {
#if GAZEBO_MAJOR_VERSION > 7
      update_.reset();
#else
      sensor_->DisconnectUpdated(update_);
#endif
    }
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(NodeHandle& nh, sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginDockCam requires a parent camera sensor.\n";
      return;
    }

    // Check that we have a mono camera
    if (sensor_->Camera()->ImageFormat() != "L8") FF_FATAL_STREAM("Camera format must be L8");

    // Set image constants
    msg_.is_bigendian = false;
    msg_.header.frame_id = GetFrame();
    msg_.encoding = sensor_msgs::image_encodings::MONO8;

    // Create a publisher
    pub_img_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::Image, TOPIC_HARDWARE_DOCK_CAM, 1);

    // Read configuration
    config_reader::ConfigReader config;
    config.AddFile("simulation/simulation.config");
    if (!config.ReadFiles()) {
      FF_FATAL("Failed to read simulation config file.");
      return;
    }
    bool dos = true;
    if (!config.GetBool("disable_cameras_on_speedup", &dos)) FF_FATAL("Could not read the drawing_width parameter.");
    if (!config.GetReal("dock_cam_rate", &rate_)) FF_FATAL("Could not read the drawing_width parameter.");
    config.Close();

    // If we have a sped up simulation and we need to disable the camera
    double simulation_speed = 1.0;
    if (nh->get_parameter("/simulation_speed", simulation_speed))
      if (simulation_speed > 1.0 && dos) rate_ = 0.0;

    // Toggle if the camera is active or not
    timer_toggle_.createTimer(0.5,
      std::bind(&GazeboSensorPluginDockCam::ToggleCallback, this), nh, false, true);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(NodeHandle& nh) {
    // Connect to the camera update event.
    update_ = sensor_->ConnectUpdated(std::bind(&GazeboSensorPluginDockCam::UpdateCallback, this));
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    if (pub_img_->get_subscription_count() > 0 && rate_ > 0) {
      sensor_->SetUpdateRate(rate_);
      sensor_->SetActive(true);
    } else {
      sensor_->SetUpdateRate(0.0001);
      sensor_->SetActive(false);
    }
  }

  // Called on each sensor update event
  void UpdateCallback() {
    msg_.header.stamp = GetTimeNow();
    msg_.height = sensor_->ImageHeight();
    msg_.width = sensor_->ImageWidth();
    msg_.step = msg_.width;
    msg_.data.resize(msg_.step * msg_.height);
    std::copy(reinterpret_cast<const uint8_t*>(sensor_->ImageData()),
              reinterpret_cast<const uint8_t*>(sensor_->ImageData()) + msg_.step * msg_.height, msg_.data.begin());
    pub_img_->publish(msg_);
  }

 private:
  ff_util::FreeFlyerTimer timer_toggle_;
  sensor_msgs::Image msg_;
  rclcpp::Publisher<sensor_msgs::Image>::SharedPtr pub_img_;
  std::shared_ptr<sensors::WideAngleCameraSensor> sensor_;
  event::ConnectionPtr update_;
  double rate_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginDockCam)

}  // namespace gazebo
