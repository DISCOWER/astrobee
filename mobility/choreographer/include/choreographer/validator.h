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

#ifndef CHOREOGRAPHER_VALIDATOR_H_
#define CHOREOGRAPHER_VALIDATOR_H_

// ROS libraries
#include <ff_common/ff_ros.h>

// ROS messages
#include <visualization_msgs/msg/marker_array.hpp>

// FSW utils
#include <ff_util/ff_flight.h>
#include <ff_util/ff_serialization.h>
#include <ff_util/ff_service.h>
#include <ff_util/config_server.h>

// FSW messages
#include <ff_msgs/msg/zone.hpp>
#include <ff_msgs/srv/set_zones.hpp>
#include <ff_msgs/srv/get_zones.hpp>
#include <ff_msgs/srv/get_occupancy_map.hpp>
#include <ff_msgs/srv/get_float.hpp>

// Voxel map
#include <jps3d/planner/jps_3d_util.h>

// STL includes
#include <string>
#include <vector>

#define EPS 0.001

namespace choreographer {

class Validator {
 public:
  // Possible responses
  enum Response : uint8_t {
    SUCCESS,
    VIOLATES_RESAMPLING,
    VIOLATES_KEEP_OUT,
    VIOLATES_KEEP_IN,
    VIOLATES_MINIMUM_FREQUENCY,
    VIOLATES_STATIONARY_ENDPOINT,
    VIOLATES_FIRST_IN_PAST,
    VIOLATES_MINIMUM_SETPOINTS,
    VIOLATES_HARD_LIMIT_VEL,
    VIOLATES_HARD_LIMIT_ACCEL,
    VIOLATES_HARD_LIMIT_OMEGA,
    VIOLATES_HARD_LIMIT_ALPHA
  };

  // Load the keep in the keeo out zones and return if successful
  bool Init(NodeHandle& nh, ff_util::ConfigServer & cfg);

  // If the check fails, then the info block is populated
  Response CheckSegment(ff_util::Segment const& msg,
    ff_msgs::msg::FlightMode const& flight_mode, bool face_forward);

 protected:
  // Markers for keep in / keep out zones
  void PublishMarkers();

  // Process zones when building the occupancy map
  void ProcessZone(std::vector<signed char>& map, int type, char cell_value, bool surface);

  // Build the occupancy map
  bool GetZonesMap();

  // Callback to get the keep in/out zones
  void GetZonesCallback(const std::shared_ptr<ff_msgs::srv::GetZones::Request> req,
                        std::shared_ptr<ff_msgs::srv::GetZones::Response> res);

  // Callback to get the keep in/out zones map
  void GetZonesMapCallback(const std::shared_ptr<ff_msgs::srv::GetOccupancyMap::Request> req,
                       std::shared_ptr<ff_msgs::srv::GetOccupancyMap::Response> res);

  // Callback to set the keep in/out zones
  void SetZonesCallback(const std::shared_ptr<ff_msgs::srv::SetZones::Request> req,
                       std::shared_ptr<ff_msgs::srv::SetZones::Response> res);

 private:
  NodeHandle* nh_;
  std::string zone_file_;                             // Zone file path
  bool overwrite_;                                    // New zones overwrite
  ff_msgs::srv::SetZones::Request zones_;             // Zones
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  pub_zones_;                  // Zone publisher
  rclcpp::Service<ff_msgs::srv::GetZones>::SharedPtr get_zones_srv_;             // Get zones service
  rclcpp::Service<ff_msgs::srv::GetOccupancyMap>::SharedPtr get_zones_map_srv_;  // Get zones map
  rclcpp::Service<ff_msgs::srv::SetZones>::SharedPtr set_zones_srv_;             // Set zones service
  ff_util::FreeFlyerServiceClient<ff_msgs::srv::GetFloat> get_resolution_;             // Get the zones map resolution
  ff_util::FreeFlyerServiceClient<ff_msgs::srv::GetFloat> get_map_inflation_;          // Get the zones map inflation

  double map_res_ = 0.08;
  std::shared_ptr<JPS::VoxelMapUtil> jps_map_util_;

  // Voxel map values
  char val_occ_ = 100;         // Assume occupied cell has value 100
  char val_free_ = 0;          // Assume free cell has value 0
  char val_unknown_ = -1;      // Assume unknown cell has value -1
};

}  // namespace choreographer

#endif  // CHOREOGRAPHER_VALIDATOR_H_
