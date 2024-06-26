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

#ifndef CHOREOGRAPHER_PLANNER_H_
#define CHOREOGRAPHER_PLANNER_H_

// Standard includes
#include <ff_common/ff_ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// FSW libraries
#include <ff_common/ff_names.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_flight.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <ff_util/conversion.h>
#include <jsonloader/keepout.h>
// #include <mapper/point_cloud.h>

// FSW messages
#include <ff_msgs/action/plan.hpp>
#include <ff_msgs/msg/control_state.hpp>
#include <ff_msgs/msg/zone.hpp>
#include <ff_msgs/srv/register_planner.hpp>
#include <ff_msgs/srv/get_zones.hpp>
#include <ff_msgs/srv/get_occupancy_map.hpp>
#include <ff_msgs/srv/get_map.hpp>
namespace ff_msgs {
typedef action::Plan Plan;
typedef msg::Zone Zone;
typedef srv::GetZones GetZones;
typedef srv::GetOccupancyMap GetOccupancyMap;
typedef srv::RegisterPlanner RegisterPlanner;
typedef srv::GetMap GetMap;
}  // namespace ff_msgs

// Voxel map
#include <jps3d/planner/jps_3d_util.h>

#include <memory>
#include <string>
#include <vector>

/**
 * \ingroup mobility
 */
namespace planner {

// Convenience declarations
using RESPONSE = ff_msgs::Plan::Result;

// Abstract class for implementing planners
class PlannerImplementation : public ff_util::FreeFlyerComponent {
 private:
  enum State {
    INITIALIZING,
    WAITING,
    PLANNING
  };

 public:
  // The constructor takes the planner name and a brief description of it
  explicit PlannerImplementation(const rclcpp::NodeOptions& options, std::string const& name,
                                 std::string const& description)
      : ff_util::FreeFlyerComponent(options, std::string(PREFIX_MOBILITY_PLANNER_PRIVATE) + name, true),
        state_(INITIALIZING) {
    // Configure the registration details
    registration_.name = name;
    registration_.description = description;
    registration_.unregister = false;
  }

  // Destructor deregisters with choreographer
  virtual ~PlannerImplementation() {
    registration_.unregister = true;
    auto response = std::make_shared<ff_msgs::RegisterPlanner::Response>();
    client_r_.call(registration_, response);
  }

 protected:
  // Derived classes must implement these functions
  virtual bool InitializePlanner(NodeHandle &nh) = 0;
  virtual void PlanCallback(ff_msgs::Plan::Goal const& goal) = 0;
  virtual void Error(std::string out) = 0;
  virtual void Warn(std::string out) = 0;
  virtual void Debug(std::string out) = 0;

  // Send the planner result
  void PlanResult(ff_msgs::Plan::Result::SharedPtr const& result) {
    switch (state_) {
    case PLANNING:
      Debug("Plan result received");
      Complete(result->response, result);
      break;
    default:
      Warn("Plan result received in non-planning state");
      break;
    }
  }

  // Send some planner feedback
  void PlanFeedback(ff_msgs::Plan::Feedback::SharedPtr const& feedback) {
    switch (state_) {
    case PLANNING:
      server_p_.SendFeedback(feedback);
      break;
    default:
      Warn("Plan feedback received in non-planning state");
      break;
    }
  }

  // Get the keepin and keepout zones
  bool GetZones(std::vector < ff_msgs::Zone > & zones) {
    ff_msgs::GetZones::Request request;
    auto response = std::make_shared<ff_msgs::GetZones::Response>();
    client_z_.waitForExistence(5.0);
    if (client_z_.isValid() && client_z_.call(request, response)) {
      zones = response->zones;
      return true;
    }
    return false;
  }
  // Get the keepin and keepout zones
  bool GetZonesMap(std::vector<signed char> &map, Vec3f &origin, Vec3i &dim, double &map_res) {
    ff_msgs::GetOccupancyMap::Request request;
    auto response = std::make_shared<ff_msgs::GetOccupancyMap::Response>();
    client_z_m_.waitForExistence(5.0);
    if (client_z_m_.isValid() && client_z_m_.call(request, response)) {
      map.resize(response->map.size());
      map = response->map;
      origin[0] = response->origin.x;
      origin[1] = response->origin.y;
      origin[2] = response->origin.z;
      dim[0] = response->dim.x;
      dim[1] = response->dim.y;
      dim[2] = response->dim.z;
      map_res = response->resolution;
      return true;
    }
    return false;
  }
  bool GetFreeMap(pcl::PointCloud<pcl::PointXYZ> *points, float *resolution) {
    ff_msgs::GetMap::Request request;
    auto response = std::make_shared<ff_msgs::GetMap::Response>();
    client_f_.waitForExistence(5.0);
    if (client_f_.isValid() && client_f_.call(request, response)) {
      pcl::fromROSMsg(response->points, *points);
      *resolution = response->resolution;
      return true;
    }
    return false;
  }
  bool GetObstacleMap(pcl::PointCloud<pcl::PointXYZ> *points, float *resolution) {
    ff_msgs::GetMap::Request request;
    auto response = std::make_shared<ff_msgs::GetMap::Response>();
    client_o_.waitForExistence(5.0);
    if (client_o_.isValid() && client_o_.call(request, response)) {
      pcl::fromROSMsg(response->points, *points);
      *resolution = response->resolution;
      return true;
    }
    return false;
  }

 private:
  void Initialize(NodeHandle &nh) {
    cfg_fm_.AddFile("flight.config");
    cfg_fm_.ReadFiles();
    // Listen for plan requests
    server_p_.SetGoalCallback(std::bind(&PlannerImplementation::GoalCallback, this, std::placeholders::_1));
    server_p_.SetPreemptCallback(std::bind(&PlannerImplementation::PreemptCallback, this));
    server_p_.SetCancelCallback(std::bind(&PlannerImplementation::CancelCallback, this));
    std::string topic = std::string(PREFIX_MOBILITY_PLANNER)
                      + registration_.name
                      + std::string(SUFFIX_MOBILITY_PLANNER);
    server_p_.Create(nh, topic);
    // Initialize the get zone call
    client_z_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_z_.SetTimeoutCallback(std::bind(&PlannerImplementation::GetZonesTimeoutCallback, this));
    client_z_.Create(nh, SERVICE_MOBILITY_GET_ZONES);
    // Initialize the get zone map call
    client_z_m_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_z_m_.SetTimeoutCallback(std::bind(&PlannerImplementation::GetZonesMapTimeoutCallback, this));
    client_z_m_.Create(nh, SERVICE_MOBILITY_GET_ZONES_MAP);
    // Initialize the register planner call
    client_r_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_r_.SetTimeoutCallback(std::bind(&PlannerImplementation::RegisterTimeoutCallback, this));
    client_r_.Create(nh, SERVICE_MOBILITY_PLANNER_REGISTER);
    // Initialize the free map call
    client_f_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_f_.SetTimeoutCallback(std::bind(&PlannerImplementation::GetFreeMapTimeoutCallback, this));
    client_f_.Create(nh, SERVICE_MOBILITY_GET_FREE_MAP);
    // Initialize the obstacle map call
    client_o_.SetConnectedCallback(std::bind(&PlannerImplementation::ConnectedCallback, this));
    client_o_.SetTimeoutCallback(std::bind(&PlannerImplementation::GetObstacleMapTimeoutCallback, this));
    client_o_.Create(nh, SERVICE_MOBILITY_GET_OBSTACLE_MAP);
    // Initialize the planner itself
    if (!InitializePlanner(nh))
      InitFault("Planner could not be initialized");
  }

  // Deal with a fault in a responsible manner - note that this may also be
  // called if action and service servers timeout on conenction.
  void InitFault(std::string const& msg ) {
    Error(msg);
    AssertFault(ff_util::INITIALIZATION_FAILED, msg, GetTimeNow());
    return;
  }

  // Finish this action
  void Complete(int32_t response_code,
                ff_msgs::Plan::Result::SharedPtr result = ff_msgs::Plan::Result::SharedPtr()) {
    switch (state_) {
    case PLANNING:
      result->response = response_code;
      if (result->response > 0)
        server_p_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
      else if (result->response < 0)
        server_p_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      else
        server_p_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
      break;
    default:
      Warn("Plan result received in non-planning state");
      break;
    }
    // We are now back to waiting
    state_ = WAITING;
  }

  // Ensure all clients are connected
  void ConnectedCallback(void) {
    Debug("ConnectedCallback()");
    if (!client_z_.IsConnected()) return;    // Zones
    if (!client_z_m_.IsConnected()) return;  // Zones Map
    if (!client_r_.IsConnected()) return;    // Register
    if (!client_o_.IsConnected()) return;    // Register
    if (!client_f_.IsConnected()) return;    // Register
    if (state_ != INITIALIZING) return;      // Don't initialize twice
    // Register this planner
    Debug("Registering planner");
    auto response = std::make_shared<ff_msgs::RegisterPlanner::Response>();
    client_r_.waitForExistence(5.0);
    if (client_r_.isValid() && client_r_.call(registration_, response)) {}

    // Move to waiting state
    state_ = WAITING;
  }

  // Timeout on a trajectory generation request
  void RegisterTimeoutCallback(void) {
    return InitFault("Timeout connecting to register service");
  }

  // Timeout on a trajectory generation request
  void GetZonesTimeoutCallback(void) {
    return InitFault("Timeout connecting to the get zones service");
  }

  // Timeout on a trajectory generation request
  void GetZonesMapTimeoutCallback(void) {
    return InitFault("Timeout connecting to the get zones map service");
  }

  // Timeout on a free map request
  void GetFreeMapTimeoutCallback(void) {
    return InitFault("Timeout connecting to the get free map service");
  }

  // Timeout on a obstacle map request
  void GetObstacleMapTimeoutCallback(void) {
    return InitFault("Timeout connecting to the get obstacle map service");
  }

  // Check that the value is less than the given bound
  bool CheckBound(double candidate, double upper_bound) {
    return (candidate <= upper_bound);
  }

  // Get the value
  double GetValue(double candidate, double upper_bound) {
    if (candidate > 0.0 && candidate <= upper_bound)
      return candidate;
    return upper_bound;
  }

  // Called when a new planning goal arrives
  void GoalCallback(std::shared_ptr<const ff_msgs::Plan::Goal> const& old_goal) {
    Debug("A new plan request was just received");
    switch (state_) {
    default:
    case INITIALIZING:
      return Complete(RESPONSE::PROBLEM_CONNECTING_TO_SERVICES);
    case PLANNING:
      Complete(RESPONSE::PREEMPTED);
    case WAITING:
      break;
    }
    // We are now planning
    state_ = PLANNING;
    // Call the implementation with the plan goal
    return PlanCallback(*old_goal);
  }

  // Cancel the current operation
  void PreemptCallback(void) {
    return Complete(RESPONSE::PREEMPTED);
  }

  // Cancel the current operation
  void CancelCallback(void) {
    return Complete(RESPONSE::CANCELLED);
  }

 private:
  State state_;                                          // Planner state
  ff_util::FreeFlyerActionServer<ff_msgs::Plan> server_p_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetZones> client_z_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetOccupancyMap> client_z_m_;
  ff_util::FreeFlyerServiceClient<ff_msgs::RegisterPlanner> client_r_;
  ff_util::FreeFlyerServiceClient<ff_msgs::GetMap> client_f_, client_o_;
  ff_msgs::RegisterPlanner::Request registration_;                // Registration info
  config_reader::ConfigReader cfg_fm_;                   // Configuration
};

}  // namespace planner

#endif  // CHOREOGRAPHER_PLANNER_H_
