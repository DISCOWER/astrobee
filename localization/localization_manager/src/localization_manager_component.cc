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

// Standard includes
#include <rclcpp/rclcpp.hpp>

// Common includes
#include <ff_util/ff_component.h>
#include <ff_util/ff_faults.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_service.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/config_server.h>
#include <ff_util/conversion.h>

// Standard messages
#include <std_srvs/srv/empty.hpp>
namespace std_srvs {
  typedef srv::Empty Empty;
}  // namespace std_srvs

// Messages we use
#include <ff_msgs/srv/set_bool.hpp>
#include <ff_msgs/srv/set_state.hpp>
#include <ff_msgs/srv/set_ekf_input.hpp>
#include <ff_msgs/msg/localization_state.hpp>
#include <ff_msgs/action/localization.hpp>
#include <ff_msgs/srv/get_pipelines.hpp>
namespace ff_msgs {
  typedef srv::SetBool SetBool;
  typedef srv::SetState SetState;
  typedef srv::SetEkfInput SetEkfInput;
  typedef msg::LocalizationState LocalizationState;
  typedef action::Localization Localization;
  typedef srv::GetPipelines GetPipelines;
}  // namespace ff_msgs

// This application header
#include <localization_manager/localization_pipeline.h>

// STL includes
#include <functional>
#include <string>
#include <map>

/**
 * \ingroup localization
 */
namespace localization_manager {

FF_DEFINE_LOGGER("localization_manager");

// Match the internal states and responses with the message definition
using FSM = ff_util::FSM;
using STATE = ff_msgs::LocalizationState;
using RESPONSE = ff_msgs::Localization::Result;

// The manager nodelet for switching between localization modes
class LocalizationManagerComponent : public ff_util::FreeFlyerComponent {
 public:
  // Finite state machine
  enum Event : FSM::Event {
    READY                  = (1<<0),    // All pipelines initialized
    GOAL_CANCEL            = (1<<1),    // Cancel the operation in process
    GOAL_PREEMPT           = (1<<2),    // Preemption request
    GOAL_SWITCH_PIPELINE   = (1<<3),    // New request
    GOAL_ESTIMATE_BIAS     = (1<<4),    // New request
    GOAL_RESET_FILTER      = (1<<5),    // New request
    CURRENT_UNSTABLE       = (1<<6),    // Current pipeline is unstable
    GOAL_UNSTABLE          = (1<<7),    // Goal pipeline is unstable
    STABLE                 = (1<<8),    // Filter/pipeline is stable
    TIMEOUT                = (1<<9),    // Could not complete the action
    MANUAL_STATE_SET       = (1<<10)    // Setting the state manually with service
  };

  // Constructor
  explicit LocalizationManagerComponent(const rclcpp::NodeOptions& options) :
    ff_util::FreeFlyerComponent(options, NODE_LOCALIZATION_MANAGER, true),
      fsm_(STATE::INITIALIZING, std::bind(&LocalizationManagerComponent::UpdateCallback,
        this, std::placeholders::_1, std::placeholders::_2)) {
    // When all services appear, then turn on optical flow and pipeline. We
    // need to activate the correct pipeline
    fsm_.Add(STATE::INITIALIZING,
      READY, [this](FSM::Event const& event) -> FSM::State {
        // Turn on optical flow
        if (!OpticalFlow(curr_->second.RequiresOpticalFlow())) {
          AssertFault(ff_util::INITIALIZATION_FAILED,
            "Could not initialize optical flow for the default pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Enable the pipeline
        if (!curr_->second.Toggle(true)) {
          AssertFault(ff_util::INITIALIZATION_FAILED,
            "Could not enable teh default pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Try and switch to the pipeline
        if (!SwitchFilterInput(curr_->second.GetMode())) {
          AssertFault(ff_util::INITIALIZATION_FAILED,
            "Could not switch to the default pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Now start using the new pipeline
        if (!curr_->second.Use(true)) {
          AssertFault(ff_util::INITIALIZATION_FAILED,
            "Could not start using the default pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Now return the correct default state
        goal_ = curr_;
        // Return the correct state
        if (curr_->first == ff_msgs::Localization::Goal::PIPELINE_NONE)
          return STATE::DISABLED;
        return STATE::LOCALIZING;
      });

    // GENERAL PIPELINE MONITORING PATTERN

    // We are busy localizing and the pipeline goes unstable -- we need to
    // react by switching to the fallback pipeline.
    fsm_.Add(STATE::LOCALIZING,
      CURRENT_UNSTABLE, [this](FSM::Event const& event) -> FSM::State {
        // If we are on the fallback pipeline and it is unstable, then we can't
        // really do anything except issue a fault...
        if (curr_ == fall_) {
          AssertFault(ff_util::LOCALIZATION_PIPELINE_UNSTABLE,
            "Currently on fallback pipeline and it is unstable", GetTimeNow());
          ResetTimer(timer_recovery_);
          return STATE::UNSTABLE;
        }
        // All other case is a fallback
        FF_DEBUG("Pipeline unstable. Falling back to safe pipeline");
        // Turn on optical flow
        if (fall_->second.RequiresOpticalFlow() && !OpticalFlow(true)) {
          AssertFault(ff_util::LOCALIZATION_PIPELINE_UNSTABLE,
            "Could not toggle optical flow on begin", GetTimeNow());
          return STATE::DISABLED;
        }
        // Enable the pipeline
        if (!fall_->second.Toggle(true)) {
          AssertFault(ff_util::LOCALIZATION_PIPELINE_UNSTABLE,
            "Could not enable fallback pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Try and switch to the pipeline
        if (!SwitchFilterInput(fall_->second.GetMode())) {
          AssertFault(ff_util::LOCALIZATION_PIPELINE_UNSTABLE,
            "Could not switch to fallback pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Now start using the new pipeline
        if (!fall_->second.Use(true)) {
          AssertFault(ff_util::INITIALIZATION_FAILED,
            "Could not start using the default pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Disable the active the pipeline
        if (!curr_->second.Toggle(false)) {
          AssertFault(ff_util::LOCALIZATION_PIPELINE_UNSTABLE,
            "Could not disable fallback pipeline", GetTimeNow());
          return STATE::DISABLED;
        }
        // Turn off optical flow if it's not required
        if (!OpticalFlow(fall_->second.RequiresOpticalFlow())) {
          AssertFault(ff_util::LOCALIZATION_PIPELINE_UNSTABLE,
            "Could not toggle optical flow on end", GetTimeNow());
          return STATE::DISABLED;
        }
        // Set all pipelines to fallback
        curr_ = fall_;
        goal_ = fall_;
        // Return the correct state
        if (curr_->first == ff_msgs::Localization::Goal::PIPELINE_NONE)
          return STATE::DISABLED;
        return STATE::LOCALIZING;
      });

    // In an unstable state we keep getting marked as unstable
    fsm_.Add(STATE::UNSTABLE,
      CURRENT_UNSTABLE, [this](FSM::Event const& event) -> FSM::State {
        ResetTimer(timer_recovery_);
        return STATE::UNSTABLE;
      });

    // In an unstable state we are finally marked as stable
    fsm_.Add(STATE::UNSTABLE,
      STABLE, [this](FSM::Event const& event) -> FSM::State {
      ClearFault(ff_util::LOCALIZATION_PIPELINE_UNSTABLE);
      return STATE::LOCALIZING;
      });

    // SWITCH PIPELINE PATTERN

    // We are disabled or busy localizing and we get a new switch pipeline event
    // We enable the pipeline and possibly optical flow, and wait for stability
    fsm_.Add(STATE::LOCALIZING, STATE::DISABLED, STATE::UNSTABLE,
      GOAL_SWITCH_PIPELINE, [this](FSM::Event const& event) -> FSM::State {
        if (goal_->second.RequiresOpticalFlow() && !OpticalFlow(true))
          return Result(RESPONSE::OPTICAL_FLOW_FAILED, "Could not toggle optical flow");
        if (!goal_->second.Toggle(true))
          return Result(RESPONSE::PIPELINE_TOGGLE_FAILED, "Could not enable new pipeline");
        ResetTimer(timer_stability_);
        ResetTimer(timer_deadline_);
        return STATE::SWITCH_WAITING_FOR_PIPELINE;
      });

    // Waiting for the goal pipeline to go stable, and it does. We need to
    // switch the EKF to use the new pipeline and wait for the filter.
    fsm_.Add(STATE::SWITCH_WAITING_FOR_PIPELINE,
      STABLE, [this](FSM::Event const& event) -> FSM::State {
        // Try and switch to the pipeline
        if (!SwitchFilterInput(goal_->second.GetMode()))
          return Result(RESPONSE::SET_INPUT_FAILED, "Could not switch to new pipeline");
        // Now activate the pipeline
        if (!goal_->second.Use(true))
          return Result(RESPONSE::PIPELINE_USE_FAILED, "Could not start using new pipeline");
        ResetTimer(timer_stability_);
        ResetTimer(timer_deadline_);
        return STATE::SWITCH_WAITING_FOR_FILTER;
      });

    // Waiting for pipeline stability and the pipeline reports being unstable.
    // We need to reset the stability timer
    fsm_.Add(STATE::SWITCH_WAITING_FOR_PIPELINE,
      GOAL_UNSTABLE, [this](FSM::Event const& event) -> FSM::State {
        ResetTimer(timer_stability_);
        return STATE::SWITCH_WAITING_FOR_PIPELINE;
      });

    // Waiting for the goal pipeline to go stable, and it does not. We need to
    // gracefully disable the goal pipeline and return an error
    fsm_.Add(STATE::SWITCH_WAITING_FOR_PIPELINE,
      TIMEOUT, [this](FSM::Event const& event) -> FSM::State {
        // Disable the goal pipeline
        if (!goal_->second.Toggle(false))
          return Result(RESPONSE::PIPELINE_TOGGLE_FAILED, "Could not enable new pipeline");
        // Possibly also disable optical flow
        if (!OpticalFlow(curr_->second.RequiresOpticalFlow()))
          return Result(RESPONSE::OPTICAL_FLOW_FAILED, "Could not toggle optical flow");
        // Return the goal pipeline to the current pipeline
        goal_ = curr_;
        // Return the error
        return Result(RESPONSE::PIPELINE_UNSTABLE, "Pipeline took too long to stabilize pre-switch");
      });

    // Waiting for the goal pipeline to go stable, and it does. We need to
    // switch the EKF to use the new pipeline and wait for the filter.
    fsm_.Add(STATE::SWITCH_WAITING_FOR_FILTER,
      STABLE, [this](FSM::Event const& event) -> FSM::State {
        // Disable the goal pipeline
        if (!curr_->second.Toggle(false))
          return Result(RESPONSE::PIPELINE_TOGGLE_FAILED, "Could not disable old pipeline");
        // Possibly also disable optical flow
        if (!OpticalFlow(goal_->second.RequiresOpticalFlow()))
          return Result(RESPONSE::OPTICAL_FLOW_FAILED, "Could not toggle optical flow");
        // Return the goal pipeline to the current pipeline
        curr_ = goal_;
        std::string msg = "Switched to pipeline: " + curr_->first;
        // Return the error
        return Result(RESPONSE::SUCCESS, msg);
      });

    // Waiting for the reset and the watchdog fires
    fsm_.Add(STATE::SWITCH_WAITING_FOR_FILTER,
      GOAL_UNSTABLE, [this](FSM::Event const& event) -> FSM::State {
        ResetTimer(timer_stability_);
        return STATE::SWITCH_WAITING_FOR_FILTER;
      });

    // Waiting for the goal pipeline to go stable, and it does not. We need to
    // gracefully turn off the goal pipeline and return an error
    fsm_.Add(STATE::SWITCH_WAITING_FOR_FILTER,
      TIMEOUT, [this](FSM::Event const& event) -> FSM::State {
        // Try and switch to the pipeline
        if (!SwitchFilterInput(curr_->second.GetMode()))
          return Result(RESPONSE::SET_INPUT_FAILED, "Could not switch back to old pipeline");
        // Disable the goal pipeline
        if (!goal_->second.Toggle(false))
          return Result(RESPONSE::PIPELINE_TOGGLE_FAILED, "Could not disable new pipeline");
        // Possibly also disable optical flow
        if (!OpticalFlow(curr_->second.RequiresOpticalFlow()))
          return Result(RESPONSE::OPTICAL_FLOW_FAILED, "Could not toggle optical flow");
        // Return the goal pipeline to the current pipeline
        goal_ = curr_;
        // Return the error
        return Result(RESPONSE::PIPELINE_UNSTABLE, "Pipeline took too long to stabilize post-switch");
      });

    //  BIAS ESTIMATION PATTERN

    // Localizing normally and bias estimate command is called
    fsm_.Add(STATE::LOCALIZING, STATE::DISABLED, STATE::UNSTABLE,
      GOAL_ESTIMATE_BIAS, [this](FSM::Event const& event) -> FSM::State {
        if (!EstimateBias())
          return Result(RESPONSE::ESTIMATE_BIAS_FAILED, "Could not call bias estimation service");
        ResetTimer(timer_stability_);
        ResetTimer(timer_deadline_);
        return STATE::BIAS_WAITING_FOR_FILTER;
      });

    // Waiting for bias estimation to complete and we get stable notification
    fsm_.Add(STATE::BIAS_WAITING_FOR_FILTER,
      STABLE, [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS, "Bias estimation completed successfully");
      });

    // Waiting for bias estimation to complete and we get an unsable notification
    fsm_.Add(STATE::BIAS_WAITING_FOR_FILTER,
      CURRENT_UNSTABLE, [this](FSM::Event const& event) -> FSM::State {
        ResetTimer(timer_stability_);
        return STATE::BIAS_WAITING_FOR_FILTER;
      });

    // Waiting for bias estimation and we exceed the deadline
    fsm_.Add(STATE::BIAS_WAITING_FOR_FILTER,
      TIMEOUT, [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::PIPELINE_UNSTABLE, "Pipeline took too long to stabilize after bias");
      });

    // FILTER RESET PATTERN

    // Busy localizing and a filter reset command is called
    fsm_.Add(STATE::LOCALIZING, STATE::UNSTABLE,
      GOAL_RESET_FILTER, [this](FSM::Event const& event) -> FSM::State {
        if (!ResetFilter())
          return Result(RESPONSE::RESET_FAILED, "Could not call filter reset service");
        ResetTimer(timer_stability_);
        ResetTimer(timer_deadline_);
        return STATE::RESET_WAITING_FOR_FILTER;
      });

    // Waiting for the reset to complete and we get stable notification
    fsm_.Add(STATE::RESET_WAITING_FOR_FILTER,
      STABLE, [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::SUCCESS, "Filter reset completed successfully");
      });

    // Waiting for the reset to complete and the EKF to stabilize
    fsm_.Add(STATE::RESET_WAITING_FOR_FILTER,
      CURRENT_UNSTABLE, [this](FSM::Event const& event) -> FSM::State {
        ResetTimer(timer_stability_);
        return STATE::RESET_WAITING_FOR_FILTER;
      });

    // Waiting for bias estimation and we exceed the deadline
    fsm_.Add(STATE::RESET_WAITING_FOR_FILTER,
      TIMEOUT, [this](FSM::Event const& event) -> FSM::State {
        return Result(RESPONSE::PIPELINE_UNSTABLE, "Pipeline took too long to stabilize after reset");
      });

    // CATCH-ALL FOR CANCELLATIONS / PREEMPTIONS

    // On cancel or preemption, notify previous callee and revert to LOCALIZING
    fsm_.Add(GOAL_CANCEL | GOAL_PREEMPT,
      [this](FSM::State const& state, FSM::Event const& event) -> FSM::State {
        if (event == GOAL_CANCEL)
          return Result(RESPONSE::CANCELLED, "User cancelled the operation");
        return Result(RESPONSE::PREEMPTED, "Third party preempted operation");
      });
  }

  // Destructor
  ~LocalizationManagerComponent() {}

 protected:
  void Initialize(NodeHandle &nh) {
    FF_DEBUG_STREAM("Initialize()");
    // Read the node parameters
    cfg_.AddFile("localization/localization_manager.config");
    if (!cfg_.Initialize(nh))
      return AssertFault(ff_util::INITIALIZATION_FAILED,
                  "Could not start config server", GetTimeNow());

    // Read and configure the pipelines
    config_reader::ConfigReader::Table pipelines(cfg_.GetConfigReader(), "pipelines");
    for (int i = 0; i < pipelines.GetSize(); i++) {
      config_reader::ConfigReader::Table pipeline;
      if (!pipelines.GetTable(i + 1, &pipeline))
        AssertFault(ff_util::INITIALIZATION_FAILED, "Could not get pipeline", GetTimeNow());
      // Get mandatory fields
      std::string id;
      if (!pipeline.GetStr("id", &id))
        AssertFault(ff_util::INITIALIZATION_FAILED, "Could not get id", GetTimeNow());
      std::string name;
      if (!pipeline.GetStr("name", &name))
        AssertFault(ff_util::INITIALIZATION_FAILED, "Could not get name", GetTimeNow());
      int mode;
      if (!pipeline.GetInt("ekf_input", &mode))
        AssertFault(ff_util::INITIALIZATION_FAILED, "Could not get ekf_input", GetTimeNow());

      // Allocate the pipeline
      Pipeline & p = pipelines_.emplace(id,
        Pipeline(static_cast<uint8_t>(mode), name)).first->second;

      // Filter requirement
      if (pipeline.CheckValExists("needs_filter")) {
        bool needs_filter = false;
        if (pipeline.GetBool("needs_filter", &needs_filter)) {
          if (needs_filter) {
            int max_confidence = 0;
            if (!pipeline.GetInt("max_confidence", &max_confidence)) {
              AssertFault(ff_util::INITIALIZATION_FAILED,
                            "Could not get max confidence", GetTimeNow());
            }

            bool optical_flow = false;
            if (!pipeline.GetBool("optical_flow", &optical_flow)) {
              AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Could not get optical flow", GetTimeNow());
            }

            double timeout = 1.0;
            if (!pipeline.GetReal("timeout", &timeout)) {
              AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Could not get timeout", GetTimeNow());
            }

            // Set a filter need
            p.NeedsFilter(max_confidence, optical_flow, timeout);
          }
        }
      }

      // Enable requirement
      if (pipeline.CheckValExists("enable_topic")) {
        std::string enable_topic;
        if (pipeline.GetStr("enable_topic", &enable_topic)) {
          double enable_timeout = 1.0;
          if (!pipeline.GetReal("enable_timeout", &enable_timeout)) {
            AssertFault(ff_util::INITIALIZATION_FAILED,
                        "Could not get enable_timeout", GetTimeNow());
          }

          // Set an enable need
          p.NeedsEnable(enable_topic, enable_timeout);
        }
      }

      // Registration requirement
      if (pipeline.CheckValExists("reg_topic")) {
        std::string reg_topic;
        if (pipeline.GetStr("reg_topic", &reg_topic)) {
          double reg_timeout = 1.0;
          if (!pipeline.GetReal("reg_timeout", &reg_timeout)) {
            AssertFault(ff_util::INITIALIZATION_FAILED,
                        "Could not get reg_timeout", GetTimeNow());
          }

          // Set an enable need
          p.NeedsRegistrations(reg_topic, reg_timeout);
        }
      }

      // Visual feature requirement
      if (pipeline.CheckValExists("feat_topic")) {
        std::string feat_topic;
        if (pipeline.GetStr("feat_topic", &feat_topic)) {
          double feat_timeout = 1.0;
          if (!pipeline.GetReal("feat_timeout", &feat_timeout)) {
            AssertFault(ff_util::INITIALIZATION_FAILED,
                        "Could not get feat_timeout", GetTimeNow());
          }

          int feat_threshold = 0;
          if (!pipeline.GetInt("feat_threshold", &feat_threshold)) {
            AssertFault(ff_util::INITIALIZATION_FAILED,
                        "Could not get feat_threshold", GetTimeNow());
          }

          // Set a feature need
          p.NeedsVisualFeatures(feat_topic, feat_timeout, feat_threshold);
        }
      }

      // Depth feature requirement
      if (pipeline.CheckValExists("depth_topic")) {
        std::string depth_topic;
        if (pipeline.GetStr("depth_topic", &depth_topic)) {
          double depth_timeout = 1.0;
          if (!pipeline.GetReal("depth_timeout", &depth_timeout)) {
            AssertFault(ff_util::INITIALIZATION_FAILED,
                        "Could not get depth_timeout", GetTimeNow());
          }

          int depth_threshold = 0;
          if (!pipeline.GetInt("depth_threshold", &depth_threshold)) {
            AssertFault(ff_util::INITIALIZATION_FAILED,
                        "Could not get depth_threshold", GetTimeNow());
          }

          // Set a feature need
          p.NeedsDepthFeatures(depth_topic, depth_timeout, depth_threshold);
        }
      }
    }

    // Set the fallback and current pipelines
    std::string pipeline;
    if (!cfg_.Get<std::string>("fallback", pipeline))
      AssertFault(ff_util::INITIALIZATION_FAILED,
        "No valid fallback pipeline specfified in the localization manager", GetTimeNow());
    fall_ = pipelines_.find(pipeline);
    if (fall_ == pipelines_.end())
      AssertFault(ff_util::INITIALIZATION_FAILED,
        "Fallback pipeline specified in config does not exist", GetTimeNow());
    if (!cfg_.Get<std::string>("pipeline", pipeline))
      AssertFault(ff_util::INITIALIZATION_FAILED,
        "No valid default pipeline specfified in the localization manager", GetTimeNow());
    curr_ = pipelines_.find(pipeline);
    if (curr_ == pipelines_.end())
      AssertFault(ff_util::INITIALIZATION_FAILED,
        "Default pipeline specified in config does not exist", GetTimeNow());
    goal_ = curr_;

    // Initialize all pipelines in the system
    for (auto & pipeline : pipelines_) {
      if (!pipeline.second.Initialize(nh,
        std::bind(&LocalizationManagerComponent::PipelineCallback, this,
          pipeline.first, std::placeholders::_1),
        std::bind(&LocalizationManagerComponent::ConnectedCallback, this),
        std::bind(&LocalizationManagerComponent::TimeoutCallback, this))) {
        AssertFault(ff_util::INITIALIZATION_FAILED,
          std::string("Could not init pipeline: ") + pipeline.first, GetTimeNow());
      }
    }

    // Publish the docking state as a latched topic
    pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::LocalizationState, TOPIC_LOCALIZATION_MANAGER_STATE, 1);

    // Set EKF input service
    service_i_.SetConnectedTimeout(cfg_.Get<double>("timeout_service_set_input"));
    service_i_.SetConnectedCallback(
      std::bind(&LocalizationManagerComponent::ConnectedCallback, this));
    service_i_.SetTimeoutCallback(
      std::bind(&LocalizationManagerComponent::TimeoutCallback, this));
    service_i_.Create(nh, SERVICE_GNC_EKF_SET_INPUT);

    // Enable optical flow service
    service_o_.SetConnectedTimeout(cfg_.Get<double>("timeout_service_enable_of"));
    service_o_.SetConnectedCallback(
      std::bind(&LocalizationManagerComponent::ConnectedCallback, this));
    service_o_.SetTimeoutCallback(
      std::bind(&LocalizationManagerComponent::TimeoutCallback, this));
    service_o_.Create(nh, SERVICE_LOCALIZATION_OF_ENABLE);

    // Initilize bias service
    service_b_.SetConnectedTimeout(cfg_.Get<double>("timeout_service_bias"));
    service_b_.SetConnectedCallback(
      std::bind(&LocalizationManagerComponent::ConnectedCallback, this));
    service_b_.SetTimeoutCallback(
      std::bind(&LocalizationManagerComponent::TimeoutCallback, this));
    service_b_.Create(nh, SERVICE_GNC_EKF_INIT_BIAS);

    // Reset EKF service
    service_r_.SetConnectedTimeout(cfg_.Get<double>("timeout_service_reset"));
    service_r_.SetConnectedCallback(
      std::bind(&LocalizationManagerComponent::ConnectedCallback, this));
    service_r_.SetTimeoutCallback(
      std::bind(&LocalizationManagerComponent::TimeoutCallback, this));
    service_r_.Create(nh, SERVICE_GNC_EKF_RESET);

    // Timers to check for EKF stability / instability
    timer_stability_.createTimer(cfg_.Get<double>("timeout_stability"),
        std::bind(&LocalizationManagerComponent::StabilityTimerCallback, this), nh, true, false);
    timer_recovery_.createTimer(cfg_.Get<double>("timeout_recovery"),
        std::bind(&LocalizationManagerComponent::StabilityTimerCallback, this), nh, true, false);
    timer_deadline_.createTimer(cfg_.Get<double>("timeout_deadline"),
        std::bind(&LocalizationManagerComponent::DeadlineTimerCallback, this), nh, true, false);

    // Allow the state to be manually set
    server_set_state_ = FF_CREATE_SERVICE(nh, ff_msgs::SetState,
      SERVICE_LOCALIZATION_MANAGER_SET_STATE,
        std::bind(&LocalizationManagerComponent::SetStateCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Allow the possible pipelines to be queried
    server_get_pipelines_ = FF_CREATE_SERVICE(nh, ff_msgs::GetPipelines,
      SERVICE_LOCALIZATION_MANAGER_GET_PIPELINES,
      std::bind(&LocalizationManagerComponent::GetPipelinesCallback,
      this, std::placeholders::_1, std::placeholders::_2));

    // Allow the current pipeline to be queried
    server_get_current_pipelines_ = FF_CREATE_SERVICE(nh, ff_msgs::GetPipelines,
      SERVICE_LOCALIZATION_MANAGER_GET_CURR_PIPELINE,
      std::bind(&LocalizationManagerComponent::GetCurrentPipelineCallback,
      this, std::placeholders::_1, std::placeholders::_2));

    // Create the switch action
    action_.SetGoalCallback(std::bind(&LocalizationManagerComponent::GoalCallback, this, std::placeholders::_1));
    action_.SetPreemptCallback(std::bind(&LocalizationManagerComponent::PreemptCallback, this));
    action_.SetCancelCallback(std::bind(&LocalizationManagerComponent::CancelCallback, this));
    action_.Create(nh, ACTION_LOCALIZATION_MANAGER_LOCALIZATION);
  }

  // Enumerate all pipelines
  std::map<std::string, std::string> EnumeratePipelines() {
    std::map<std::string, std::string> enumeration;
    for (auto & pipeline : pipelines_) {
      enumeration[pipeline.first] = pipeline.second.GetName();
      FF_DEBUG_STREAM(pipeline.first << " " << pipeline.second.GetName());
    }
    return enumeration;
  }

  // Ensure all clients are connected
  void ConnectedCallback() {
    FF_DEBUG_STREAM("ConnectedCallback()");
    if (!service_i_.IsConnected()) return;     // Check set input service
    if (!service_o_.IsConnected()) return;     // Check optical flow service
    if (!service_r_.IsConnected()) return;     // Check EKF reset service
    if (!service_b_.IsConnected()) return;     // Check EKF init biase
    for (auto & pipeline : pipelines_)         // Check all pipelines
      if (!pipeline.second.IsConnected()) return;
    fsm_.Update(READY);                        // Ready!
  }

  // Timeout on a trajectory generation request
  void TimeoutCallback() {
    FF_DEBUG_STREAM("TimeoutCallback()");
    AssertFault(ff_util::INITIALIZATION_FAILED,
      "One of the manager or pipeline services failed to appear", GetTimeNow());
  }

  // Called when a user manually updates the internal state
  bool SetStateCallback(const std::shared_ptr<ff_msgs::SetState::Request> req,
                        std::shared_ptr<ff_msgs::SetState::Response> res) {
    FF_DEBUG_STREAM("SetStateCallback()");
    fsm_.SetState(req->state);
    res->success = true;
    UpdateCallback(fsm_.GetState(), MANUAL_STATE_SET);
    return true;
  }

  // Called when a user requests the internal state
  bool GetCurrentPipelineCallback(const std::shared_ptr<ff_msgs::GetPipelines::Request> req,
                            std::shared_ptr<ff_msgs::GetPipelines::Response> res) {
    FF_DEBUG_STREAM("GetPipelinesCallback()");
    ff_msgs::LocalizationPipeline msg;
    msg.id = curr_->first;
    msg.mode = curr_->second.GetMode();
    msg.name = curr_->second.GetName();
    msg.requires_optical_flow = curr_->second.RequiresOpticalFlow();
    msg.requires_filter = curr_->second.RequiresFilter();
    res->pipelines.push_back(msg);
    return true;
  }

  // Called when a user manually updates the internal state
  bool GetPipelinesCallback(const std::shared_ptr<ff_msgs::GetPipelines::Request> req,
                            std::shared_ptr<ff_msgs::GetPipelines::Response> res) {
    FF_DEBUG_STREAM("GetPipelinesCallback()");
    ff_msgs::LocalizationPipeline msg;
    for (auto & pipeline : pipelines_) {
      msg.id = pipeline.first;
      msg.mode = pipeline.second.GetMode();
      msg.name = pipeline.second.GetName();
      msg.requires_optical_flow = pipeline.second.RequiresOpticalFlow();
      msg.requires_filter = pipeline.second.RequiresFilter();
      res->pipelines.push_back(msg);
    }
    return true;
  }

  // Complete the current dock or undock action
  FSM::State Result(int32_t response, std::string const& msg = "") {
    FF_DEBUG_STREAM("Result(" << response << "): " << msg);
    FF_DEBUG_STREAM("Current pipeline is " << curr_->first);
    // Always return the goal pipeline to the current pipeline
    goal_ = curr_;
    // Send the feedback if needed
    switch (fsm_.GetState()) {
    case STATE::INITIALIZING:
    case STATE::LOCALIZING:
    case STATE::DISABLED:
      FF_DEBUG("Result called but action is not being tracked");
      break;
    default: {
        auto result = std::make_shared<ff_msgs::Localization::Result>();
        result->fsm_result = msg;
        result->response = response;
        if (response > 0)
          action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        else if (response < 0)
          action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        else
          action_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
      }
      break;
    }
    // If the current state is "no localization", then localization is disabled
    if (curr_->first == ff_msgs::Localization::Goal::PIPELINE_NONE)
      return STATE::DISABLED;
    // All other states are localizing states
    return STATE::LOCALIZING;
  }

  // When the FSM state changes we get a callback here, so that we
  // can choose to do various things.
  void UpdateCallback(FSM::State const& state, FSM::Event const& event) {
    // Debug events
    ff_msgs::LocalizationState msg;
    msg.header.frame_id = GetPlatform();
    msg.header.stamp = GetTimeNow();
    msg.state = state;
    // Info about the current pipeline
    msg.pipeline.id = curr_->first;
    msg.pipeline.mode = curr_->second.GetMode();
    msg.pipeline.name = curr_->second.GetName();
    msg.pipeline.requires_optical_flow = curr_->second.RequiresOpticalFlow();
    msg.pipeline.requires_filter = curr_->second.RequiresFilter();
    // Debug events
    switch (event) {
    case READY:                msg.fsm_event = "READY";                 break;
    case GOAL_CANCEL:          msg.fsm_event = "GOAL_CANCEL";           break;
    case GOAL_PREEMPT:         msg.fsm_event = "GOAL_PREEMPT";          break;
    case GOAL_SWITCH_PIPELINE: msg.fsm_event = "GOAL_SWITCH_PIPELINE";  break;
    case GOAL_ESTIMATE_BIAS:   msg.fsm_event = "GOAL_ESTIMATE_BIAS";    break;
    case GOAL_RESET_FILTER:    msg.fsm_event = "GOAL_RESET_FILTER";     break;
    case CURRENT_UNSTABLE:     msg.fsm_event = "CURRENT_UNSTABLE";      break;
    case GOAL_UNSTABLE:        msg.fsm_event = "GOAL_UNSTABLE";         break;
    case STABLE:               msg.fsm_event = "STABLE";                break;
    case TIMEOUT:              msg.fsm_event = "TIMEOUT";               break;
    case MANUAL_STATE_SET:     msg.fsm_event = "MANUAL_STATE_SET";      break;
    }
    FF_DEBUG_STREAM("Received event " << msg.fsm_event);
    // Debug state changes
    switch (state) {
    case STATE::INITIALIZING:
      msg.fsm_state = "INITIALIZING";                       break;
    case STATE::DISABLED:
      msg.fsm_state = "DISABLED";                           break;
    case STATE::LOCALIZING:
      msg.fsm_state = "LOCALIZING";                         break;
    case STATE::SWITCH_WAITING_FOR_PIPELINE:
      msg.fsm_state = "SWITCH_WAITING_FOR_PIPELINE";        break;
    case STATE::SWITCH_WAITING_FOR_FILTER:
      msg.fsm_state = "SWITCH_WAITING_FOR_FILTER";          break;
    case STATE::BIAS_WAITING_FOR_FILTER:
      msg.fsm_state = "BIAS_WAITING_FOR_FILTER";            break;
    case STATE::RESET_WAITING_FOR_FILTER:
      msg.fsm_state = "RESET_WAITING_FOR_FILTER";           break;
    case STATE::UNSTABLE:
      msg.fsm_state = "UNSTABLE";                           break;
    }
    FF_DEBUG_STREAM("State changed to " << msg.fsm_state);
    // Broadcast the docking state
    pub_->publish(msg);
    // Send the feedback if needed
    switch (state) {
    case STATE::INITIALIZING:
    case STATE::DISABLED:
    case STATE::LOCALIZING:
    case STATE::UNSTABLE:
      return;
    default:
      break;
    }
    // If we get here then we have an active goal running, so send feedback
    auto feedback = std::make_shared<ff_msgs::Localization::Feedback>();
    feedback->state = msg;
    action_.SendFeedback(feedback);
  }

  // Called when any of the pipelines
  void PipelineCallback(std::string const& name, uint8_t error) {
    FF_DEBUG_STREAM("PipelineCallback("  << name << ")");
    // Work out which pipeline is giving problems
    bool current = false;
    if (pipelines_.find(name) == curr_)
      current = true;
    else if (pipelines_.find(name) == goal_)
      current = false;
    else
      return;
    // Create a pipeline name programatically
    std::string pipeline = std::string("Localization unstable: Pipeline ")
      + name + (current ? " [CURRENT] " : " [GOAL] ");
    // Debug output for the node
    switch (error) {
    case ERROR_REGISTRATION_TIMEOUT:
      FF_DEBUG_STREAM(pipeline << ": ERROR_REGISTRATION_TIMEOUT"); break;
    case ERROR_VISUAL_TIMEOUT:
      FF_DEBUG_STREAM(pipeline << ": ERROR_VISUAL_TIMEOUT");       break;
    case ERROR_DEPTH_TIMEOUT:
      FF_DEBUG_STREAM(pipeline << ": ERROR_DEPTH_TIMEOUT");        break;
    case ERROR_FILTER_TIMEOUT:
      FF_DEBUG_STREAM(pipeline << ": ERROR_FILTER_TIMEOUT");       break;
    }
    // Advance the state machine
    return fsm_.Update(current ? CURRENT_UNSTABLE : GOAL_UNSTABLE);
  }

  // Called when the localization mode must be switched
  void GoalCallback(std::shared_ptr<const ff_msgs::Localization::Goal> goal) {
    auto result = std::make_shared<ff_msgs::Localization::Result>();
    switch (goal->command) {
    case ff_msgs::Localization::Goal::COMMAND_SWITCH_PIPELINE: {
      FF_DEBUG_STREAM("GoalCallback Switch pipeline: " << goal->pipeline);
      PipelineMap::iterator arg = pipelines_.find(goal->pipeline);
      if (arg == pipelines_.end()) {
        result->fsm_result = "Invalid pipeline in request";
        result->response = RESPONSE::INVALID_PIPELINE;
        action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
        return;
      }
      if (arg == curr_) {
        result->fsm_result = "We are already on this pipeline";
        result->response = RESPONSE::PIPELINE_ALREADY_ACTIVE;
        action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
        return;
      }
      goal_ = arg;
      return fsm_.Update(GOAL_SWITCH_PIPELINE);
    }
    case ff_msgs::Localization::Goal::COMMAND_ESTIMATE_BIAS:
      FF_DEBUG_STREAM("GoalCallback estimate bias.");
      return fsm_.Update(GOAL_ESTIMATE_BIAS);
      break;
    case ff_msgs::Localization::Goal::COMMAND_RESET_FILTER:
      FF_DEBUG_STREAM("GoalCallback reset filter.");
      if (curr_->second.RequiresFilter())
        return fsm_.Update(GOAL_RESET_FILTER);
      break;
    default:
      FF_DEBUG_STREAM("GoalCallback invalid localization goal.");
      result->fsm_result = "Invalid command";
      result->response = RESPONSE::INVALID_COMMAND;
      action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
      return;
    }
    // Catch-all for filter actions in a non-filtering state
    result->fsm_result = "Cannot reset filter when not in use";
    result->response = RESPONSE::FILTER_NOT_IN_USE;
    action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
  }

  // Called when a switch request is preempted by another switch request
  void PreemptCallback() {
    FF_DEBUG_STREAM("CancelCallback()");
    fsm_.Update(GOAL_CANCEL);
  }

  // Called when a switch request is preempted by another switch request
  void CancelCallback() {
    FF_DEBUG_STREAM("PreemptCallback()");
    fsm_.Update(GOAL_PREEMPT);
  }

  // Estimate bias
  bool EstimateBias() {
    std_srvs::Empty::Request req;
    auto res = std::make_shared<std_srvs::Empty::Response>();
    return service_b_.Call(req, res);
  }

  // Reset filter
  bool ResetFilter() {
    std_srvs::Empty::Request req;
    auto res = std::make_shared<std_srvs::Empty::Response>();
    return service_r_.Call(req, res);
  }

  // Enable or disable optical flow
  bool OpticalFlow(bool enable) {
    ff_msgs::SetBool::Request req;
    req.enable = enable;
    auto res = std::make_shared<ff_msgs::SetBool::Response>();
    return service_o_.Call(req, res);
  }

  // Set the EKF to use the goal localizatin pipeline
  bool SwitchFilterInput(uint8_t mode) {
    ff_msgs::SetEkfInput::Request req;
    req.mode = mode;
    auto res = std::make_shared<ff_msgs::SetEkfInput::Response>();
    return service_i_.Call(req, res);
  }

  // Start a watchdog timer to expire after a given number of seconds
  void ResetTimer(ff_util::FreeFlyerTimer &timer) {
    timer.stop();
    timer.start();
  }

  // Called if the filter or pipeline is stable after
  void StabilityTimerCallback() {
    fsm_.Update(STABLE);
  }

  // Called
  void DeadlineTimerCallback() {
    fsm_.Update(TIMEOUT);
  }

 private:
  ff_util::FSM fsm_;
  ff_util::ConfigServer cfg_;
  ff_util::FreeFlyerServiceClient<ff_msgs::SetEkfInput> service_i_;
  ff_util::FreeFlyerServiceClient<ff_msgs::SetBool> service_o_;
  ff_util::FreeFlyerServiceClient<std_srvs::Empty> service_b_;
  ff_util::FreeFlyerServiceClient<std_srvs::Empty> service_r_;
  ff_util::FreeFlyerActionServer<ff_msgs::Localization> action_;
  rclcpp::Publisher<ff_msgs::LocalizationState>::SharedPtr pub_;
  ff_util::FreeFlyerTimer timer_deadline_, timer_recovery_, timer_stability_;
  rclcpp::Service<ff_msgs::SetState>::SharedPtr server_set_state_;
  rclcpp::Service<ff_msgs::GetPipelines>::SharedPtr server_get_pipelines_;
  rclcpp::Service<ff_msgs::GetPipelines>::SharedPtr server_get_current_pipelines_;
  PipelineMap pipelines_;
  PipelineMap::iterator curr_, goal_, fall_;
};

}  // namespace localization_manager

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(localization_manager::LocalizationManagerComponent)
