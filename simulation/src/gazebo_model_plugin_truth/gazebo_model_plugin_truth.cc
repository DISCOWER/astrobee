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

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// Tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Messages
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
namespace geometry_msgs {
typedef msg::TransformStamped TransformStamped;
typedef msg::PoseStamped PoseStamped;
typedef msg::TwistStamped TwistStamped;
}  // namespace geometry_msgs

// STL includes
#include <string>

namespace gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the force to be applied to the rigid body
class GazeboModelPluginTruth : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginTruth() :
    FreeFlyerModelPlugin("gazebo_truth", ""), rate_(62.5), tf_(true),
      pose_(true), twist_(true), static_(false),
        parent_("world"), child_("truth") {}

  ~GazeboModelPluginTruth() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle &nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    // If we specify a frame name different to our sensor tag name
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    if (sdf->HasElement("parent"))
      parent_ = sdf->Get<std::string>("parent");
    if (sdf->HasElement("child"))
      child_ = sdf->Get<std::string>("child");
    if (sdf->HasElement("pose"))
      pose_ = sdf->Get<bool>("pose");
    if (sdf->HasElement("twist"))
      twist_ = sdf->Get<bool>("twist");
    if (sdf->HasElement("tf"))
      tf_ = sdf->Get<bool>("tf");
    if (sdf->HasElement("static"))
      static_ = sdf->Get<bool>("static");

    // Setup TF2 message
    msg_.header.frame_id = parent_;
    msg_.child_frame_id = GetFrame(child_);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*nh);

    // If we are
    if (static_) {
      msg_.header.stamp = GetTimeNow();
      #if GAZEBO_MAJOR_VERSION > 7
        msg_.transform.translation.x = GetModel()->WorldPose().Pos().X();
        msg_.transform.translation.y = GetModel()->WorldPose().Pos().Y();
        msg_.transform.translation.z = GetModel()->WorldPose().Pos().Z();
        msg_.transform.rotation.x = GetModel()->WorldPose().Rot().X();
        msg_.transform.rotation.y = GetModel()->WorldPose().Rot().Y();
        msg_.transform.rotation.z = GetModel()->WorldPose().Rot().Z();
        msg_.transform.rotation.w = GetModel()->WorldPose().Rot().W();
      #else
        msg_.transform.translation.x = GetModel()->GetWorldPose().pos.x;
        msg_.transform.translation.y = GetModel()->GetWorldPose().pos.y;
        msg_.transform.translation.z = GetModel()->GetWorldPose().pos.z;
        msg_.transform.rotation.x = GetModel()->GetWorldPose().rot.x;
        msg_.transform.rotation.y = GetModel()->GetWorldPose().rot.y;
        msg_.transform.rotation.z = GetModel()->GetWorldPose().rot.z;
        msg_.transform.rotation.w = GetModel()->GetWorldPose().rot.w;
      #endif
      tf_broadcaster_->sendTransform(msg_);
      return;
    }

    // Ground truth
    pub_truth_pose_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::PoseStamped, TOPIC_LOCALIZATION_TRUTH, 1);
    pub_truth_twist_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::TwistStamped, TOPIC_LOCALIZATION_TRUTH_TWIST, 1);

    // Called before each iteration of simulated world update
    timer_.createTimer(1 / rate_,
      std::bind(&GazeboModelPluginTruth::TimerCallback, this), nh, false, true);
  }

  // Called on simulation reset
  void Reset() {}

  // Called on every discrete time tick in the simulated world
  void TimerCallback() {
    msg_.header.stamp = GetTimeNow();
    // If the rate is higher than the sim time, prevent repeated timestamps
    bool publish_tf = true;
    if (msg_.header.stamp == last_time_) {
      publish_tf = false;
    } else {
      last_time_ = msg_.header.stamp;
    }

    #if GAZEBO_MAJOR_VERSION > 7
    if (tf_ && publish_tf) {
      msg_.transform.translation.x = GetModel()->WorldPose().Pos().X();
      msg_.transform.translation.y = GetModel()->WorldPose().Pos().Y();
      msg_.transform.translation.z = GetModel()->WorldPose().Pos().Z();
      msg_.transform.rotation.x = GetModel()->WorldPose().Rot().X();
      msg_.transform.rotation.y = GetModel()->WorldPose().Rot().Y();
      msg_.transform.rotation.z = GetModel()->WorldPose().Rot().Z();
      msg_.transform.rotation.w = GetModel()->WorldPose().Rot().W();
      tf_broadcaster_->sendTransform(msg_);
    }
    // Pose
    if (pose_) {
      ros_truth_pose_.header = msg_.header;
      ros_truth_pose_.pose.position.x = GetModel()->WorldPose().Pos().X();
      ros_truth_pose_.pose.position.y = GetModel()->WorldPose().Pos().Y();
      ros_truth_pose_.pose.position.z = GetModel()->WorldPose().Pos().Z();
      ros_truth_pose_.pose.orientation.x = GetModel()->WorldPose().Rot().X();
      ros_truth_pose_.pose.orientation.y = GetModel()->WorldPose().Rot().Y();
      ros_truth_pose_.pose.orientation.z = GetModel()->WorldPose().Rot().Z();
      ros_truth_pose_.pose.orientation.w = GetModel()->WorldPose().Rot().W();
      pub_truth_pose_->publish(ros_truth_pose_);
    }
    // Twist
    if (twist_) {
      ros_truth_twist_.header = msg_.header;
      ros_truth_twist_.twist.linear.x = GetModel()->WorldLinearVel().X();
      ros_truth_twist_.twist.linear.y = GetModel()->WorldLinearVel().Y();
      ros_truth_twist_.twist.linear.z = GetModel()->WorldLinearVel().Z();
      ros_truth_twist_.twist.angular.x = GetModel()->RelativeAngularVel().X();
      ros_truth_twist_.twist.angular.y = GetModel()->RelativeAngularVel().Y();
      ros_truth_twist_.twist.angular.z = GetModel()->RelativeAngularVel().Z();
      pub_truth_twist_->publish(ros_truth_twist_);
    }
    #else
    if (tf_ && publish_tf) {
      msg_.transform.translation.x = GetModel()->GetWorldPose().pos.x;
      msg_.transform.translation.y = GetModel()->GetWorldPose().pos.y;
      msg_.transform.translation.z = GetModel()->GetWorldPose().pos.z;
      msg_.transform.rotation.x = GetModel()->GetWorldPose().rot.x;
      msg_.transform.rotation.y = GetModel()->GetWorldPose().rot.y;
      msg_.transform.rotation.z = GetModel()->GetWorldPose().rot.z;
      msg_.transform.rotation.w = GetModel()->GetWorldPose().rot.w;
      tf_broadcaster_->sendTransform(msg_);
    }
    // Pose
    if (pose_) {
      ros_truth_pose_.header = msg_.header;
      ros_truth_pose_.pose.position.x = GetModel()->GetWorldPose().pos.x;
      ros_truth_pose_.pose.position.y = GetModel()->GetWorldPose().pos.y;
      ros_truth_pose_.pose.position.z = GetModel()->GetWorldPose().pos.z;
      ros_truth_pose_.pose.orientation.x = GetModel()->GetWorldPose().rot.x;
      ros_truth_pose_.pose.orientation.y = GetModel()->GetWorldPose().rot.y;
      ros_truth_pose_.pose.orientation.z = GetModel()->GetWorldPose().rot.z;
      ros_truth_pose_.pose.orientation.w = GetModel()->GetWorldPose().rot.w;
      pub_truth_pose_->publish(ros_truth_pose_);
    }
    // Twist
    if (twist_) {
      ros_truth_twist_.header = msg_.header;
      ros_truth_twist_.twist.linear.x = GetModel()->GetWorldLinearVel().x;
      ros_truth_twist_.twist.linear.y = GetModel()->GetWorldLinearVel().y;
      ros_truth_twist_.twist.linear.z = GetModel()->GetWorldLinearVel().z;
      ros_truth_twist_.twist.angular.x = GetModel()->GetRelativeAngularVel().x;
      ros_truth_twist_.twist.angular.y = GetModel()->GetRelativeAngularVel().y;
      ros_truth_twist_.twist.angular.z = GetModel()->GetRelativeAngularVel().z;
      pub_truth_twist_->publish(ros_truth_twist_);
    }
    #endif
  }

 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double rate_;
  bool tf_, pose_, twist_, static_;
  std::string parent_, child_;
  geometry_msgs::TransformStamped msg_;
  geometry_msgs::PoseStamped ros_truth_pose_;
  geometry_msgs::TwistStamped ros_truth_twist_;
  rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr pub_truth_pose_;
  rclcpp::Publisher<geometry_msgs::TwistStamped>::SharedPtr pub_truth_twist_;
  ff_util::FreeFlyerTimer timer_;
  rclcpp::Time last_time_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginTruth)

}   // namespace gazebo
