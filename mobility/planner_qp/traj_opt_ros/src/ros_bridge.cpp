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

#include <traj_opt_ros/ros_bridge.h>
#include <ff_util/ff_component.h>
#include <string>

TrajRosBridge::TrajRosBridge(){
  nh_traj_ = std::make_shared<rclcpp::Node>("~","~");
  nh_info_ = std::make_shared<rclcpp::Node>("~","~");
  // rclcpp::Node n("~");
  // nh_ = std::shared_ptr(n);
}
TrajRosBridge &TrajRosBridge::instance() {
  static TrajRosBridge inst;
  return inst;
}
rclcpp::Publisher<traj_opt_msgs::Trajectory>::SharedPtr TrajRosBridge::getPub(std::string topic) {
  if (instance().pubs_traj_.find(topic) == instance().pubs_traj_.end()) {
    instance().pubs_traj_[topic] =
        instance().nh_traj_->create_publisher<traj_opt_msgs::Trajectory>(topic, 1);
  }
  return instance().pubs_traj_[topic];
}
rclcpp::Publisher<traj_opt_msgs::SolverInfo>::SharedPtr TrajRosBridge::getInfoPub(std::string topic) {
  if (instance().pubs_info_.find(topic) == instance().pubs_info_.end()) {
    instance().pubs_info_[topic] =
        instance().nh_info_->create_publisher<traj_opt_msgs::SolverInfo>(topic, 1);
  }
  return instance().pubs_info_[topic];
}

bool TrajRosBridge::are_subscribers(std::string topic) {
  rclcpp::PublisherBase::SharedPtr pub = getPub(topic);
  return pub->get_subscription_count() > 0;
}

void TrajRosBridge::publish_msg(const traj_opt_msgs::Trajectory &msg,
                                std::string frame_id, std::string topic) {
  traj_opt_msgs::Trajectory msgc = msg;
  msgc.header.frame_id = frame_id;
  getPub(topic)->publish(msgc);
}
void TrajRosBridge::publish_msg(const traj_opt::TrajData &data,
                                std::string frame_id, std::string topic) {
  publish_msg(convert(data), frame_id, topic);
}

// these convert functions can be written more cleanly with templates
traj_opt_msgs::Trajectory TrajRosBridge::convert(
    const traj_opt::TrajData &data) {
  traj_opt_msgs::Trajectory traj;
  traj.header.stamp = GetTimeNow(); //nh_traj_->get_clock()->now(); //
  traj.header.frame_id = "map";

  traj.dimension_names = data.dimension_names;
  traj.dimensions = data.dimensions;
  // copy all fields
  for (auto spline : data.data) {
    traj_opt_msgs::Spline s;
    for (auto poly : spline.segs) {
      traj_opt_msgs::Polynomial p;
      p.degree = poly.degree;
      p.dt = poly.dt;
      p.basis = poly.basis;
      p.coeffs = poly.coeffs;
      s.segs.push_back(p);
    }
    s.segments = spline.segments;
    s.t_total = spline.t_total;
    traj.data.push_back(s);
  }
  return traj;
}
traj_opt::TrajData TrajRosBridge::convert(
    const traj_opt_msgs::Trajectory &msg) {
  traj_opt::TrajData data;
  // copy all fields
  data.dimension_names = msg.dimension_names;
  data.dimensions = msg.dimensions;
  for (auto spline : msg.data) {
    traj_opt::SplineData s;
    for (auto poly : spline.segs) {
      traj_opt::PolynomialData p;
      p.degree = poly.degree;
      p.dt = poly.dt;
      p.basis = (traj_opt::PolyType)poly.basis;
      p.coeffs = poly.coeffs;
      s.segs.push_back(p);
    }
    s.segments = spline.segments;
    s.t_total = spline.t_total;
    data.data.push_back(s);
  }
  return data;
}
traj_opt_msgs::SolverInfo TrajRosBridge::convert(
    const traj_opt::SolverInfo &data) {
  traj_opt_msgs::SolverInfo info;
  for (auto &g : data.gap_history) info.gap_history.push_back(g);
  for (auto &g : data.cost_history) info.cost_history.push_back(g);
  for (auto &g : data.slack) info.slack.push_back(g);

  info.iterations = data.iterations;
  info.cost = data.cost;
  info.gap = data.gap;

  return info;
}

void TrajRosBridge::publish_msg(const traj_opt_msgs::SolverInfo &msg,
                                std::string topic) {
  getInfoPub(topic)->publish(msg);
}
void TrajRosBridge::publish_msg(const traj_opt::SolverInfo &data,
                                std::string topic) {
  publish_msg(convert(data), topic);
}
