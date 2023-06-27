/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef REACH_ROS_KINEMATICS_UTILS_H
#define REACH_ROS_KINEMATICS_UTILS_H

#include <Eigen/Dense>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <map>

namespace reach
{
class ReachRecord;
}

namespace reach_ros
{
namespace utils
{
moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                        const std::string& parent_link, const std::string& object_name);

visualization_msgs::msg::Marker makeVisual(const reach::ReachRecord& r, const std::string& frame, const double scale,
                                           const std::string& ns = "reach",
                                           const Eigen::Vector3f& color = { 0.5, 0.5, 0.5 });

visualization_msgs::msg::InteractiveMarker makeInteractiveMarker(const std::string& id, const reach::ReachRecord& r,
                                                                 const std::string& frame, const double scale,
                                                                 const Eigen::Vector3f& rgb_color = { 0.5, 0.5, 0.5 });

visualization_msgs::msg::Marker makeMarker(const std::vector<geometry_msgs::msg::Point>& pts, const std::string& frame,
                                           const double scale, const std::string& ns = "");

std::vector<double> transcribeInputMap(const std::map<std::string, double>& input,
                                       const std::vector<std::string>& joint_names);

/**
 * @brief Returns a singleton ROS2 node for accessing parameters and publishing data
 * @details ROS must be initialized (rclcpp::init, rclpy.init) before calling this method
 */
rclcpp::Node::SharedPtr getNodeInstance();

}  // namespace utils
}  // namespace reach_ros

#endif  // REACH_ROS_KINEMATICS_UTILS_H
