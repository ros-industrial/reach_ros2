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
#include <reach_ros/evaluation/distance_penalty_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <reach/plugin_utils.h>
#include <yaml-cpp/yaml.h>

namespace reach_ros
{
namespace evaluation
{
std::string DistancePenaltyMoveIt::COLLISION_OBJECT_NAME = "reach_object";

DistancePenaltyMoveIt::DistancePenaltyMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                             const double dist_threshold, int exponent)
  : model_(model)
  , jmg_(model_->getJointModelGroup(planning_group))
  , dist_threshold_(dist_threshold)
  , exponent_(exponent)
{
  if (!jmg_)
    throw std::runtime_error("Failed to get joint model group");

  scene_.reset(new planning_scene::PlanningScene(model_));
}

void DistancePenaltyMoveIt::addCollisionMesh(const std::string& collision_mesh_filename,
                                             const std::string& collision_mesh_frame)
{
  // Add the collision object to the planning scene
  moveit_msgs::msg::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename, collision_mesh_frame, COLLISION_OBJECT_NAME);
  if (!scene_->processCollisionObjectMsg(obj))
    throw std::runtime_error("Failed to add collision mesh to planning scene");
}

void DistancePenaltyMoveIt::setTouchLinks(const std::vector<std::string>& touch_links)
{
  scene_->getAllowedCollisionMatrixNonConst().setEntry(COLLISION_OBJECT_NAME, touch_links, true);
}

double DistancePenaltyMoveIt::calculateScore(const std::map<std::string, double>& pose) const
{
  // Pull the joints from the planning group out of the input pose map
  std::vector<double> pose_subset = utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames());
  moveit::core::RobotState state(model_);
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  const double dist = scene_->distanceToCollision(state, scene_->getAllowedCollisionMatrix());
  const double clipped_distance = std::min(std::abs(dist / dist_threshold_), 1.0);
  return std::pow(clipped_distance, exponent_);
}

reach::Evaluator::ConstPtr DistancePenaltyMoveItFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");
  auto exponent = reach::get<int>(config, "exponent");

  moveit::core::RobotModelConstPtr model =
      moveit::planning_interface::getSharedRobotModel(reach_ros::utils::getNodeInstance(), "robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto evaluator = std::make_shared<DistancePenaltyMoveIt>(model, planning_group, dist_threshold, exponent);

  // Optionally add a collision mesh
  const std::string collision_mesh_filename_key = "collision_mesh_filename";
  const std::string collision_mesh_frame_key = "collision_mesh_frame";
  if (config[collision_mesh_filename_key])
  {
    auto collision_mesh_filename = reach::get<std::string>(config, collision_mesh_filename_key);
    const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(planning_group);
    if (!jmg)
      throw std::runtime_error("Joint model group '" + planning_group + "' does not exist");

    kinematics::KinematicsBaseConstPtr solver = jmg->getSolverInstance();
    if (!solver)
      throw std::runtime_error("No IK solver defined for joint model group '" + planning_group + "'");

    std::string collision_mesh_frame = config[collision_mesh_frame_key] ?
                                           reach::get<std::string>(config, collision_mesh_frame_key) :
                                           solver->getBaseFrame();

    evaluator->addCollisionMesh(collision_mesh_filename, collision_mesh_frame);
  }

  const std::string touch_links_key = "touch_links";
  if (config[touch_links_key])
  {
    auto touch_links = reach::get<std::vector<std::string>>(config, touch_links_key);
    evaluator->setTouchLinks(touch_links);
  }

  return evaluator;
}

}  // namespace evaluation
}  // namespace reach_ros
