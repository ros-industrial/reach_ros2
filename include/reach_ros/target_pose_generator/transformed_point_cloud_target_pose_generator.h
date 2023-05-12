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
#ifndef REACH_ROS_TARGET_POSE_GENERATOR_POINT_CLOUD_TARGET_POSE_GENERATOR_H
#define REACH_ROS_TARGET_POSE_GENERATOR_POINT_CLOUD_TARGET_POSE_GENERATOR_H

#include <reach/plugins/point_cloud_target_pose_generator.h>

namespace reach_ros
{
class TransformedPointCloudTargetPoseGenerator : public reach::PointCloudTargetPoseGenerator
{
public:
  TransformedPointCloudTargetPoseGenerator(std::string filename, std::string source_frame, std::string target_frame);

  reach::VectorIsometry3d generate() const override;

private:
  std::string filename_;
  std::string points_frame_;
  std::string target_frame_;
};

struct TransformedPointCloudTargetPoseGeneratorFactory : public reach::PointCloudTargetPoseGeneratorFactory
{
  using PointCloudTargetPoseGeneratorFactory::PointCloudTargetPoseGeneratorFactory;
  reach::TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace reach_ros

#endif  // REACH_ROS_TARGET_POSE_GENERATOR_POINT_CLOUD_TARGET_POSE_GENERATOR_H
