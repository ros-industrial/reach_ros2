#include <reach_ros/target_pose_generator/transformed_yaml_target_pose_generator.h>
#include <reach_ros/utils.h>

#include <reach/plugin_utils.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace reach_ros
{
TransformedYAMLTargetPoseGenerator::TransformedYAMLTargetPoseGenerator(std::string filename, std::string points_frame,
                                                                       std::string target_frame)
  : reach::YAMLTargetPoseGenerator(filename)
  , points_frame_(std::move(points_frame))
  , target_frame_(std::move(target_frame))
{
}

reach::VectorIsometry3d TransformedYAMLTargetPoseGenerator::generate() const
{
  reach::VectorIsometry3d target_poses = YAMLTargetPoseGenerator::generate();

  // Look up the transform between the source and target frame
  tf2_ros::Buffer buffer(utils::getNodeInstance()->get_clock());
  tf2_ros::TransformListener listener(buffer);
  Eigen::Isometry3d transform = tf2::transformToEigen(
      buffer.lookupTransform(target_frame_, points_frame_, tf2::TimePointZero, tf2::durationFromSec(3.0)));

  // Apply the transform to the poses
  for (Eigen::Isometry3d& pose : target_poses)
  {
    pose = transform * pose;
  }

  return target_poses;
}

reach::TargetPoseGenerator::ConstPtr TransformedYAMLTargetPoseGeneratorFactory::create(const YAML::Node& config) const
{
  std::string filename = reach::get<std::string>(config, "poses");
  std::string source_frame = reach::get<std::string>(config, "points_frame");
  std::string target_frame = reach::get<std::string>(config, "target_frame");
  return std::make_shared<TransformedYAMLTargetPoseGenerator>(filename, source_frame, target_frame);
}

}  // namespace reach_ros
