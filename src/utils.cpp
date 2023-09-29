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
#include <reach_ros/utils.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <reach/types.h>
#include <reach/utils.h>

const static double ARROW_SCALE_RATIO = 6.0;
const static double NEIGHBOR_MARKER_SCALE_RATIO = ARROW_SCALE_RATIO / 2.0;

/**
 * @brief Singleton class for interacting with a ROS network
 */
class ROSInterface
{
public:
  ROSInterface();
  virtual ~ROSInterface();

  rclcpp::Node::SharedPtr node;

private:
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  std::shared_ptr<std::thread> executor_thread;
};

ROSInterface::ROSInterface()
{
  // Ensure ROS is initialized before creating the node/executor
  // Note: we cannot initialize ROS here ourselves with rclcpp::init(0, nullptr) because ROS2 parameters which we want
  // to access are passed in via argv. Since we don't have access to argv outside an executable and can't require
  // users/plugin loaders to pass it around, this function should throw an exception if ROS is not initialized
  if (!rclcpp::ok())
    throw std::runtime_error("ROS must be initialized before accessing the node");

  // Create a node that accepts arbitrary parameters later
  node = std::make_shared<rclcpp::Node>(
      "reach_study_node",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  // Create an executor and add the node to it
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);

  // Create a thread
  executor_thread =
      std::make_shared<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &*executor));
}

ROSInterface::~ROSInterface()
{
  rclcpp::shutdown();
  executor_thread->join();
}

namespace reach_ros
{
namespace utils
{
rclcpp::Node::SharedPtr getNodeInstance()
{
  static std::unique_ptr<ROSInterface> ros;

  // Create an instance of the ROS interface if it doesn't exist yet
  if (!ros)
    ros = std::make_unique<ROSInterface>();

  return ros->node;
}

moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                        const std::string& parent_link, const std::string& object_name)
{
  // Create a CollisionObject message for the reach object
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = parent_link;
  obj.id = object_name;
  shapes::ShapeMsg shape_msg;
  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_filename);
  shapes::constructMsgFromShape(mesh, shape_msg);
  obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
  obj.operation = obj.ADD;

  // Assign a default pose to the mesh
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose.position.y = pose.position.z = 0.0;
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  obj.mesh_poses.push_back(pose);

  return obj;
}

visualization_msgs::msg::Marker makeVisual(const reach::ReachRecord& r, const std::string& frame, const double scale,
                                           const std::string& ns, const Eigen::Vector3f& color)
{
  static int idx = 0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = getNodeInstance()->get_clock()->now();
  marker.ns = ns;
  marker.id = idx++;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Transform arrow such that arrow x-axis points along goal pose z-axis (Rviz convention)
  // convert msg parameter goal to Eigen matrix
  Eigen::AngleAxisd rot_flip_normal(M_PI, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_x_to_z(-M_PI / 2, Eigen::Vector3d::UnitY());

  // Transform
  Eigen::Isometry3d goal_eigen = r.goal * rot_flip_normal * rot_x_to_z;

  // Convert back to geometry_msgs pose
  geometry_msgs::msg::Pose msg = tf2::toMsg(goal_eigen);
  marker.pose = msg;

  marker.scale.x = scale;
  marker.scale.y = scale / ARROW_SCALE_RATIO;
  marker.scale.z = scale / ARROW_SCALE_RATIO;

  marker.color.a = 1.0;  // Don't forget to set the alpha!
  if (r.reached)
  {
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
  }
  else
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }

  return marker;
}

visualization_msgs::msg::InteractiveMarker makeInteractiveMarker(const std::string& id, const reach::ReachRecord& r,
                                                                 const std::string& frame, const double scale,
                                                                 const Eigen::Vector3f& rgb_color)
{
  visualization_msgs::msg::InteractiveMarker m;
  m.header.frame_id = frame;
  m.name = id;

  // Create a menu entry to display the score
  {
    visualization_msgs::msg::MenuEntry entry;
    entry.command_type = visualization_msgs::msg::MenuEntry::FEEDBACK;
    entry.id = 1;
    entry.parent_id = 0;

    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(4);
    ss << "Score: " << r.score;
    entry.title = ss.str();

    m.menu_entries.push_back(entry);
  }

  // Control
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  // Visuals
  auto visual = makeVisual(r, frame, scale, "reach", rgb_color);
  control.markers.push_back(visual);
  m.controls.push_back(control);

  return m;
}

visualization_msgs::msg::Marker makeMarker(const std::vector<geometry_msgs::msg::Point>& pts, const std::string& frame,
                                           const double scale, const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = getNodeInstance()->get_clock()->now();
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = marker.scale.y = marker.scale.z = scale / NEIGHBOR_MARKER_SCALE_RATIO;

  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0;
  marker.color.g = 1.0;
  marker.color.b = 0;

  for (std::size_t i = 0; i < pts.size(); ++i)
  {
    marker.points.push_back(pts[i]);
  }

  return marker;
}

std::vector<double> transcribeInputMap(const std::map<std::string, double>& input,
                                       const std::vector<std::string>& joint_names)
{
  return reach::extractSubset(input, joint_names);
}

}  // namespace utils
}  // namespace reach_ros
