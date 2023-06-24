#include <reach_ros/display/ros_display.h>
#include <reach_ros/evaluation/distance_penalty_moveit.h>
#include <reach_ros/evaluation/joint_penalty_moveit.h>
#include <reach_ros/evaluation/manipulability_moveit.h>
#include <reach_ros/ik/moveit_ik_solver.h>
#include <reach_ros/target_pose_generator/transformed_point_cloud_target_pose_generator.h>

#include <reach/plugin_utils.h>
EXPORT_DISPLAY_PLUGIN(reach_ros::display::ROSDisplayFactory, ROSDisplay)
EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::DistancePenaltyMoveItFactory, DistancePenaltyMoveIt)
EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::JointPenaltyMoveItFactory, JointPenaltyMoveIt)
EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::ManipulabilityMoveItFactory, ManipulabilityMoveIt)
EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::ManipulabilityScaledFactory, ManipulabilityScaledMoveIt)
EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::ManipulabilityRatioFactory, ManipulabilityRatioMoveIt)
EXPORT_IK_SOLVER_PLUGIN(reach_ros::ik::MoveItIKSolverFactory, MoveItIKSolver)
EXPORT_IK_SOLVER_PLUGIN(reach_ros::ik::DiscretizedMoveItIKSolverFactory, DiscretizedMoveItIKSolverFactory)
EXPORT_TARGET_POSE_GENERATOR_PLUGIN(reach_ros::TransformedPointCloudTargetPoseGeneratorFactory,
                                    TransformedPointCloudTargetPoseGenerator)
