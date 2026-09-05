#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp> //include the pose message position and orientation
#include <rclcpp/rclcpp.hpp> //include the rclcpp library for the node and logger
#include <moveit/move_group_interface/move_group_interface.h> //include the move group interface for the move group

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;


bool planAndExecute(
    MoveGroupInterface &move_group,
    const std::string &description)
{
  MoveGroupInterface::Plan plan; 
 /* This object stores the generated joint trajectory:

Joint positions
Joint velocities
Joint accelerations
Timing */

  RCLCPP_INFO( //log the planning message
      rclcpp::get_logger("pose_target"),
      "Planning: %s",
      description.c_str());

  const bool planned = //plan the trajectory
      static_cast<bool>(move_group.plan(plan));

  if (!planned) {
    RCLCPP_ERROR(
        rclcpp::get_logger("pose_target"),
        "Planning failed: %s",
        description.c_str());
    return false;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("pose_target"),
      "Executing: %s",
      description.c_str());

  const bool executed = //execute the trajectory
      static_cast<bool>(move_group.execute(plan));

  if (!executed) {
    RCLCPP_ERROR(
        rclcpp::get_logger("pose_target"),
        "Execution failed: %s",
        description.c_str());
    return false;
  }

  return true;
}


bool moveToNamedTarget(
    MoveGroupInterface &move_group,
    const std::string &target_name)
{
  move_group.setStartStateToCurrentState();

  if (!move_group.setNamedTarget(target_name)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("pose_target"),
        "Unknown named target: %s",
        target_name.c_str());
    return false;
  }

  return planAndExecute(
      move_group,
      "named target " + target_name);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "moveit_pose_target",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  MoveGroupInterface move_group(node, "arm");

  //The robot moves at 10% of the maximum configured velocity and acceleratio
  move_group.setMaxVelocityScalingFactor(0.10); //set the maximum velocity scaling factor
  move_group.setMaxAccelerationScalingFactor(0.10); //set the maximum acceleration scaling factor
  //MoveIt receives up to 10 seconds and 10 attempts to find a valid path.
  move_group.setPlanningTime(10.0); //set the planning time
  move_group.setNumPlanningAttempts(10); //set the number of planning attempts

  move_group.setPoseReferenceFrame("world");
  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(0.02);

  const std::string tcp_link = "gripper_tcp";

  RCLCPP_INFO(
      node->get_logger(),
      "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      node->get_logger(),
      "TCP link: %s",
      tcp_link.c_str());

  bool success = moveToNamedTarget(
      move_group,
      "grasp_approach");

  if (success) {
    std::this_thread::sleep_for(
        std::chrono::seconds(2));

        //MoveIt uses forward kinematics to calculate the current TCP pose from the current six joint angles.
        //The current pose is the pose of the end effector in the world frame.
    geometry_msgs::msg::Pose target_pose =
        move_group.getCurrentPose(tcp_link).pose;

    RCLCPP_INFO(
        node->get_logger(),
        "Current TCP pose: x=%.3f, y=%.3f, z=%.3f",
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z);

    target_pose.position.z -= 0.03;

    RCLCPP_INFO(
        node->get_logger(),
        "Target TCP pose: x=%.3f, y=%.3f, z=%.3f",
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z);

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(
        target_pose,
        tcp_link);

    success = planAndExecute(
        move_group,
        "move gripper_tcp 3 cm downward");

    move_group.clearPoseTargets();
  }

  if (success) {
    std::this_thread::sleep_for(
        std::chrono::seconds(2));

    success = moveToNamedTarget(
        move_group,
        "grasp_approach");
  }

  if (success) {
    std::this_thread::sleep_for(
        std::chrono::seconds(2));

    success = moveToNamedTarget(
        move_group,
        "home");
  }

  if (success) {
    RCLCPP_INFO(
        node->get_logger(),
        "Pose-target sequence completed successfully.");
  } else {
    RCLCPP_ERROR(
        node->get_logger(),
        "Pose-target sequence stopped.");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return success ? 0 : 1;
}