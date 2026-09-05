#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp> // rclcpp/rclcpp.hpp
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

using MoveGroup =
    moveit::planning_interface::MoveGroupInterface;

const auto LOGGER = rclcpp::get_logger("cartesian_path");
const std::string TCP = "gripper_tcp";


bool namedMove(MoveGroup &group, const std::string &name)
{
  group.clearPoseTargets();
  group.setStartStateToCurrentState();

  if (!group.setNamedTarget(name)) {
    RCLCPP_ERROR(LOGGER, "Unknown target: %s", name.c_str());
    return false;
  }

  MoveGroup::Plan plan;

  RCLCPP_INFO(LOGGER, "Planning named target: %s", name.c_str());

  if (!static_cast<bool>(group.plan(plan))) {
    RCLCPP_ERROR(LOGGER, "Named-target planning failed.");
    return false;
  }

  return static_cast<bool>(group.execute(plan));
}


// Read a fresh joint state and calculate the TCP pose using FK.
bool readPose(MoveGroup &group, geometry_msgs::msg::Pose &pose)
{
  auto state = group.getCurrentState(5.0);

  if (!state) {
    RCLCPP_ERROR(LOGGER, "No current robot state available.");
    return false;
  }

  state->update();
  const auto &transform = state->getGlobalLinkTransform(TCP);
  const Eigen::Quaterniond quaternion(transform.rotation());

  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  return true;
}


bool cartesianMove(
    MoveGroup &group,
    const geometry_msgs::msg::Pose &target)
{
  auto start = group.getCurrentState(5.0);

  if (!start) {
    RCLCPP_ERROR(LOGGER, "Cannot obtain Cartesian start state.");
    return false;
  }

  group.clearPoseTargets();
  group.setStartState(*start);

  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory trajectory;

  // 2 mm interpolation steps.
  // 5.0 enables relative joint-jump checking.
  // true enables collision checking.
  const double fraction = group.computeCartesianPath(
      waypoints, 0.002, 5.0, trajectory, true);

  RCLCPP_INFO(
      LOGGER, "Cartesian path completion: %.2f%%",
      fraction * 100.0);

  if (!std::isfinite(fraction) ||
      fraction < 0.999999 ||
      trajectory.joint_trajectory.points.size() < 2)
  {
    RCLCPP_ERROR(
        LOGGER, "Incomplete Cartesian path. Execution skipped.");
    return false;
  }

  robot_trajectory::RobotTrajectory timed(
      group.getRobotModel(), "arm");

  timed.setRobotTrajectoryMsg(*start, trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization timing;

  if (!timing.computeTimeStamps(timed, 0.10, 0.10)) {
    RCLCPP_ERROR(LOGGER, "Trajectory timing failed.");
    return false;
  }

  timed.getRobotTrajectoryMsg(trajectory);

  MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;

  RCLCPP_INFO(LOGGER, "Executing complete Cartesian path.");

  if (!static_cast<bool>(group.execute(plan))) {
    RCLCPP_ERROR(LOGGER, "Cartesian execution failed.");
    return false;
  }

  return true;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "moveit_cartesian_path",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  bool success = false;

  try {
    MoveGroup group(node, "arm");

    group.setEndEffectorLink(TCP);
    group.setPoseReferenceFrame(group.getPlanningFrame());
    group.setMaxVelocityScalingFactor(0.10);
    group.setMaxAccelerationScalingFactor(0.10);
    group.setPlanningTime(10.0);

    success = namedMove(group, "grasp_approach");

    geometry_msgs::msg::Pose original_pose;

    if (success) {
      success = readPose(group, original_pose);
    }

    if (success) {
      auto lower_pose = original_pose;
      lower_pose.position.z -= 0.03;

      RCLCPP_INFO(
          LOGGER, "Straight descent: z=%.4f -> %.4f",
          original_pose.position.z, lower_pose.position.z);

      success = cartesianMove(group, lower_pose);
    }

    if (success) {
      RCLCPP_INFO(LOGGER, "Straight return to original TCP pose.");
      success = cartesianMove(group, original_pose);
    }

    if (success) {
      success = namedMove(group, "home");
    }
  }
  catch (const std::exception &error) {
    RCLCPP_ERROR(LOGGER, "Exception: %s", error.what());
    success = false;
  }

  if (success) {
    RCLCPP_INFO(LOGGER, "Cartesian sequence completed successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Sequence stopped; later moves skipped.");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return success ? 0 : 1;
}