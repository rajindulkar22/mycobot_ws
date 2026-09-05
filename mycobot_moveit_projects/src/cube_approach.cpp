#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace
{

using MoveGroup =
    moveit::planning_interface::MoveGroupInterface;
using GripperCommand = control_msgs::action::GripperCommand;

const auto LOGGER = rclcpp::get_logger("cube_approach");
const std::string kTcpLink = "gripper_tcp";
const std::string kGripperAction =
    "/gripper_action_controller/gripper_cmd";

constexpr double kGripperOpen = 0.0;
constexpr double kGripperClosed = -0.55;
constexpr double kGripperMaxEffort = 8.0;
constexpr double kGripperHoldSec = 1.5;
constexpr double kGripperCloseTimeoutSec = 3.0;
constexpr double kGripperOpenTimeoutSec = 10.0;
constexpr double kPlaceReleaseHoldSec = 0.5;

geometry_msgs::msg::Pose poseFromState(
    moveit::core::RobotState &state)
{
  state.update();

  const auto &transform =
      state.getGlobalLinkTransform(kTcpLink);

  Eigen::Quaterniond quaternion(transform.rotation());
  quaternion.normalize();

  geometry_msgs::msg::Pose pose;

  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  return pose;
}


geometry_msgs::msg::Pose tcpPoseFromNamedTarget(
    MoveGroup &group,
    const std::string &named_target)
{
  if (!group.setNamedTarget(named_target)) {
    throw std::runtime_error(
        "Unknown named target: " + named_target);
  }

  std::vector<double> joint_values;
  group.getJointValueTarget(joint_values);

  if (joint_values.empty()) {
    throw std::runtime_error(
        "Could not read joint values for " + named_target);
  }

  group.clearPoseTargets();

  moveit::core::RobotState fk_state(group.getRobotModel());
  fk_state.setToDefaultValues();
  fk_state.setJointGroupPositions(group.getName(), joint_values);

  return poseFromState(fk_state);
}


void reportPosition(
    const std::string &label,
    const geometry_msgs::msg::Pose &actual,
    const geometry_msgs::msg::Pose &target)
{
  const double dx = actual.position.x - target.position.x;
  const double dy = actual.position.y - target.position.y;
  const double dz = actual.position.z - target.position.z;

  const double error =
      std::sqrt(dx * dx + dy * dy + dz * dz);

  RCLCPP_INFO(
      LOGGER,
      "%s: measured TCP x=%.6f, y=%.6f, z=%.6f",
      label.c_str(),
      actual.position.x,
      actual.position.y,
      actual.position.z);

  RCLCPP_INFO(
      LOGGER,
      "Position error: dx=%.6f, dy=%.6f, dz=%.6f m; total=%.2f mm",
      dx, dy, dz, error * 1000.0);
}


bool sendGripperCommand(
    const rclcpp::Node::SharedPtr &node,
    double position,
    double timeout_sec,
    bool accept_stall)
{
  auto client = rclcpp_action::create_client<GripperCommand>(
      node, kGripperAction);

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    throw std::runtime_error("Gripper action server unavailable.");
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = kGripperMaxEffort;

  RCLCPP_INFO(
      LOGGER,
      "Sending gripper command: position=%.3f rad",
      position);

  auto send_future = client->async_send_goal(goal);

  // Node is already on a background executor; wait for callbacks there.
  if (send_future.wait_for(std::chrono::seconds(10)) !=
      std::future_status::ready)
  {
    throw std::runtime_error("Gripper goal send timed out.");
  }

  const auto goal_handle = send_future.get();

  if (!goal_handle) {
    throw std::runtime_error("Gripper goal was rejected.");
  }

  auto result_future = client->async_get_result(goal_handle);

  if (result_future.wait_for(
          std::chrono::duration<double>(timeout_sec)) !=
      std::future_status::ready)
  {
    if (accept_stall) {
      RCLCPP_WARN(
          LOGGER,
          "Gripper close timed out after %.1f s; proceeding.",
          timeout_sec);
      return true;
    }

    throw std::runtime_error("Gripper command timed out.");
  }

  const auto wrapped = result_future.get();

  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    if (accept_stall) {
      RCLCPP_WARN(
          LOGGER,
          "Gripper close did not report success; proceeding.");
      return true;
    }

    throw std::runtime_error("Gripper command failed.");
  }

  const auto &result = wrapped.result;

  if (accept_stall && result->stalled) {
    RCLCPP_INFO(LOGGER, "Gripper stalled on object (grasp OK).");
    return true;
  }

  if (!result->reached_goal) {
    if (accept_stall) {
      RCLCPP_WARN(
          LOGGER,
          "Gripper did not reach goal; proceeding for grasp.");
      return true;
    }

    throw std::runtime_error("Gripper did not reach goal.");
  }

  return true;
}


void waitForArmSettled(MoveGroup &group, double timeout_sec)
{
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);

  constexpr double velocity_tolerance = 0.05;

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    auto state = group.getCurrentState(1.0);

    if (!state)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    const auto *joint_group =
        group.getRobotModel()->getJointModelGroup(group.getName());

    if (joint_group == nullptr)
    {
      return;
    }

    bool settled = true;

    for (const std::string &joint_name :
         joint_group->getActiveJointModelNames())
    {
      if (std::abs(state->getVariableVelocity(joint_name)) >
          velocity_tolerance)
      {
        settled = false;
        break;
      }
    }

    if (settled)
    {
      RCLCPP_INFO(LOGGER, "Arm settled (ready to plan).");
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_WARN(
      LOGGER,
      "Arm did not fully settle within %.1f s; continuing anyway.",
      timeout_sec);
}


using ReplanFn = std::function<bool(MoveGroup &, MoveGroup::Plan &)>;


void executePlan(
    MoveGroup &group,
    MoveGroup::Plan &plan,
    const std::string &label,
    const ReplanFn &replan)
{
  group.setStartStateToCurrentState();

  if (static_cast<bool>(group.execute(plan)))
  {
    return;
  }

  RCLCPP_WARN(
      LOGGER,
      "%s execution failed; settling, re-planning, and retrying once.",
      label.c_str());

  waitForArmSettled(group, 10.0);
  group.setStartStateToCurrentState();

  if (!replan(group, plan))
  {
    throw std::runtime_error("Re-planning failed for " + label + ".");
  }

  group.setStartStateToCurrentState();

  if (static_cast<bool>(group.execute(plan)))
  {
    return;
  }

  throw std::runtime_error(
      "Execution failed for " + label +
      ". Restart move_group after restarting Gazebo. "
      "Do not use gazebo_pose_commander while cube_approach runs.");
}


bool buildCartesianPlan(
    MoveGroup &group,
    const geometry_msgs::msg::Pose &target,
    const std::string &label,
    MoveGroup::Plan &plan)
{
  auto start = group.getCurrentState(5.0);

  if (!start) {
    throw std::runtime_error(
        "No current robot state for " + label + ".");
  }

  group.clearPoseTargets();
  group.setStartState(*start);

  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory trajectory;

  const double fraction = group.computeCartesianPath(
      waypoints,
      0.002,
      5.0,
      trajectory,
      true);

  RCLCPP_INFO(
      LOGGER,
      "%s Cartesian path completion: %.2f%%",
      label.c_str(),
      fraction * 100.0);

  if (!std::isfinite(fraction) ||
      fraction < 0.999999 ||
      trajectory.joint_trajectory.points.size() < 2)
  {
    return false;
  }

  robot_trajectory::RobotTrajectory timed(
      group.getRobotModel(), "arm");

  timed.setRobotTrajectoryMsg(*start, trajectory);

  trajectory_processing::TimeOptimalTrajectoryGeneration timing;

  if (!timing.computeTimeStamps(timed, 0.10, 0.10)) {
    throw std::runtime_error(
        "Trajectory timing failed for " + label + ".");
  }

  timed.getRobotTrajectoryMsg(trajectory);
  plan.trajectory_ = trajectory;
  return true;
}


void executeCartesianToPose(
    MoveGroup &group,
    const geometry_msgs::msg::Pose &target,
    const std::string &label)
{
  RCLCPP_INFO(
      LOGGER,
      "%s target: x=%.6f, y=%.6f, z=%.6f",
      label.c_str(),
      target.position.x,
      target.position.y,
      target.position.z);

  MoveGroup::Plan plan;

  if (!buildCartesianPlan(group, target, label, plan)) {
    throw std::runtime_error(
        "Incomplete Cartesian path for " + label + ".");
  }

  const ReplanFn replan = [&](MoveGroup &move_group, MoveGroup::Plan &new_plan) {
    return buildCartesianPlan(move_group, target, label, new_plan);
  };

  RCLCPP_INFO(LOGGER, "Executing %s.", label.c_str());
  executePlan(group, plan, label, replan);
}


void planAndExecuteToNamedTarget(
    MoveGroup &group,
    const std::string &target_name,
    const std::string &label)
{
  group.clearPoseTargets();
  group.setStartStateToCurrentState();

  if (!group.setNamedTarget(target_name)) {
    throw std::runtime_error(
        "Unknown named target for " + label + ": " + target_name);
  }

  MoveGroup::Plan plan;

  RCLCPP_INFO(LOGGER, "Planning %s (named target '%s').", label.c_str(), target_name.c_str());

  if (!static_cast<bool>(group.plan(plan))) {
    throw std::runtime_error("Planning failed for " + label + ".");
  }

  const ReplanFn replan = [target_name](MoveGroup &move_group, MoveGroup::Plan &new_plan) {
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    if (!move_group.setNamedTarget(target_name)) {
      return false;
    }
    return static_cast<bool>(move_group.plan(new_plan));
  };

  RCLCPP_INFO(LOGGER, "Executing %s.", label.c_str());
  executePlan(group, plan, label, replan);
}


void planAndExecuteToPose(
    MoveGroup &group,
    const geometry_msgs::msg::Pose &target,
    const std::string &label)
{
  group.clearPoseTargets();
  group.setStartStateToCurrentState();

  RCLCPP_INFO(
      LOGGER,
      "%s target: x=%.6f, y=%.6f, z=%.6f",
      label.c_str(),
      target.position.x,
      target.position.y,
      target.position.z);

  if (!group.setPoseTarget(target, kTcpLink)) {
    throw std::runtime_error("Could not set target for " + label + ".");
  }

  MoveGroup::Plan plan;

  if (!static_cast<bool>(group.plan(plan))) {
    throw std::runtime_error("Planning failed for " + label + ".");
  }

  const ReplanFn replan = [target](MoveGroup &move_group, MoveGroup::Plan &new_plan) {
    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();
    if (!move_group.setPoseTarget(target, kTcpLink)) {
      return false;
    }
    return static_cast<bool>(move_group.plan(new_plan));
  };

  RCLCPP_INFO(LOGGER, "Executing %s.", label.c_str());
  executePlan(group, plan, label, replan);
}


void cartesianOrPlanToPose(
    MoveGroup &group,
    const geometry_msgs::msg::Pose &target,
    const std::string &label)
{
  try {
    executeCartesianToPose(group, target, label);
  }
  catch (const std::exception &cartesian_error) {
    RCLCPP_WARN(
        LOGGER,
        "%s Cartesian path failed (%s); trying OMPL.",
        label.c_str(),
        cartesian_error.what());
    planAndExecuteToPose(group, target, label);
  }
}

}  // namespace


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "cube_approach",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  auto display =
      node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
          "/display_planned_path",
          rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  bool success = false;

  try {
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(10);

    while (
        rclcpp::ok() &&
        (!node->get_clock()->ros_time_is_active() ||
         node->now().nanoseconds() == 0))
    {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error(
            "No simulation clock received. Start Gazebo first.");
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(50));
    }

    if (!rclcpp::ok()) {
      throw std::runtime_error("Interrupted before initialization.");
    }

    RCLCPP_INFO(
        LOGGER,
        "Simulation clock active: %.3f s",
        node->now().seconds());

    const double place_x = node->get_parameter("place_x").as_double();
    const double place_y = node->get_parameter("place_y").as_double();
    const double place_z = node->get_parameter("place_z").as_double();
    const double place_descend_z =
        node->get_parameter("place_descend_z").as_double();
    const bool return_home =
        node->get_parameter("return_home").as_bool();

    MoveGroup group(node, "arm");

    if (!group.startStateMonitor(5.0)) {
      throw std::runtime_error(
          "Joint-state monitor did not initialize.");
    }

    group.setEndEffectorLink(kTcpLink);
    group.setPoseReferenceFrame("world");

    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(10);
    group.setMaxVelocityScalingFactor(0.10);
    group.setMaxAccelerationScalingFactor(0.10);

    group.setGoalPositionTolerance(0.005);
    group.setGoalOrientationTolerance(0.02);

    auto start = group.getCurrentState(5.0);

    if (!start) {
      throw std::runtime_error("Current robot state unavailable.");
    }

    if (std::abs(
            start->getVariablePosition("gripper_controller")) > 0.02)
    {
      throw std::runtime_error(
          "Gripper is not at its open preset. "
          "Run gripper_commander.py open first.");
    }

    waitForArmSettled(group, 20.0);

    const double cube_x = 0.200;
    const double cube_y = 0.000;
    const double cube_z = 0.0075;

    auto approach_target =
        tcpPoseFromNamedTarget(group, "grasp_approach");

    approach_target.position.x = cube_x;
    approach_target.position.y = cube_y;
    approach_target.position.z = cube_z + 0.1325;

    group.clearPoseTargets();
    group.setStartStateToCurrentState();

    RCLCPP_INFO(
        LOGGER,
        "Approach target: x=%.6f, y=%.6f, z=%.6f",
        approach_target.position.x,
        approach_target.position.y,
        approach_target.position.z);

    if (!group.setPoseTarget(approach_target, kTcpLink)) {
      throw std::runtime_error("Could not set approach target.");
    }

    MoveGroup::Plan approach_plan;

    if (!static_cast<bool>(group.plan(approach_plan))) {
      throw std::runtime_error("Approach planning failed.");
    }

    moveit_msgs::msg::DisplayTrajectory preview;
    preview.model_id = group.getRobotModel()->getName();
    preview.trajectory_start = approach_plan.start_state_;
    preview.trajectory.push_back(approach_plan.trajectory_);
    display->publish(preview);

    const ReplanFn replan_approach =
        [approach_target](MoveGroup &move_group, MoveGroup::Plan &new_plan) {
          move_group.clearPoseTargets();
          move_group.setStartStateToCurrentState();
          if (!move_group.setPoseTarget(approach_target, kTcpLink)) {
            return false;
          }
          return static_cast<bool>(move_group.plan(new_plan));
        };

    RCLCPP_INFO(LOGGER, "Executing approach.");
    executePlan(group, approach_plan, "Approach", replan_approach);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto descent_start = group.getCurrentState(5.0);

    if (!descent_start) {
      throw std::runtime_error("Descent start state unavailable.");
    }

    const auto actual_approach = poseFromState(*descent_start);
    reportPosition("Approach reached", actual_approach, approach_target);

    // Joint-space staged descent (same as gazebo_pose_commander pick_cube).
    // A single Cartesian drop often misaligns fingers or knocks the cube.
    const auto grasp_reference =
        tcpPoseFromNamedTarget(group, "grasp_approach");

    group.setMaxVelocityScalingFactor(0.05);
    group.setMaxAccelerationScalingFactor(0.05);

    planAndExecuteToNamedTarget(group, "grasp_hover", "Descend to hover");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    planAndExecuteToNamedTarget(group, "grasp_descend", "Descend to pre-grasp");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    group.setMaxVelocityScalingFactor(0.10);
    group.setMaxAccelerationScalingFactor(0.10);

    auto pregrasp_state = group.getCurrentState(5.0);

    if (!pregrasp_state) {
      throw std::runtime_error(
          "Pre-grasp state measurement unavailable.");
    }

    const auto pregrasp_target =
        tcpPoseFromNamedTarget(group, "grasp_descend");
    const auto actual_grasp = poseFromState(*pregrasp_state);
    reportPosition("Pre-grasp reached", actual_grasp, pregrasp_target);

    RCLCPP_INFO(LOGGER, "Closing gripper.");

    sendGripperCommand(
        node,
        kGripperClosed,
        kGripperCloseTimeoutSec,
        true);

    if (kGripperHoldSec > 0.0) {
      RCLCPP_INFO(
          LOGGER,
          "Holding grasp for %.1f s.",
          kGripperHoldSec);
      std::this_thread::sleep_for(
          std::chrono::duration<double>(kGripperHoldSec));
    }

    // Joint-space staged lift (matches gazebo_pose_commander LIFT_SEQUENCE).
    // Straight Cartesian lift from pre-grasp height fails collision checking.
    planAndExecuteToNamedTarget(group, "grasp_hover", "Lift to hover");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    planAndExecuteToNamedTarget(group, "grasp_approach", "Lift to carry height");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Pose place_pose;
    place_pose.position.x = place_x;
    place_pose.position.y = place_y;
    place_pose.position.z = place_z;
    place_pose.orientation = grasp_reference.orientation;

    RCLCPP_INFO(
        LOGGER,
        "Place pose (world): x=%.6f, y=%.6f, z=%.6f",
        place_x, place_y, place_z);

    planAndExecuteToPose(group, place_pose, "Place approach");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Pose place_descend = place_pose;
    place_descend.position.z = place_descend_z;

    RCLCPP_INFO(
        LOGGER,
        "Place descend target: x=%.6f, y=%.6f, z=%.6f",
        place_descend.position.x,
        place_descend.position.y,
        place_descend.position.z);

    cartesianOrPlanToPose(group, place_descend, "Place descend");

    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(LOGGER, "Opening gripper to release cube.");

    sendGripperCommand(
        node,
        kGripperOpen,
        kGripperOpenTimeoutSec,
        false);

    if (kPlaceReleaseHoldSec > 0.0) {
      RCLCPP_INFO(
          LOGGER,
          "Waiting %.1f s for cube to settle after release.",
          kPlaceReleaseHoldSec);
      std::this_thread::sleep_for(
          std::chrono::duration<double>(kPlaceReleaseHoldSec));
    }

    RCLCPP_INFO(
        LOGGER,
        "Retracting to carry height: z=%.6f",
        place_z);

    cartesianOrPlanToPose(group, place_pose, "Place retract");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    if (return_home) {
      planAndExecuteToNamedTarget(group, "home", "Return home");
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    auto final_state = group.getCurrentState(5.0);

    if (final_state) {
      const auto final_pose = poseFromState(*final_state);

      RCLCPP_INFO(
          LOGGER,
          "Pick-and-place complete. Final TCP: x=%.6f, y=%.6f, z=%.6f",
          final_pose.position.x,
          final_pose.position.y,
          final_pose.position.z);
    } else {
      RCLCPP_INFO(LOGGER, "Pick-and-place complete.");
    }

    success = true;
  }
  catch (const std::exception &error) {
    RCLCPP_ERROR(LOGGER, "%s", error.what());
    success = false;
  }

  executor.cancel();
  spinner.join();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return success ? 0 : 1;
}
