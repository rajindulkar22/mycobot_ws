// cube_approach.cpp — vision-guided pick-and-place for the myCobot 280.
//
// Pipeline:
//   color_cube_detector + pixel_to_world  →  /selected_cube/world_center
//   MoveIt arm planning  →  gripper close  →  attach cube in planning scene
//   carry  →  place  →  detach cube  →  gripper open  →  optional home
//
// Requires Gazebo sim clock, move_group, gripper action server, and vision nodes.

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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <mutex>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace
{
// Anonymous namespace: helpers and constants are file-local.

using MoveGroup =
    moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface =
    moveit::planning_interface::PlanningSceneInterface;
using GripperCommand = control_msgs::action::GripperCommand;

const auto LOGGER = rclcpp::get_logger("cube_approach");

// MoveIt planning frame (= robot base / joint1 origin).
const std::string kPlanningFrame = "world";
// Tool-centre-point link used for Cartesian goals.
const std::string kTcpLink = "gripper_tcp";
// Planning-scene ID for the 25 mm pick cube (matches SDF model name).
const std::string kCubeId = "pick_cube";
// Planning-scene ID for the work table collision box.
const std::string kTableId = "work_table";
// ros2_control gripper action (Gazebo adaptive gripper).
const std::string kGripperAction =
    "/gripper_action_controller/gripper_cmd";

constexpr double kCubeSize = 0.025;  // 25 mm cube edge length

// Links allowed to touch the attached cube without self-collision flags.
const std::vector<std::string> kGripperTouchLinks = {
    "gripper_tcp",
    "gripper_base",
    "gripper_left1",
    "gripper_left2",
    "gripper_left3",
    "gripper_right1",
    "gripper_right2",
    "gripper_right3",
};

// Gripper joint position targets (radians) and timing.
constexpr double kGripperOpen = 0.0;
constexpr double kGripperClosed = -0.55;
constexpr double kGripperMaxEffort = 8.0;
constexpr double kGripperHoldSec = 1.5;           // dwell after close before lift
constexpr double kGripperCloseTimeoutSec = 3.0;    // close may stall on cube
constexpr double kGripperOpenTimeoutSec = 15.0;
constexpr double kPlaceReleaseHoldSec = 0.5;     // wait after open before retract

// joint2_to_joint1 value in grasp_approach, tuned for top-down reach toward +X.
constexpr double kGraspReferenceJoint1 = 0.330216;
// Vision settle time after first detection (seconds).
constexpr double kVisionSettleSec = 1.5;
// TCP height offsets relative to cube centre (metres).
constexpr double kHoverOffsetZ = 0.0975;
constexpr double kPregraspOffsetZ = 0.046872;
constexpr double kCarryOffsetZ = 0.1325;

// Read TCP pose (position + orientation) from a forward-kinematics RobotState.
geometry_msgs::msg::Pose poseFromState(
    moveit::core::RobotState &state)
{
  state.update();  // propagate joint values to link transforms

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


// Resolve a SRDF named target (e.g. "grasp_approach") to a TCP pose via FK.
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


// Point joint2_to_joint1 at the cube bearing (grasp_approach tuned at (0.20, 0)).
double joint1ForCube(double cube_x, double cube_y)
{
  return kGraspReferenceJoint1 + std::atan2(cube_y, cube_x);
}


// grasp_approach joint values with joint2_to_joint1 aimed at the detected cube.
std::vector<double> graspApproachJointsForCube(
    MoveGroup &group,
    double cube_x,
    double cube_y)
{
  if (!group.setNamedTarget("grasp_approach")) {
    throw std::runtime_error("Unknown named target: grasp_approach");
  }

  std::vector<double> joints;
  group.getJointValueTarget(joints);
  group.clearPoseTargets();

  if (joints.empty()) {
    throw std::runtime_error(
        "Could not read grasp_approach joint values.");
  }

  const auto *joint_group =
      group.getRobotModel()->getJointModelGroup(group.getName());
  const auto &names = joint_group->getActiveJointModelNames();

  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == "joint2_to_joint1") {
      const double joint1 = joint1ForCube(cube_x, cube_y);
      RCLCPP_INFO(
          LOGGER,
          "Adjusting joint2_to_joint1 to %.6f rad for cube at (%.3f, %.3f).",
          joint1,
          cube_x,
          cube_y);
      joints[i] = joint1;
      break;
    }
  }

  return joints;
}


// Build a TCP goal with vision XY/Z and orientation copied from a reference pose.
geometry_msgs::msg::Pose visionTcpTarget(
    const geometry_msgs::msg::Pose &orientation_ref,
    double cube_x,
    double cube_y,
    double z)
{
  geometry_msgs::msg::Pose target = orientation_ref;
  target.position.x = cube_x;
  target.position.y = cube_y;
  target.position.z = z;
  return target;
}


// Build a box CollisionObject message (ADD or REMOVE).
moveit_msgs::msg::CollisionObject createBoxObject(
    const std::string &id,
    const std::string &frame_id,
    double x,
    double y,
    double z,
    double size_x,
    double size_y,
    double size_z,
    uint8_t operation)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = operation;
  return object;
}


// Simple pose with identity orientation at the cube centre (world frame).
geometry_msgs::msg::Pose cubeCenterPose(
    double cube_x,
    double cube_y,
    double cube_z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = cube_x;
  pose.position.y = cube_y;
  pose.position.z = cube_z;
  pose.orientation.w = 1.0;
  return pose;
}


// Transform a world-frame pose into a link's local frame (for attach offset).
geometry_msgs::msg::Pose poseInLinkFrame(
    moveit::core::RobotState &state,
    const std::string &link_name,
    const geometry_msgs::msg::Pose &pose_in_world)
{
  state.update();

  const Eigen::Isometry3d link_transform =
      state.getGlobalLinkTransform(link_name);

  Eigen::Isometry3d world_pose = Eigen::Isometry3d::Identity();
  world_pose.translation() = Eigen::Vector3d(
      pose_in_world.position.x,
      pose_in_world.position.y,
      pose_in_world.position.z);

  Eigen::Quaterniond world_quat(
      pose_in_world.orientation.w,
      pose_in_world.orientation.x,
      pose_in_world.orientation.y,
      pose_in_world.orientation.z);
  world_quat.normalize();
  world_pose.linear() = world_quat.toRotationMatrix();

  const Eigen::Isometry3d link_pose =
      link_transform.inverse() * world_pose;

  geometry_msgs::msg::Pose result;
  result.position.x = link_pose.translation().x();
  result.position.y = link_pose.translation().y();
  result.position.z = link_pose.translation().z();

  Eigen::Quaterniond link_quat(link_pose.rotation());
  link_quat.normalize();
  result.orientation.x = link_quat.x();
  result.orientation.y = link_quat.y();
  result.orientation.z = link_quat.z();
  result.orientation.w = link_quat.w();

  return result;
}


// Brief pause so planning-scene updates propagate before the next plan.
void waitForPlanningSceneSync()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}


// Remove any previous pick_cube from scene (attached or free-floating).
void clearCubeFromScene(PlanningSceneInterface &scene)
{
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.link_name = kTcpLink;
  detach.object.id = kCubeId;
  detach.object.operation =
      moveit_msgs::msg::CollisionObject::REMOVE;
  scene.applyAttachedCollisionObject(detach);
  scene.removeCollisionObjects({kCubeId});
  waitForPlanningSceneSync();
}


// Add work_table box to MoveIt scene (Gazebo has its own table; this is for planning).
void ensureTableInScene(PlanningSceneInterface &scene)
{
  clearCubeFromScene(scene);

  // Table top ~0.4 m in Gazebo; box centre at z = -0.03 in MoveIt world frame.
  const auto table = createBoxObject(
      kTableId,
      kPlanningFrame,
      0.0,
      0.0,
      -0.030,
      0.8,
      0.6,
      0.05,
      moveit_msgs::msg::CollisionObject::ADD);

  if (!scene.applyCollisionObjects({table})) {
    throw std::runtime_error(
        "Failed to add work_table to planning scene.");
  }

  waitForPlanningSceneSync();

  RCLCPP_INFO(
      LOGGER,
      "Planning scene: work_table added (world frame).");
}


// After physical grasp in Gazebo, attach cube to TCP in MoveIt for carry planning.
void attachCubeToGripper(
    MoveGroup &group,
    PlanningSceneInterface &scene,
    const geometry_msgs::msg::Pose &cube_in_world)
{
  auto state = group.getCurrentState(5.0);

  if (!state) {
    throw std::runtime_error(
        "Cannot attach cube: current robot state unavailable.");
  }

  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = kTcpLink;
  attached.object = createBoxObject(
      kCubeId,
      kTcpLink,
      0.0,
      0.0,
      0.0,
      kCubeSize,
      kCubeSize,
      kCubeSize,
      moveit_msgs::msg::CollisionObject::ADD);
  attached.object.header.frame_id = kTcpLink;
  // Express cube pose relative to gripper_tcp at grasp time.
  attached.object.primitive_poses.at(0) =
      poseInLinkFrame(*state, kTcpLink, cube_in_world);
  attached.touch_links = kGripperTouchLinks;

  if (!scene.applyAttachedCollisionObject(attached)) {
    throw std::runtime_error(
        "Failed to attach pick_cube to gripper_tcp.");
  }

  waitForPlanningSceneSync();
  group.setStartStateToCurrentState();

  RCLCPP_INFO(
      LOGGER,
      "Attached pick_cube to gripper_tcp for carry planning.");
}


// Detach cube from gripper and re-add it as a free object at the place location.
void detachAndPlaceCube(
    MoveGroup &group,
    PlanningSceneInterface &scene,
    const geometry_msgs::msg::Pose &cube_in_world)
{
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.link_name = kTcpLink;
  detach.object.id = kCubeId;
  detach.object.operation =
      moveit_msgs::msg::CollisionObject::REMOVE;

  if (!scene.applyAttachedCollisionObject(detach)) {
    throw std::runtime_error(
        "Failed to detach pick_cube from gripper_tcp.");
  }

  waitForPlanningSceneSync();

  const auto placed = createBoxObject(
      kCubeId,
      kPlanningFrame,
      cube_in_world.position.x,
      cube_in_world.position.y,
      cube_in_world.position.z,
      kCubeSize,
      kCubeSize,
      kCubeSize,
      moveit_msgs::msg::CollisionObject::ADD);

  if (!scene.applyCollisionObject(placed)) {
    throw std::runtime_error(
        "Failed to place pick_cube in planning scene.");
  }

  waitForPlanningSceneSync();
  group.setStartStateToCurrentState();

  RCLCPP_INFO(
      LOGGER,
      "Detached pick_cube; placed at x=%.6f, y=%.6f, z=%.6f (world).",
      cube_in_world.position.x,
      cube_in_world.position.y,
      cube_in_world.position.z);
}


// Log measured TCP vs commanded target and total position error (mm).
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


// Send open/close goal to gripper_action_controller.
// accept_stall=true on close: object contact may prevent reaching goal (normal).
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
      // Fingers may stop early when contacting the cube — not a hard failure.
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


// Wait until all arm joint velocities fall below tolerance (sim settling).
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


// Open gripper if a prior pick left it closed (sort_cubes chain).
void ensureGripperOpen(
    const rclcpp::Node::SharedPtr &node,
    MoveGroup &group)
{
  constexpr double kOpenTolerance = 0.02;

  auto state = group.getCurrentState(5.0);

  if (!state) {
    throw std::runtime_error(
        "Gripper state unavailable before pick.");
  }

  const double gripper_pos =
      state->getVariablePosition("gripper_controller");

  if (std::abs(gripper_pos) <= kOpenTolerance) {
    return;
  }

  RCLCPP_WARN(
      LOGGER,
      "Gripper not open (%.3f rad); opening before pick.",
      gripper_pos);

  sendGripperCommand(
      node,
      kGripperOpen,
      kGripperOpenTimeoutSec,
      true);

  waitForArmSettled(group, 10.0);

  state = group.getCurrentState(5.0);

  if (!state) {
    throw std::runtime_error(
        "Gripper state unavailable after open command.");
  }

  const double after_open =
      state->getVariablePosition("gripper_controller");

  if (std::abs(after_open) > kOpenTolerance) {
    throw std::runtime_error(
        "Gripper did not reach open preset after command.");
  }
}


// Callback type for one-shot re-plan after a failed execute().
using ReplanFn = std::function<bool(MoveGroup &, MoveGroup::Plan &)>;


// Execute a plan; on failure settle, re-plan once, and retry.
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


// Build a straight-line Cartesian path to target; time-parameterize at 10% vel/acc.
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
      0.002,   // eef step size (m)
      5.0,     // jump threshold (m) — reject discontinuous segments
      trajectory,
      true);   // avoid collisions

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


// Plan and execute a Cartesian move; throws if path completion < 100%.
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


// Plan and execute to an SRDF named joint target (e.g. "home").
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


// Plan and execute to explicit arm joint values (e.g. bearing-adjusted grasp_approach).
void planAndExecuteToJointValues(
    MoveGroup &group,
    const std::vector<double> &joint_values,
    const std::string &label)
{
  group.clearPoseTargets();
  group.setStartStateToCurrentState();

  if (!group.setJointValueTarget(joint_values)) {
    throw std::runtime_error(
        "Could not set joint target for " + label + ".");
  }

  MoveGroup::Plan plan;

  RCLCPP_INFO(LOGGER, "Planning %s.", label.c_str());

  if (!static_cast<bool>(group.plan(plan))) {
    throw std::runtime_error("Planning failed for " + label + ".");
  }

  const ReplanFn replan =
      [joint_values](MoveGroup &move_group, MoveGroup::Plan &new_plan) {
        move_group.clearPoseTargets();
        move_group.setStartStateToCurrentState();
        if (!move_group.setJointValueTarget(joint_values)) {
          return false;
        }
        return static_cast<bool>(move_group.plan(new_plan));
      };

  RCLCPP_INFO(LOGGER, "Executing %s.", label.c_str());
  executePlan(group, plan, label, replan);
}


// Plan and execute to an arbitrary TCP pose via OMPL sampling planner.
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


// Prefer straight Cartesian descent; fall back to OMPL if Cartesian fails.
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

  // Parameters (place_x/y/z, return_home, etc.) come from launch YAML overrides.
  auto node = rclcpp::Node::make_shared(
      "cube_approach",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));
    

  // Shared state for the latest vision detection (written by callback, read by main).
  std::mutex cube_mutex;
  bool cube_received = false;
  geometry_msgs::msg::PointStamped detected_cube;

  // Subscribe to 3D cube centre from pixel_to_world (fed by color_cube_detector).
  // Topic chain: /overhead_camera/image → color_cube_detector → pixel_to_world.
  auto cube_subscription =
      node->create_subscription<geometry_msgs::msg::PointStamped>(
          "/selected_cube/world_center",
          10,
          [&](geometry_msgs::msg::PointStamped::SharedPtr message) {
            // Only accept points already transformed into the MoveIt world frame.
            if (message->header.frame_id != "world") {
              return;
            }

            std::lock_guard<std::mutex> lock(cube_mutex);
            detected_cube = *message;
            cube_received = true;
          });

  // Spin subscriptions on a background thread so main can block on detection.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  bool success = false;

  try {
    // --- Wait for Gazebo /clock (sim time must be active before MoveIt calls) ---
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

    // --- Place pose parameters (world frame, metres) ---
    const double place_x = node->get_parameter("place_x").as_double();
    const double place_y = node->get_parameter("place_y").as_double();
    const double place_z = node->get_parameter("place_z").as_double();
    const double place_descend_z =
        node->get_parameter("place_descend_z").as_double();
    const bool return_home =
        node->get_parameter("return_home").as_bool();

    // --- MoveIt arm group setup ---
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

    ensureGripperOpen(node, group);

    waitForArmSettled(group, 20.0);

    // Add table collision geometry so plans avoid the tabletop.
    PlanningSceneInterface planning_scene;
    ensureTableInScene(planning_scene);

    // --- Wait for vision: first message on /selected_cube/world_center ---
    RCLCPP_INFO(
        LOGGER,
        "Waiting for cube position from /selected_cube/world_center...");

    // Block up to 10 s for the first valid detection before planning.
    const auto detection_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(10);

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(cube_mutex);

        if (cube_received) {
          break;
        }
      }

      if (std::chrono::steady_clock::now() >= detection_deadline) {
        throw std::runtime_error(
            "No cube detection received within 10 seconds.");
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(50));
    }

    // Let detections settle, then use the latest world_center (not the first).
    RCLCPP_INFO(
        LOGGER,
        "Stabilizing vision for %.1f s before planning.",
        kVisionSettleSec);

    const auto stabilize_until =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(kVisionSettleSec);

    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() < stabilize_until)
    {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(50));
    }

    // Copy stabilized cube pose out of the shared buffer.
    geometry_msgs::msg::PointStamped cube;

    {
      std::lock_guard<std::mutex> lock(cube_mutex);
      cube = detected_cube;
    }

    // Cube centre in MoveIt world frame (metres).
    const double cube_x = cube.point.x;
    const double cube_y = cube.point.y;
    const double cube_z = cube.point.z;

    if (!std::isfinite(cube_x) ||
        !std::isfinite(cube_y) ||
        !std::isfinite(cube_z))
    {
      throw std::runtime_error(
          "Received invalid cube coordinates.");
    }

    RCLCPP_INFO(
        LOGGER,
        "Vision cube position: x=%.6f, y=%.6f, z=%.6f",
        cube_x,
        cube_y,
        cube_z);

    const auto pick_cube_center =
        cubeCenterPose(cube_x, cube_y, cube_z);

    // --- Phase 1: Rotate base toward cube, then Cartesian align over it ---
    // OMPL pose approach with fixed grasp_approach orientation arcs past off-axis cubes.
    const auto approach_joints =
        graspApproachJointsForCube(group, cube_x, cube_y);

    planAndExecuteToJointValues(
        group,
        approach_joints,
        "Align base toward cube");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto align_state = group.getCurrentState(5.0);

    if (!align_state) {
      throw std::runtime_error("Align state unavailable.");
    }

    const auto grasp_reference = poseFromState(*align_state);

    const geometry_msgs::msg::Pose hover_target =
        visionTcpTarget(
            grasp_reference,
            cube_x,
            cube_y,
            cube_z + kHoverOffsetZ);

    const geometry_msgs::msg::Pose pregrasp_target =
        visionTcpTarget(
            grasp_reference,
            cube_x,
            cube_y,
            cube_z + kPregraspOffsetZ);

    const geometry_msgs::msg::Pose carry_target =
        visionTcpTarget(
            grasp_reference,
            cube_x,
            cube_y,
            cube_z + kCarryOffsetZ);

    // --- Phase 2: Cartesian descent to hover and pre-grasp ---
    // Slower motion during delicate descent.
    group.setMaxVelocityScalingFactor(0.05);
    group.setMaxAccelerationScalingFactor(0.05);

    cartesianOrPlanToPose(
        group,
        hover_target,
        "Descend to vision hover");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    cartesianOrPlanToPose(
        group,
        pregrasp_target,
        "Descend to vision pre-grasp");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    group.setMaxVelocityScalingFactor(0.10);
    group.setMaxAccelerationScalingFactor(0.10);

    auto pregrasp_state = group.getCurrentState(5.0);

    if (!pregrasp_state) {
      throw std::runtime_error(
          "Pre-grasp state measurement unavailable.");
    }

    const auto actual_grasp = poseFromState(*pregrasp_state);
    reportPosition("Pre-grasp reached", actual_grasp, pregrasp_target);

    // TCP-to-cube offset at grasp; reused to infer cube pose at release.
    const double cube_offset_x = cube_x - actual_grasp.position.x;
    const double cube_offset_y = cube_y - actual_grasp.position.y;
    const double cube_offset_z = cube_z - actual_grasp.position.z;

    // --- Phase 3: Close gripper (Gazebo physics grasp) ---
    RCLCPP_INFO(LOGGER, "Closing gripper.");

    sendGripperCommand(
        node,
        kGripperClosed,
        kGripperCloseTimeoutSec,
        true);  // accept stall when fingers hit the cube

    if (kGripperHoldSec > 0.0) {
      RCLCPP_INFO(
          LOGGER,
          "Holding grasp for %.1f s.",
          kGripperHoldSec);
      std::this_thread::sleep_for(
          std::chrono::duration<double>(kGripperHoldSec));
    }

    // Sync MoveIt planning scene: cube follows TCP during carry.
    attachCubeToGripper(
        group,
        planning_scene,
        pick_cube_center);

    // --- Phase 4: Lift with attached cube ---
    cartesianOrPlanToPose(
        group,
        hover_target,
        "Lift from vision grasp");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    cartesianOrPlanToPose(
        group,
        carry_target,
        "Lift to vision carry height");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // --- Phase 5: Move to place location (parameters from launch file) ---
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

    // Descend to release height at place location.
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

    // --- Phase 6: Detach cube in planning scene, then open gripper ---
    auto release_state = group.getCurrentState(5.0);

    if (!release_state) {
      throw std::runtime_error(
          "Release state measurement unavailable.");
    }

    const auto release_tcp = poseFromState(*release_state);
    // Estimate where cube centre ended up using grasp-time TCP offset.
    geometry_msgs::msg::Pose released_cube_center;
    released_cube_center.position.x =
        release_tcp.position.x + cube_offset_x;
    released_cube_center.position.y =
        release_tcp.position.y + cube_offset_y;
    released_cube_center.position.z =
        release_tcp.position.z + cube_offset_z;
    released_cube_center.orientation.w = 1.0;

    detachAndPlaceCube(
        group,
        planning_scene,
        released_cube_center);

    RCLCPP_INFO(LOGGER, "Opening gripper to release cube.");

    sendGripperCommand(
        node,
        kGripperOpen,
        kGripperOpenTimeoutSec,
        true);  // sim may stall; cube already detached in planning scene

    if (kPlaceReleaseHoldSec > 0.0) {
      RCLCPP_INFO(
          LOGGER,
          "Waiting %.1f s for cube to settle after release.",
          kPlaceReleaseHoldSec);
      std::this_thread::sleep_for(
          std::chrono::duration<double>(kPlaceReleaseHoldSec));
    }

    // --- Phase 7: Retract and optionally return home ---
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

  // Stop background executor and join spinner thread.
  executor.cancel();
  spinner.join();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return success ? 0 : 1;
}
