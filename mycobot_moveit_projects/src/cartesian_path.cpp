#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp> // rclcpp/rclcpp.hpp
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

using MoveGroup =
    moveit::planning_interface::MoveGroupInterface;

const auto LOGGER = rclcpp::get_logger("cartesian_path");
const std::string TCP = "gripper_tcp"; //TCP identifies the robot link whose position and orientation we control.


bool namedMove(MoveGroup &group, const std::string &name) //namedMove is a function that moves the robot to a named target.

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
bool readPose(MoveGroup &group, geometry_msgs::msg::Pose &pose) //readPose is a function that reads the current joint state and calculates the TCP pose using the forward kinematics.
{ //group.getCurrentState(5.0) is a function that gets the current state of the robot.
  //5.0 is the timeout in seconds.
  //state is a pointer to the current state of the robot.
  //state->update() is a function that updates the state of the robot.
  //state->getGlobalLinkTransform(TCP) is a function that gets the global transform of the TCP.
  //const Eigen::Quaterniond quaternion(transform.rotation()) is a function that gets the quaternion of the TCP.
  //pose.position.x = transform.translation().x() is a function that gets the x position of the TCP.
  //pose.position.y = transform.translation().y() is a function that gets the y position of the TCP.
  //pose.position.z = transform.translation().z() is a function that gets the z position of the TCP.
  auto state = group.getCurrentState(5.0);

  if (!state) {
    RCLCPP_ERROR(LOGGER, "No current robot state available.");
    return false;
  }

  state->update();
  const auto &transform = state->getGlobalLinkTransform(TCP);
  const Eigen::Quaterniond quaternion(transform.rotation());
  //The transform contains a rotation matrix. This converts that rotation into a quaternion.
  //The quaternion is then used to set the orientation of the pose.
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  return true;
}


bool cartesianMove( //generate and execute the straight path between the current and target poses.
    MoveGroup &group,
    const geometry_msgs::msg::Pose &target)
{
  //start is a pointer to the current state of the robot.
  //5.0 is the timeout in seconds.
  //start is a pointer to the current state of the robot.
  //start->update() is a function that updates the state of the robot.
  //start->getGlobalLinkTransform(TCP) is a function that gets the global transform of the TCP.
  //const Eigen::Quaterniond quaternion(transform.rotation()) is a function that gets the quaternion of the TCP.
  //pose.position.x = transform.translation().x() is a function that gets the x position of the TCP.
  //pose.position.y = transform.translation().y() is a function that gets the y position of the TCP.
  //pose.position.z = transform.translation().z() is a function that gets the z position of the TCP.
  auto start = group.getCurrentState(5.0);

  if (!start) {
    RCLCPP_ERROR(LOGGER, "Cannot obtain Cartesian start state.");
    return false;
  }

  RCLCPP_INFO(
      LOGGER,
      "Cartesian start joint2_to_joint1: %.6f",
      start->getVariablePosition("joint2_to_joint1"));

  group.clearPoseTargets(); //clear the current pose targets.
  group.setStartState(*start); //set the start state of the robot.

  std::vector<geometry_msgs::msg::Pose> waypoints{target}; //waypoints is a vector of poses.
  moveit_msgs::msg::RobotTrajectory trajectory; //trajectory is a message that contains the robot trajectory.

  // 2 mm interpolation steps.
  // 5.0 enables relative joint-jump checking.
  // true enables collision checking.
  const double fraction = group.computeCartesianPath(
      waypoints, 0.002, 5.0, trajectory, true);

  RCLCPP_INFO(
      LOGGER, "Cartesian path completion: %.2f%%",
      fraction * 100.0);
//The fraction is a value between 0 and 1 that indicates the completion of the path.
//If the fraction is not finite, the path is incomplete.
//If the fraction is less than 0.999999, the path is incomplete.
//If the trajectory has less than 2 points, the path is incomplete.
  if (!std::isfinite(fraction) || //std::isfinite is a function that checks if the fraction is finite.
      fraction < 0.999999 ||
      trajectory.joint_trajectory.points.size() < 2) //trajectory.joint_trajectory.points.size() is the number of points in the trajectory.
  {
    RCLCPP_ERROR(
        LOGGER, "Incomplete Cartesian path. Execution skipped.");
    return false;
  }

  robot_trajectory::RobotTrajectory timed( //timed is a robot trajectory that contains the timed robot trajectory.
      group.getRobotModel(), "arm"); //group.getRobotModel() is the robot model.
  //"arm" is the name of the robot.
  //*start is the start state of the robot.
  //trajectory is the robot trajectory.

  //This creates a trajectory object connected to the robot model and arm group.
  //This sets the robot trajectory to the timed robot trajectory.
  timed.setRobotTrajectoryMsg(*start, trajectory); //timed.setRobotTrajectoryMsg(*start, trajectory) is a function that sets the robot trajectory.
  //This sets the robot trajectory to the timed robot trajectory.
  trajectory_processing::TimeOptimalTrajectoryGeneration timing;

  if (!timing.computeTimeStamps(timed, 0.10, 0.10)) {
    RCLCPP_ERROR(LOGGER, "Trajectory timing failed.");
    return false;
  }

  timed.getRobotTrajectoryMsg(trajectory);

  const auto &joint_names = trajectory.joint_trajectory.joint_names;
  const auto &first_point = trajectory.joint_trajectory.points.front();

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(
        LOGGER,
        "Trajectory start %s: %.6f",
        joint_names[i].c_str(),
        first_point.positions[i]);
  }

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

    // Wait for this node to receive Gazebo's simulation clock.
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);

    while (rclcpp::ok() && !node->get_clock()->ros_time_is_active()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error(
            "No simulation clock received. Launch gazebo_sim.launch.py first "
            "and verify: ros2 topic echo /clock --once");
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(
        LOGGER, "Simulation clock active: %.3f s", node->now().seconds());

    // Start receiving joint states before the first motion.
    if (!group.startStateMonitor(5.0)) {
      throw std::runtime_error("Could not initialize joint-state monitor.");
    }

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