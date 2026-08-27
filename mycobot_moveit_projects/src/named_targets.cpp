#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

bool moveToNamedTarget(
    MoveGroupInterface &move_group,
    const std::string &target_name)
{
  RCLCPP_INFO(
      rclcpp::get_logger("named_targets"),
      "Planning movement to '%s'...",
      target_name.c_str());

      //Read the robot’s current position
  move_group.setStartStateToCurrentState();

  //Select an SRDF target
  //If the target does not exist, the function prints an error and returns false
  if (!move_group.setNamedTarget(target_name)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("named_targets"),
        "Named target '%s' does not exist.",
        target_name.c_str());
    return false;
  }

  //Create a trajectory-plan container
  //This container will hold the planned trajectory
  MoveGroupInterface::Plan plan;

  //Ask robot to compute the plan and visualize it
  //returns true if the plan is successful, false otherwise
  const bool planning_succeeded =
      static_cast<bool>(move_group.plan(plan));

  if (!planning_succeeded) {
    RCLCPP_ERROR(
        rclcpp::get_logger("named_targets"),
        "Planning to '%s' failed.",
        target_name.c_str());
    return false;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("named_targets"),
      "Plan found. Executing '%s'...",
      target_name.c_str());

      //Execute the planned trajectory
      //returns true if the execution is successful, false otherwise
  const bool execution_succeeded =
      static_cast<bool>(move_group.execute(plan));

  if (!execution_succeeded) {
    RCLCPP_ERROR(
        rclcpp::get_logger("named_targets"),
        "Execution to '%s' failed.",
        target_name.c_str());
    return false;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("named_targets"),
      "Reached '%s'.",
      target_name.c_str());

  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "moveit_named_targets",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  MoveGroupInterface move_group(node, "arm"); //Create a MoveGroupInterface object for the arm

  move_group.setMaxVelocityScalingFactor(0.10); //Set the maximum velocity scaling factor
  move_group.setMaxAccelerationScalingFactor(0.10); //Set the maximum acceleration scaling factor
  move_group.setPlanningTime(10.0); //Set the planning time
  move_group.setNumPlanningAttempts(10); //Set the number of planning attempts

  //Print the planning frame
  RCLCPP_INFO(
      node->get_logger(), //Print the planning frame
      "Planning frame: %s", //Print the planning frame
      move_group.getPlanningFrame().c_str());

  const std::vector<std::string> sequence = { //Create a sequence of targets
      "home",
      "grasp_approach",
      "home"
  };

  bool sequence_succeeded = true; //Initialize the sequence succeeded flag to true

  for (const auto &target : sequence) {
    if (!moveToNamedTarget(move_group, target)) { //Move to the named target
      sequence_succeeded = false;
      break;
    }

    std::this_thread::sleep_for(
        std::chrono::seconds(2));
  }

  if (sequence_succeeded) {
    RCLCPP_INFO(
        node->get_logger(),
        "Sequence completed: home -> grasp_approach -> home");
  } else {
    RCLCPP_ERROR(
        node->get_logger(),
        "Sequence stopped because a movement failed.");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return sequence_succeeded ? 0 : 1;
}