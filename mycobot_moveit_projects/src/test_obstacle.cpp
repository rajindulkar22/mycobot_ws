#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "test_obstacle_manager");

  const std::string operation =
      node->declare_parameter<std::string>(
          "operation",
          "add");

  moveit::planning_interface::PlanningSceneInterface
      planning_scene_interface;

  moveit_msgs::msg::CollisionObject obstacle;

  obstacle.header.frame_id = "world";
  obstacle.id = "test_obstacle";

  if (operation == "remove") {
    obstacle.operation =
        moveit_msgs::msg::CollisionObject::REMOVE;

    const bool removed =
        planning_scene_interface.applyCollisionObject(
            obstacle);

    if (!removed) {
      RCLCPP_ERROR(
          node->get_logger(),
          "Failed to remove test obstacle.");

      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Test obstacle removed.");

    rclcpp::shutdown();
    return 0;
  }

  if (operation != "add") {
    RCLCPP_ERROR(
        node->get_logger(),
        "Unknown operation '%s'. Use 'add' or 'remove'.",
        operation.c_str());

    rclcpp::shutdown();
    return 1;
  }

  shape_msgs::msg::SolidPrimitive box;
  box.type =
      shape_msgs::msg::SolidPrimitive::BOX;

  box.dimensions = {
      0.10,
      0.10,
      0.10
  };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.101;
  pose.position.y = -0.065;
  pose.position.z = 0.437;
  pose.orientation.w = 1.0;

  obstacle.primitives.push_back(box);
  obstacle.primitive_poses.push_back(pose);
  obstacle.operation =
      moveit_msgs::msg::CollisionObject::ADD;

  const bool added =
      planning_scene_interface.applyCollisionObject(
          obstacle);

  if (!added) {
    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to add test obstacle.");

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      node->get_logger(),
      "Test obstacle added at the grasp_approach TCP pose.");

  RCLCPP_INFO(
      node->get_logger(),
      "Centre=(0.101, -0.065, 0.437), "
      "size=(0.10, 0.10, 0.10)");

  rclcpp::shutdown();
  return 0;
}