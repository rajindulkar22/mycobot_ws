#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>


moveit_msgs::msg::CollisionObject createBox(
    const std::string &id,
    const std::string &frame_id,
    double x,
    double y,
    double z,
    double size_x,
    double size_y,
    double size_z)
{
  moveit_msgs::msg::CollisionObject object;

  object.header.frame_id = frame_id;
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type =
      shape_msgs::msg::SolidPrimitive::BOX;

  primitive.dimensions = {
      size_x,
      size_y,
      size_z
  };

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation =
      moveit_msgs::msg::CollisionObject::ADD;

  return object;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "planning_scene_objects");

  moveit::planning_interface::PlanningSceneInterface
      planning_scene_interface;

  const std::string planning_frame = "world";

  auto table = createBox(
      "work_table",
      planning_frame,
      0.0,
      0.0,
      -0.030,
      0.8,
      0.6,
      0.05);

  auto cube = createBox(
      "pick_cube",
      planning_frame,
      0.20,
      0.0,
      0.4125,
      0.025,
      0.025,
      0.025);

  std::vector<moveit_msgs::msg::CollisionObject>
      collision_objects = {
          table,
          cube
      };

  RCLCPP_INFO(
      node->get_logger(),
      "Adding table and cube to the MoveIt planning scene...");

  const bool success =
      planning_scene_interface.applyCollisionObjects(
          collision_objects);

  if (!success) {
    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to add collision objects.");

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      node->get_logger(),
      "Collision objects added successfully.");

  RCLCPP_INFO(
      node->get_logger(),
      "Table: centre=(0.000, 0.000, -0.030), "
      "size=(0.800, 0.600, 0.050)");

  RCLCPP_INFO(
      node->get_logger(),
      "Cube: centre=(0.200, 0.000, 0.4125), "
      "size=(0.025, 0.025, 0.025)");

  rclcpp::shutdown();
  return 0;
}