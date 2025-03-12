#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "target_interfaces/srv/set_target.hpp"

// Create a ROS logger
auto const logger = rclcpp::get_logger("ur5_moveit");

// Create pose goble variable
geometry_msgs::msg::Pose current_pose;
geometry_msgs::msg::Pose pick_target;
geometry_msgs::msg::Pose place_target;
geometry_msgs::msg::Pose before_pick_target;
geometry_msgs::msg::Pose before_place_target;

bool service_call = false;
bool move_fail = false;

void set_target(const std::shared_ptr<target_interfaces::srv::SetTarget::Request> request,
  std::shared_ptr<target_interfaces::srv::SetTarget::Response> response)
{                               
  // Get pick target from request
  pick_target = request->pick_target;

  // Get place target from request
  place_target = request->place_target;

  // Get before pick target
  before_pick_target = pick_target;
  before_pick_target.position.z += 0.3;

  // Get before place target
  before_place_target = place_target;
  before_place_target.position.z += 0.3;

  // Print the current pose
  RCLCPP_INFO(logger,
    "Current position\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f", 
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);

  // Print the pick target pose
  RCLCPP_INFO(logger, "Pick target\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f",
    pick_target.position.x,
    pick_target.position.y,
    pick_target.position.z,
    pick_target.orientation.x,
    pick_target.orientation.y,
    pick_target.orientation.z,
    pick_target.orientation.w);

  // Print the place target pose
  RCLCPP_INFO(logger, "Place target\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f",
    place_target.position.x,
    place_target.position.y,
    place_target.position.z,
    place_target.orientation.x,
    place_target.orientation.y,
    place_target.orientation.z,
    place_target.orientation.w);

  RCLCPP_INFO(logger, "Set target success!");
  service_call = true;
  response->success = true;
}

void get_current_pose(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
  // Get current pose
  current_pose = move_group_interface.getCurrentPose().pose;

  // Print the current pose
  RCLCPP_INFO(logger,
    "Current position\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f", 
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);
}

void move_ur5(moveit::planning_interface::MoveGroupInterface &move_group_interface, geometry_msgs::msg::Pose target_pose) {
  // Create a Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Check if the planning was successful
  bool success = (fraction > 0.95);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  // Execute the plan
  if(success) {
    RCLCPP_INFO(logger, "Planing success!");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    move_fail = true;
    return;
  }
      
  RCLCPP_INFO(logger, "Move Done!");
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur5_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set service server
  auto service = node->create_service<target_interfaces::srv::SetTarget>(
    "/set_target", &set_target);

  // Log that the service is ready
  RCLCPP_INFO(logger, "Service 'set_target' is ready");
  RCLCPP_INFO(logger, "UR5 MoveIt Node has been started");

  while (rclcpp::ok()) {
    // Check if service call is made
    if (service_call) {
      // Move to before pick target
      get_current_pose(move_group_interface);
      move_ur5(move_group_interface, before_pick_target);
      if (move_fail) {
        service_call = false;
        continue;
      }
      // Move to pick target
      get_current_pose(move_group_interface);
      move_ur5(move_group_interface, pick_target);
      if (move_fail) {
        service_call = false;
        continue;
      }
      // Move to before pick target
      get_current_pose(move_group_interface);
      move_ur5(move_group_interface, before_pick_target);
      if (move_fail) {
        service_call = false;
        continue;
      }
      // Move to before place target
      get_current_pose(move_group_interface);
      move_ur5(move_group_interface, before_place_target);
      if (move_fail) {
        service_call = false;
        continue;
      }
      // Move to place target
      get_current_pose(move_group_interface);
      move_ur5(move_group_interface, place_target);
      if (move_fail) {
        service_call = false;
        continue;
      }
      // Move to before place target
      get_current_pose(move_group_interface);
      move_ur5(move_group_interface, before_place_target);
      if (move_fail) {
        service_call = false;
        continue;
      }
      service_call = false;

      RCLCPP_INFO(logger, "All Move Done!");
    }
  }

  // Keep the node running indefinitely by waiting for the executor thread
  spinner.join();

  // Shutdown ROS (this part won't be reached unless node is stopped)
  rclcpp::shutdown();
  return 0;
}


