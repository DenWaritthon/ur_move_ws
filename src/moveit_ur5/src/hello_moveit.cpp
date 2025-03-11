#include <memory>
#include <thread> 
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  rclcpp::Time now = node->get_clock()->now();
  RCLCPP_INFO(logger, "Time now: %f sec %ld nanosec", now.seconds(), now.nanoseconds());

  // Get current pose
  geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;

  // Print the current pose
  RCLCPP_INFO(node->get_logger(),
  "Current position\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f", 
  current_pose.position.x,
  current_pose.position.y,
  current_pose.position.z,
  current_pose.orientation.x,
  current_pose.orientation.y,
  current_pose.orientation.z,
  current_pose.orientation.w);

  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = current_pose.position.x + 0.1;
  target_pose.position.y = current_pose.position.y;
  target_pose.position.z = current_pose.position.z;
  target_pose.orientation.x = current_pose.orientation.x;
  target_pose.orientation.y = current_pose.orientation.y;
  target_pose.orientation.z = current_pose.orientation.z;
  target_pose.orientation.w = current_pose.orientation.w;
  move_group_interface.setPoseTarget(target_pose);

  // Print the target pose
  RCLCPP_INFO(node->get_logger(), 
  "Target position\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f",
  target_pose.position.x,
  target_pose.position.y,
  target_pose.position.z,
  target_pose.orientation.x,
  target_pose.orientation.y,
  target_pose.orientation.z,
  current_pose.orientation.w);

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
  }
      
  RCLCPP_INFO(logger, "Move Done!");
    // Get current pose
    geometry_msgs::msg::Pose end_pose = move_group_interface.getCurrentPose().pose;

    // Print the current pose
    RCLCPP_INFO(node->get_logger(),
    "End position\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f", 
    end_pose.position.x,
    end_pose.position.y,
    end_pose.position.z,
    end_pose.orientation.x,
    end_pose.orientation.y,
    end_pose.orientation.z,
    end_pose.orientation.w);
  
  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}