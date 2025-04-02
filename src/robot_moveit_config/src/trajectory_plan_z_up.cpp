#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_path_planner");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 初始化MoveGroup
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 获取当前末端姿态
  geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;

  // 设置目标姿态（沿X轴移动20cm）
  geometry_msgs::Pose target_pose = start_pose;
  target_pose.position.z += 0.1;

  // 定义路径点（起点+终点）
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose);
  waypoints.push_back(target_pose);

  // 笛卡尔路径参数设置
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;  // 禁用跳跃检测
  const double eef_step = 0.01;       // 步长1cm
  double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

  // 执行规划结果
  if (fraction >= 0.99) {
    ROS_INFO("Cartesian path planned (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
  } else {
    ROS_ERROR("Cartesian path planning failed (only %.2f%% achieved)", fraction * 100.0);
  }

  ros::shutdown();
  return 0;
}

