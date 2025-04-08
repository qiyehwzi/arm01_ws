#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ArmApproachPlanner {
private:
    moveit::planning_interface::MoveGroupInterface arm_;
    moveit::planning_interface::PlanningSceneInterface scene_;
    
public:
    ArmApproachPlanner() : arm_("manipulator") {
        arm_.setPlannerId("RRTConnect");
        arm_.setPlanningTime(10.0);  // 延长规划时间
        arm_.setMaxVelocityScalingFactor(0.2);
        arm_.setGoalPositionTolerance(0.01);  // 放宽位置容差
        arm_.setGoalOrientationTolerance(0.1);  // 放宽姿态容差
    }

bool calculateApproachPose(const geometry_msgs::Pose& obstacle_pose, 
                          geometry_msgs::Pose& target_pose,
                          double approach_distance = 0.3) {
    // 1. 获取机械臂末端当前Z轴方向
    geometry_msgs::PoseStamped current_pose = arm_.getCurrentPose();
    tf2::Quaternion q_end;
    tf2::fromMsg(current_pose.pose.orientation, q_end);
    tf2::Vector3 end_z = tf2::quatRotate(q_end, tf2::Vector3(0,0,1));

    // 2. 设置目标位置（沿末端Z轴反方向偏移）
    target_pose.position.x = 0.5;//current_pose.pose.position.x - end_z.x()*approach_distance;
    target_pose.position.y = 0;//current_pose.pose.position.y - end_z.y()*approach_distance;
    target_pose.position.z = 0.5;//.5current_pose.pose.position.z - end_z.z()*approach_distance;

    // 3. 直接继承末端当前姿态
    target_pose.orientation = current_pose.pose.orientation;

    return true;
}

    // 添加障碍物到规划场景
    void addObstacle(const std::string& id, 
                    const geometry_msgs::Pose& pose,
                    const std::vector<double>& dimensions) {
        moveit_msgs::CollisionObject obstacle;
        obstacle.header.frame_id = arm_.getPlanningFrame();
        obstacle.id = id;
        
        // 设置障碍物形状（长方体）
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = dimensions;
        
        obstacle.primitives.push_back(primitive);
        obstacle.primitive_poses.push_back(pose);
        obstacle.operation = obstacle.ADD;
        
        scene_.applyCollisionObject(obstacle);
        ROS_INFO("Added obstacle %s to planning scene", id.c_str());
    }

    // 执行运动规划
    bool executeApproach(const geometry_msgs::Pose& target_pose) {
    arm_.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    // 直接使用RRT算法规划
    if(arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) 
    {
        arm_.execute(plan);
        return true;
    }
    return false;
  }

};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "arm_approach_planner");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ArmApproachPlanner planner;
    
    //添加障碍物
    geometry_msgs::Pose obstacle_pose;
    obstacle_pose.position.x = 0.9;
    obstacle_pose.position.y = 0.0;
    obstacle_pose.position.z = 0.5;
    
    // 生成障碍物姿态
    // tf2::Quaternion q_obstacle;
    // q_obstacle.setRPY(0, -M_PI/2, 0);  // 示例姿态：Pitch=-90°
    // obstacle_pose.orientation = tf2::toMsg(q_obstacle);
    
    // planner.addObstacle("target_box", obstacle_pose, {0.288, 0.288, 0.288});
    
    //计算正对位姿
    geometry_msgs::Pose target_pose;
    if(!planner.calculateApproachPose(obstacle_pose, target_pose)) {
        ROS_ERROR("Failed to calculate approach pose");
        return -1;
    }
    
    //执行运动
    if(planner.executeApproach(target_pose)) {
        ROS_INFO("Approach movement completed");
    }
    else {
        ROS_ERROR("Movement execution failed");
    }
    
    ros::waitForShutdown();
    return 0;
}
