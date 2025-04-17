#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdint.h>
#include <ctime>

struct SharedData{
    pthread_mutex_t mutex;       // 互斥锁确保独占访问
    double matrix[4][4];         // 4x4变换矩阵
    short color;                 // 我方颜色
    double x,y,z;                //
    uint64_t version;            // 版本号
};

#define SHM_NAME "/transform_matrix_shm"
#define SHM_SIZE sizeof(SharedData)


/* 原子化写入（统一矩阵类型为 double） */
void update_shared_data(
    SharedData* data,
    short new_color
) {
    pthread_mutex_lock(&data->mutex); // 安全复制
    data->color = new_color;
    pthread_mutex_unlock(&data->mutex);
}

/* 原子化读取（完整数据拷贝） */
void read_shared_data(
    SharedData* data,
    double out_matrix[4][4],
    double* x,
    double* y,
    double* z,
    uint64_t* out_version
) {
    pthread_mutex_lock(&data->mutex);
    memcpy(out_matrix, data->matrix, sizeof(double[4][4]));
    *out_version = data->version;
    *x = data->x;
    *y = data->y;
    *z = data->z;
    pthread_mutex_unlock(&data->mutex);
}

class camera_message{
public:

    double matrix[4][4];
    double position_center[3];
    uint64_t version;            // 版本号
}cmes;
uint64_t lastversion;

class pose_and_position{
public:
  double dimension;
  tf2::Matrix3x3 R_final;
  tf2::Quaternion q;
  double position_x;
  double position_y;
  double position_z;
  double position_x_near;
  double position_y_near;
  double position_z_near;
  double distance;
};

// 判断是否已成功添加物体到规划场景，或是否成功把物体附着到机械臂上
bool waitForStateUpdate(std::string obstacle_name, moveit::planning_interface::PlanningSceneInterface& scene, bool obstacle_is_known=false, bool object_is_attached=false, double  timeout=4){
  ros::Time start_time = ros::Time::now();
  ros::Time seconds = ros::Time::now();
  bool is_attached, is_known;
  while(seconds - start_time <ros::Duration(timeout) && !ros::isShuttingDown()){
    std::vector<std::string> attached_object_ids;
    attached_object_ids.push_back(obstacle_name);
    // 获取由指定ID列表标记的附着对象物体
    auto attached_objects = scene.getAttachedObjects(attached_object_ids);
    is_attached = attached_objects.size();
    // 获取规划场景中已有的所有物体的ID
    std::vector<std::string> world_object_names = scene.getKnownObjectNames();
    if(std::find(world_object_names.begin(),world_object_names.end(), obstacle_name) != world_object_names.end()){
      is_known =true;
    }
    else {
      is_known =false;
    }
    if(object_is_attached == is_attached && obstacle_is_known == is_known)
      return true;
    usleep(100000);
    seconds = ros::Time::now();
  }
  return false;
}

int main(int argc, char **argv){
  //shared_memory  
  int fd = -1;
  while(fd == -1) {
      fd = shm_open(SHM_NAME, O_RDWR | O_CREAT, 0666);
      if (fd == -1) {
          std::cerr << "Failed to create shared memory." << std::endl;
      }
  }
  //fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
  SharedData* shm = (SharedData*)mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);

  //read data
  double matrix[4][4];
  double x,y,z;
  read_shared_data(shm, matrix, &x, &y, &z, &lastversion);

  // 更新数据
  short color = 1; // 1 for red, 2 for blue
  update_shared_data(shm, color);

  ros::init(argc, argv, "moveit_planning_scene_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface arm("manipulator");
  // 创建PlanningSceneInterface对象scene用来对规划场景进行操作
  moveit::planning_interface::PlanningSceneInterface scene;
  // 设置一个比例因子以选择性地降低最大关节速度限制，可取值为(0,1]
  arm.setMaxVelocityScalingFactor(0.3);

  pose_and_position block_pp;
  pose_and_position arm_pp;

  block_pp.dimension = 0.288;

  // 欧拉角 → 旋转矩阵
  tf2::Matrix3x3 R1;
  tf2::Matrix3x3 R2;

  // 旋转矩阵 → 四元数
  block_pp.R_final.getRotation(block_pp.q);

  auto last_time = std::chrono::high_resolution_clock::now(); // 获取当前高精度时间点
  int time_diff_ms = 0; // 初始化时间差为0

  while (time_diff_ms <= 2000) {
  auto now_time = std::chrono::high_resolution_clock::now(); // 获取当前高精度时间点
  time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - last_time).count(); // 计算时间差（毫秒）
  //读取数据
  double matrix[4][4];
  uint64_t version;
  double x,y,z;
  read_shared_data(shm, matrix, &x, &y, &z, &version);
  if(version > lastversion)
  {
    //updata
    if(!abs(cmes.matrix[0][3]) < 0.01 && !abs(cmes.matrix[1][3]) < 0.01 &&! !abs(cmes.matrix[2][3]) < 0.01)
    {
      for(int i = 0;i < 4;i++)
      {
        for(int j = 0;j < 4;j++)
        {
          cmes.matrix[i][j]= matrix[i][j];
        }
      }
      cmes.position_center[0] = x;
      cmes.position_center[1] = y;
      cmes.position_center[2] = z;
    }
    else{
      for(int i = 0;i < 4;i++)
      {
        for(int j = 0;j < 4;j++)
        {
          cmes.matrix[i][j]= matrix[i][j]*0.1 +cmes.matrix[i][j]*0.9;
        }
      }
      cmes.position_center[0] = x*0.1 + cmes.position_center[0]*0.9;
      cmes.position_center[1] = y*0.1 + cmes.position_center[1]*0.9;
      cmes.position_center[2] = z*0.1 + cmes.position_center[2]*0.9;
    }
  }

  // std::cout << "Matrix: " << std::endl;
  // for (int i = 0; i < 4; ++i) {
  //     for (int j = 0; j < 4; ++j) {
  //         std::cout << matrix[i][j] << " ";
  //     }
  //     std::cout << std::endl;
  // }
  // std::cout<<x<<" "<<y<<" "<<z<<std::endl;
  }

  R1.setRPY(M_PI, -M_PI/2, 0);
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          R2[i][j] = cmes.matrix[i][j];
          ROS_INFO("R2[%d][%d]:%f \n",i,j,R2[i][j]);
      }
  }
  // R2.setRPY(0, 0/*-M_PI/6*/, 0);
  block_pp.R_final = R1 * R2;

  block_pp.distance = 0.4;
  block_pp.position_x = cmes.position_center[2]/1000.0-0.220+0.440-0.2;
  block_pp.position_y = -cmes.position_center[1]/1000.0-0.017;
  block_pp.position_z = cmes.position_center[0]/1000.0+0.34;
  block_pp.position_x_near = cmes.matrix[2][3]/1000.0-0.220+0.440-0.2;
  block_pp.position_y_near = -cmes.matrix[1][3]/1000.0-0.017;
  block_pp.position_z_near = cmes.matrix[0][3]/1000.0+0.34;
  double temp = sqrt((block_pp.position_x - block_pp.position_x_near)*(block_pp.position_x - block_pp.position_x_near) + 
        (block_pp.position_y - block_pp.position_y_near)*(block_pp.position_y - block_pp.position_y_near) + 
        (block_pp.position_z - block_pp.position_z_near)*(block_pp.position_z - block_pp.position_z_near));
  arm_pp.R_final = block_pp.R_final;
  arm_pp.q = block_pp.q;
  arm_pp.position_x = block_pp.position_x + (block_pp.position_x_near - block_pp.position_x) / temp * block_pp.distance;
  arm_pp.position_y = block_pp.position_y + (block_pp.position_y_near - block_pp.position_y) / temp * block_pp.distance;
  arm_pp.position_z = block_pp.position_z + (block_pp.position_z_near - block_pp.position_z) / temp * block_pp.distance;
  ROS_INFO("temp:%f \n",temp);
  ROS_INFO("block_pp.position_x:%f \n",block_pp.position_x);
  ROS_INFO("block_pp.position_y:%f \n",block_pp.position_y);
  ROS_INFO("block_pp.position_z:%f \n",block_pp.position_z);
  ROS_INFO("block_pp.position_x_near:%f \n",block_pp.position_x_near);
  ROS_INFO("block_pp.position_y_near:%f \n",block_pp.position_y_near);
  ROS_INFO("block_pp.position_z_near:%f \n",block_pp.position_z_near);
  ROS_INFO("arm_pp.position_x:%f \n",arm_pp.position_x);
  ROS_INFO("arm_pp.position_y:%f \n",arm_pp.position_y);
  ROS_INFO("arm_pp.position_z:%f \n",arm_pp.position_z);

  // 设置兑换站的形状和位姿
  moveit_msgs::CollisionObject box_object;
  box_object.header.frame_id = "base_link";
  box_object.id = "box";
  shape_msgs::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[0] = block_pp.dimension;
  box_primitive.dimensions[1] = block_pp.dimension;
  box_primitive.dimensions[2] = block_pp.dimension;
  geometry_msgs::Pose box_pose;
  box_pose.position.x = block_pp.position_x;
  box_pose.position.y = block_pp.position_y;
  box_pose.position.z = block_pp.position_z;
  box_pose.orientation.x = block_pp.q.x();
  box_pose.orientation.y = block_pp.q.y();
  box_pose.orientation.z = block_pp.q.z();
  box_pose.orientation.w = block_pp.q.w();
  box_object.primitives.push_back(box_primitive);
  box_object.primitive_poses.push_back(box_pose);
  box_object.operation = box_object.ADD;

  // 将兑换站添加到collision_objects
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(box_object);

  // 将兑换站添加到规划场景中
  scene.addCollisionObjects(collision_objects);
  scene.applyCollisionObjects(collision_objects);

  // 判断兑换站是否已被添加到规划场景中
  if(waitForStateUpdate(box_object.id,scene,true)){
    ROS_INFO("The Box has been successfully added.");
  }
  else{
    ROS_INFO("Failed to add the Box.");
  }

  // 设置规划组的目标位姿，让机械臂运动到该目标
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = arm_pp.position_x;
  target_pose.pose.position.y = arm_pp.position_y;
  target_pose.pose.position.z = arm_pp.position_z;
  target_pose.pose.orientation.x = arm_pp.q.x();
  target_pose.pose.orientation.y = arm_pp.q.y();
  target_pose.pose.orientation.z = arm_pp.q.z();
  target_pose.pose.orientation.w = arm_pp.q.w();
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);
  arm.move();

  // // 删除规划场景里的物体对象
  // std::vector<std::string> object_ids;
  // object_ids.push_back(box_object.id);
  // scene.removeCollisionObjects(object_ids);

  //5s延时
  sleep(5);

  // //机械臂回到初始home位置
  // ROS_INFO("Moving to pose: home");
  // arm.setNamedTarget("home");
  // arm.move();

  //结束节点运行
  ros::shutdown();
  return 0;
}
