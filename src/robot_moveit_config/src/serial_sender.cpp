#include <ros/ros.h>
#include <serial/serial.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <cstring>

#define LEN_TX_PACKET 29

#pragma pack(1)
typedef struct {
  uint8_t sof;
  float tra[6];
  uint8_t tailer;
} send_msg_t;
#pragma pack()

serial::Serial sp;
send_msg_t send_msg;
uint8_t PC_SEND_BUF[LEN_TX_PACKET+1];
moveit_msgs::RobotTrajectory moveit_tra;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* server) {
    moveit_tra.joint_trajectory = goalPtr->trajectory;
    ROS_INFO("Received trajectory with %d points", (int)moveit_tra.joint_trajectory.points.size());
    server->setSucceeded();
}

void serial_init() {
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_sender");
    ros::NodeHandle nh;
    
    // Action server
    Server server(nh, "manipulator_controller/follow_joint_trajectory", 
                 boost::bind(&execute_callback, _1, &server), false);
    server.start();

    // Serial init
    serial_init();
    try {
        sp.open();
        ROS_INFO("Serial port opened");
    } catch(serial::IOException& e) {
        ROS_ERROR("Failed to open serial port");
        return -1;
    }

    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        
        if (!moveit_tra.joint_trajectory.points.empty()) {
            for(size_t i=0; i<moveit_tra.joint_trajectory.points.size(); i+=5) {
                send_msg.sof = 0x69;
                for(int j=0; j<6; j++) {
                    send_msg.tra[j] = moveit_tra.joint_trajectory.points[i].positions[j];
                }
                send_msg.tailer = 0x65;
                
                memcpy(PC_SEND_BUF, &send_msg, sizeof(send_msg));
                PC_SEND_BUF[LEN_TX_PACKET] = '\n';
                sp.write(PC_SEND_BUF, sizeof(PC_SEND_BUF));
                
                ros::Duration(0.02).sleep();
            }
            moveit_tra.joint_trajectory.points.clear(); // Clear after sending
        }
        rate.sleep();
    }
    
    sp.close();
    return 0;
}

