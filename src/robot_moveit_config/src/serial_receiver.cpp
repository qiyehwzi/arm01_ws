#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/JointState.h>

#pragma pack(1)
typedef struct {
  uint8_t sof;
  float tra[6];
  uint8_t tailer;
} receive_msg_t;
#pragma pack()

serial::Serial sp;
receive_msg_t receive_msg;

void serial_init() {
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_receiver");
    ros::NodeHandle nh;
    
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    sensor_msgs::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    joint_state.position.resize(6);
    
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
        
        // Read from serial
        if(sp.available()) {
            uint8_t byte;
            while(sp.read(&byte, 1) == 1) {
                if(byte == 0x69) { // Start byte
                    uint8_t buffer[sizeof(receive_msg)];
                    buffer[0] = byte;
                    size_t n = sp.read(buffer+1, sizeof(receive_msg)-1);
                    
                    if(n == sizeof(receive_msg)-1 && buffer[sizeof(receive_msg)-1] == 0x65) {
                        memcpy(&receive_msg, buffer, sizeof(receive_msg));
                        
                        // Update joint state
                        for(int i=0; i<6; i++) {
                            joint_state.position[i] = receive_msg.tra[i];
                        }
                        joint_state.header.stamp = ros::Time::now();
                        joint_pub.publish(joint_state);
                    }
                }
            }
        }
        rate.sleep();
    }
    
    sp.close();
    return 0;
}
