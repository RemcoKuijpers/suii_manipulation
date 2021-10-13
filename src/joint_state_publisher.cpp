#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Rate r(100);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("joint_state_publisher");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.178.71");

    while(rclcpp::ok()){
        std::vector<double> joint_positions = rtde_receive.getActualQ();
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = rclcpp::Clock().now();
        msg.name = {"ur3/shoulder_pan_joint", "ur3/shoulder_lift_joint", "ur3/elbow_joint", "ur3/wrist_1_joint", "ur3/wrist_2_joint", "ur3/wrist_3_joint"};
        msg.position = joint_positions;
        publisher->publish(msg);
        rclcpp::spin_some(node);
        r.sleep();
    }
    rtde_receive.disconnect();
    rclcpp::shutdown();
    return 0;
}