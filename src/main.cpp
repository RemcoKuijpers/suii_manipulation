#include "rclcpp/rclcpp.hpp"
#include "config.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>

// Define states
typedef enum {
    wait_for_service_call = 1,
    move,
    pick,
    place,
    done
} state_t;


int main(int argc, char **argv)
{
    // Initialize ROS and create node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("Manipulation", node_options);
    rclcpp::Rate r(EXECUTION_RATE);
    static const std::string PLANNING_GROUP = "ur3/suii_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Initialize state machine
    state_t state = move;

    while(rclcpp::ok()){

        switch(state) {

            case wait_for_service_call:
            {
                break;
            }

            case move:
            {
                geometry_msgs::msg::Pose target_pose1;
                target_pose1.orientation.w = 1.0;
                target_pose1.position.x = 0.28;
                target_pose1.position.y = 0.3;
                target_pose1.position.z = 0.3;
                move_group.setPoseTarget(target_pose1);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                move_group.plan(my_plan);
                move_group.move();
                state = pick;
                break;
            }

            case pick:
            {
                geometry_msgs::msg::Pose target_pose2;
                target_pose2.orientation.w = 1.0;
                target_pose2.position.x = 0.28;
                target_pose2.position.y = 0.3;
                target_pose2.position.z = 0.4;
                move_group.setPoseTarget(target_pose2);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
                move_group.plan(my_plan2);
                move_group.move();
                state = move;
                break;
            }

            case place:
            {
                break;
            }

            case done:
            {
                break;
            }
        }

        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}