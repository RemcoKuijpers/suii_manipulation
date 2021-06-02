#ifndef robotControl_H
#define robotControl_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>

class RobotControl
        {
private:
    moveit::planning_interface::MoveGroupInterface move_group("ur3/manipulator");
    
public:

};

#endif //driveControl_H
