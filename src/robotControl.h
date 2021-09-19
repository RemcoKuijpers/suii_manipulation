#ifndef robotControl_H
#define robotControl_H

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include "objectHandler.h"
#include "config.h"
#include <thread>
#include <math.h>
#include <vector>

class RobotControl
{
    ur_rtde::RTDEControlInterface robot;
    ur_rtde::RTDEIOInterface io;

public:
    RobotControl(std::string ip_address) : robot{ip_address}, io{ip_address} {
        if(this->isRobotConnected()){
            RCLCPP_INFO(this->object_handler.get_logger(), "Connected to UR3 with ip-address '%s'", ip_address.c_str());
        }else{
            RCLCPP_INFO(this->object_handler.get_logger(), "Could not connect to UR3 at '%s'", ip_address.c_str());
        }
    }
    ObjectHandler object_handler;
    bool isRobotConnected();
    void disconnectRobot();
    bool moveL(const std::vector<double> &pose, double speed = 0.25, double acceleration = 1.2, bool async = false);
    bool moveJ(const std::vector<double> &q, double speed = JOINT_SPACE_SPEED, double acceleration = JOINT_SPACE_ACCELERATION, bool async = false);
    bool moveJ_IK(const std::vector<double> &pose, double speed = JOINT_SPACE_SPEED, double acceleration = JOINT_SPACE_ACCELERATION, bool async = false);
    bool moveJ_Path(const std::vector<std::vector<double>> &path, bool async = false); // {x, y, z}
    bool pickObject(const std::string frame_name);
    bool placeObject(const std::string object_frame, const std::string place_frame);
    bool moveObject(const std::string object_frame, const std::string target_frame);
    std::vector<double> getRotation(const double roll, const double pitch, const double yaw); // Get UR rotation angles from euler angles
    void openGripper();
    void closeGripper();
};

#endif //driveControl_H
