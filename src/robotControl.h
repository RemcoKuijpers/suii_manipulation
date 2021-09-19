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
    ObjectHandler object_handler;

public:
    RobotControl(std::string ip_address) : robot{ip_address}, io{ip_address} { }
    bool isRobotConnected();
    void disconnectRobot();
    bool moveL(const std::vector<double> &pose, double speed = 0.25, double acceleration = 1.2, bool async = false);
    bool moveJ(const std::vector<double> &q, double speed = JOINT_SPACE_SPEED, double acceleration = JOINT_SPACE_ACCELERATION, bool async = false);
    bool moveJ_IK(const std::vector<double> &pose, double speed = JOINT_SPACE_SPEED, double acceleration = JOINT_SPACE_ACCELERATION, bool async = false);
    bool moveJ_Path(const std::vector<std::vector<double>> &path, bool async = false); // {x, y, z}
    bool pickObject(const std::string frame_name);
    std::vector<double> getRotation(const double roll, const double pitch, const double yaw); // Get UR rotation angles from euler angles
    void openGripper();
    void closeGripper();
};

#endif //driveControl_H
