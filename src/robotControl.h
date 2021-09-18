#ifndef robotControl_H
#define robotControl_H

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include "objectHandler.h"
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
    bool moveL(const std::vector<double> &pose, double speed = 0.25, double acceleration = 1.2, bool async = false);
    bool moveJ(const std::vector<double> &q, double speed = 1.05, double acceleration = 1.4, bool async = false);
    bool moveJ_IK(const std::vector<double> &pose, double speed = 1.05, double acceleration = 1.4, bool async = false);
    bool pickObject(const std::string frame_name);
    std::vector<double> getRotation(const double roll, const double pitch, const double yaw);
    void openGripper();
    void closeGripper();
};

#endif //driveControl_H
