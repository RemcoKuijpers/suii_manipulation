#ifndef robotControl_H
#define robotControl_H

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <thread>

class RobotControl
{
    ur_rtde::RTDEControlInterface robot;
    ur_rtde::RTDEIOInterface io;

public:
    RobotControl(std::string ip_address) : robot{ip_address}, io{ip_address} { }
    bool isRobotConnected();
    bool moveL(const std::vector<double> &pose, double speed = 0.25, double acceleration = 1.2, bool async = false);
    bool moveJ(const std::vector<double> &q, double speed = 1.05, double acceleration = 1.4, bool async = false);
    void openGripper();
    void closeGripper();
};

#endif //driveControl_H
