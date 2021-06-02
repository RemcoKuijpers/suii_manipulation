#include "robotControl.h"
#include "config.h"

bool RobotControl::isRobotConnected(){
    return robot.isConnected();
}

bool RobotControl::moveL(const std::vector<double> &pose, double speed, double acceleration, bool async){
    return robot.moveL(pose, speed, acceleration, async);
}

bool RobotControl::moveJ(const std::vector<double> &q, double speed, double acceleration, bool async){
    return robot.moveJ(q, speed, acceleration, async);
}

void RobotControl::openGripper(){
    io.setAnalogOutputVoltage(0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void RobotControl::closeGripper(){
    io.setAnalogOutputVoltage(0, 0.45);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}