#include "robotControl.h"

bool RobotControl::isRobotConnected(){
    return robot.isConnected();
}

bool RobotControl::moveL(const std::vector<double> &pose, double speed, double acceleration, bool async){
    return robot.moveL(pose, speed, acceleration, async);
}

bool RobotControl::moveJ(const std::vector<double> &q, double speed, double acceleration, bool async){
    return robot.moveJ(q, speed, acceleration, async);
}

bool RobotControl::moveJ_IK(const std::vector<double> &pose, double speed, double acceleration, bool async){
    return robot.moveJ_IK(pose, speed, acceleration, async);
}

bool RobotControl::moveJ_Path(const std::vector<std::vector<double>> &path, bool async){
    return robot.moveJ(path, async);
}

void RobotControl::disconnectRobot(){
    robot.disconnect();
}

bool RobotControl::pickObject(const std::string frame_name){
    this->openGripper();
    double roll, pitch, yaw;
    std::vector<double> rotation;

    tf2::Transform base_t;
    tf2::Transform gripper_t;
    tf2::Transform transform;
    tf2::Transform prePick;
    tf2::Transform pre_t;
    tf2::Transform tool0_t;
    tf2::Quaternion q;
    tf2::Vector3 translation;
    tf2::Vector3 pre_translation;

    base_t = this->object_handler.getTransform("ur3/base", frame_name);
    gripper_t = this->object_handler.getTransform("gripper", "ur3/tool0");
    tool0_t = this->object_handler.getTransform("ur3/base", "ur3/tool0");
    prePick.setOrigin(tf2::Vector3(0, 0, -PREPICK_HEIGHT));

    transform.mult(base_t, gripper_t);
    pre_t.mult(transform, prePick);
    translation = transform.getOrigin();
    pre_translation = pre_t.getOrigin();

    q = transform.getRotation();
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);

    rotation = this->getRotation(round(100000*roll)/100000, round(100000*pitch)/100000, round(100000*(yaw))/100000);

    std::vector<std::vector<double>> path;

    if((tool0_t.getOrigin().getY() > 0 && base_t.getOrigin().getY() < 0) || (tool0_t.getOrigin().getY() < 0 && base_t.getOrigin().getY() > 0)){
        std::vector<double> path_pose1;
        for (int i = 0; i<(int)safety_pose.size(); i++){
            path_pose1.push_back(safety_pose[i]);
        }
        path_pose1.push_back(JOINT_SPACE_SPEED);
        path_pose1.push_back(JOINT_SPACE_ACCELERATION);
        path_pose1.push_back(0.15); //Blend radius
        std::vector<double> path_pose2 = robot.getInverseKinematics({pre_translation.getX(), pre_translation.getY(), pre_translation.getZ(), rotation[0], rotation[1], rotation[2]});
        path_pose2.push_back(JOINT_SPACE_SPEED);
        path_pose2.push_back(JOINT_SPACE_ACCELERATION);
        path_pose2.push_back(0.05); //Blend radius
        std::vector<double> path_pose3 = robot.getInverseKinematics({translation.getX(), translation.getY(), translation.getZ(), rotation[0], rotation[1], rotation[2]});
        path_pose3.push_back(JOINT_SPACE_SPEED);
        path_pose3.push_back(JOINT_SPACE_ACCELERATION);
        path_pose3.push_back(0.0); //Blend radius
        path.push_back(path_pose1);
        path.push_back(path_pose2);
        path.push_back(path_pose3);
    }else{
        std::vector<double> path_pose1 = robot.getInverseKinematics({pre_translation.getX(), pre_translation.getY(), pre_translation.getZ(), rotation[0], rotation[1], rotation[2]});
        path_pose1.push_back(JOINT_SPACE_SPEED);
        path_pose1.push_back(JOINT_SPACE_ACCELERATION);
        path_pose1.push_back(0.05); //Blend radius
        std::vector<double> path_pose2 = robot.getInverseKinematics({translation.getX(), translation.getY(), translation.getZ(), rotation[0], rotation[1], rotation[2]});
        path_pose2.push_back(JOINT_SPACE_SPEED);
        path_pose2.push_back(JOINT_SPACE_ACCELERATION);
        path_pose2.push_back(0.0); //Blend radius
        path.push_back(path_pose1);
        path.push_back(path_pose2);
    }


    if (this->moveJ_Path(path)){
        this->closeGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSING_TIME-100));
    }
    
    return this->moveL({pre_translation.getX(), pre_translation.getY(), pre_translation.getZ(), rotation[0], rotation[1], rotation[2]});
}

void RobotControl::openGripper(){
    io.setAnalogOutputVoltage(0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void RobotControl::closeGripper(){
    io.setAnalogOutputVoltage(0, 0.45);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

std::vector<double> RobotControl::getRotation(const double roll, const double pitch, const double yaw){
    std::vector<double> rot;

    double yawMatrix[3][3] = {
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}
    };

    double pitchMatrix[3][3] = {
        {cos(pitch), 0, sin(pitch)},
        {0, 1, 0},
        {-sin(pitch), 0, cos(pitch)}
    };

    double rollMatrix[3][3] = {
        {1, 0, 0},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll), cos(roll)}
    };

    double R1[3][3];
    double R[3][3];
    
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            R1[i][j] = 0;
            for (int u = 0; u < 3; u++){
                R1[i][j] += yawMatrix[i][u] * pitchMatrix[u][j];
            }
        }
    }
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            R[i][j] = 0;
            for (int u = 0; u < 3; u++){
                R[i][j] += R1[i][u] * rollMatrix[u][j];
            }
        }
    }

    double theta = acos(((R[0][0] + R[1][1] + R[2][2]) - 1) / 2);
    double multi = 1 / (2 * sin(theta));
    double rx = multi * (R[2][1] - R[1][2]) * theta;
    double ry = multi * (R[0][2] - R[2][0]) * theta;
    double rz = multi * (R[1][0] - R[0][1]) * theta;
    rot.push_back(rx);
    rot.push_back(ry);
    rot.push_back(rz);
    return rot;
}
