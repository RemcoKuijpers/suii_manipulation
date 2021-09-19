#define EXECUTION_RATE 10 // [Hz]
#define PREPICK_HEIGHT 0.1 // [m]
#define GRIPPER_CLOSING_TIME 400 //[ms]

const double JOINT_SPACE_SPEED =  1.05; //[rad/s]
const double  JOINT_SPACE_ACCELERATION = 1.4; //[rad/s^2]

// Define fixed arm poses
const std::vector<double> drive_pose = {-1.5726707617389124, -0.2505281607257288, -2.4049914518939417,
  -1.4284794966327112, 1.645344614982605, 0.002140682190656662};

const std::vector<double> safety_pose = {0.2641944885253906, -1.149320427571432, -1.4262059370623987,
  -1.968114201222555, 1.5591015815734863, 0.29005083441734314};

const std::vector<double> q_near = {-M_PI_2, -1.919, -M_PI_2, -1.2217, M_PI_2, 0};

const std::vector<std::string> containers = {"container_1", "container_2", "container_3"};
const double ZERO_TRANSLATION[3] = {0.0, 0.0, 0.0};
const double ZERO_ROTATION[3] = {0.0, 0.0, 0.0};