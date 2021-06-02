import launch
from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import launch_ros
import launch.launch_description_sources
import os
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
  declared_arguments = []
  # UR specific arguments
  declared_arguments.append(
    DeclareLaunchArgument("ur_type", default_value="ur3",description="Type/series of used UR robot.")
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "robot_ip",default_value="192.168.178.71", description="IP address by which the robot can be reached."
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "safety_limits",
      default_value="true",
      description="Enables the safety limits controller if true.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "safety_pos_margin",
      default_value="0.15",
      description="The margin to lower and upper limits in the safety controller.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "safety_k_position",
      default_value="20",
      description="k-position factor in the safety controller.",
    )
  )
  # General arguments
  declared_arguments.append(
    DeclareLaunchArgument(
      "runtime_config_package",
      default_value="ur_bringup",
      description='Package with the controller\'s configuration in "config" folder. \
      Usually the argument is not set, it enables use of a custom setup.',
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "controllers_file",
      default_value="ur_controllers.yaml",
      description="YAML file with the controllers configuration.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "description_package",
      default_value="ur_description",
      description="Description package with robot URDF/XACRO files. Usually the argument \
      is not set, it enables use of a custom description.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "description_file",
      default_value="suii_description.urdf.xacro",
      description="URDF/XACRO description file with the robot.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "prefix",
      default_value='ur3/',
      description="Prefix of the joint names, useful for \
      multi-robot setup. If changed than also joint names in the controllers' configuration \
      have to be updated.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "use_fake_hardware",
      default_value="false",
      description="Start robot with fake hardware mirroring command to its states.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "fake_sensor_commands",
      default_value="false",
      description="Enable fake command interfaces for sensors used for simple simulations. \
      Used only if 'use_fake_hardware' parameter is true.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument(
      "robot_controller",
      default_value="joint_trajectory_controller",
      description="Robot controller to start.",
    )
  )
  declared_arguments.append(
    DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
  )

  # Initialize Arguments
  ur_type = "ur3"
  robot_ip = LaunchConfiguration("robot_ip")
  safety_limits = "true"
  safety_pos_margin = "0.15"
  safety_k_position = "20"
  # General arguments
  runtime_config_package = "ur_bringup"
  controllers_file = "ur_controllers.yaml"
  description_package = "ur_description"
  description_file = "suii_description.urdf.xacro"
  prefix = "ur3/"
  use_fake_hardware = LaunchConfiguration("use_fake_hardware")
  fake_sensor_commands = "false"
  robot_controller = "joint_trajectory_controller"
  launch_rviz = "false"

  joint_limit_params = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "config", "joint_limits.yaml"]
  )
  kinematics_params = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "config", "default_kinematics.yaml"]
  )
  physical_params = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "config", "physical_parameters.yaml"]
  )
  visual_params = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "config", "visual_parameters.yaml"]
  )
  script_filename = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "resources", "ros_control.urscript"]
  )
  input_recipe_filename = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "resources", "rtde_input_recipe.txt"]
  )
  output_recipe_filename = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "resources", "rtde_output_recipe.txt"]
  )

  robot_description_content = Command(
      [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("suii_manipulation"), "urdf", description_file]),
        " ",
        "robot_ip:=",
        robot_ip,
        " ",
        "joint_limit_params:=",
        joint_limit_params,
        " ",
        "kinematics_params:=",
        kinematics_params,
        " ",
        "physical_params:=",
        physical_params,
        " ",
        "visual_params:=",
        visual_params,
        " ",
        "safety_limits:=",
        safety_limits,
        " ",
        "safety_pos_margin:=",
        safety_pos_margin,
        " ",
        "safety_k_position:=",
        safety_k_position,
        " ",
        "name:=",
        ur_type,
        " ",
        "script_filename:=",
        script_filename,
        " ",
        "input_recipe_filename:=",
        input_recipe_filename,
        " ",
        "output_recipe_filename:=",
        output_recipe_filename,
        " ",
        "prefix:=",
        prefix,
        " ",
        "use_fake_hardware:=",
        use_fake_hardware,
        " ",
        "fake_sensor_commands:=",
        fake_sensor_commands,
        " "
      ]
    )

  robot_description = {"robot_description": robot_description_content}

  robot_controllers = PathJoinSubstitution(
      [FindPackageShare("suii_manipulation"), "config", controllers_file]
  )

  pkg_share = FindPackageShare(package='suii_manipulation').find('suii_manipulation')
  #robot_path = os.path.join(pkg_share, 'urdf/suii_description.urdf.xacro')
  #doc = xacro.process_file(robot_path)
  #robot_description = {'robot_description': doc.toprettyxml(indent='  ')}

  rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')

  control_node = launch_ros.actions.Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, robot_controllers],
    output={
        "stdout": "screen",
        "stderr": "screen",
    },
  )

  robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[robot_description]
  )

  joint_state_publisher_node = launch_ros.actions.Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
  )

  rviz_node = launch_ros.actions.Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_path]
  )

  joint_state_broadcaster_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner.py",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
  )

  robot_controller_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner.py",
    arguments=[robot_controller, "-c", "/controller_manager"],
  )

  io_and_status_controller_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner.py",
    arguments=["io_and_status_controller", "-c", "/controller_manager"],
  )
    
  speed_scaling_state_broadcaster_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner.py",
    arguments=[
      "speed_scaling_state_broadcaster",
      "--controller-manager",
      "/controller_manager",
    ],
  )

  force_torque_sensor_broadcaster_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner.py",
    arguments=[
      "force_torque_sensor_broadcaster",
      "--controller-manager",
      "/controller_manager",
    ],
  )

  nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        io_and_status_controller_spawner,
        speed_scaling_state_broadcaster_spawner,
        force_torque_sensor_broadcaster_spawner,
        robot_controller_spawner,
  ]
  
  return LaunchDescription(declared_arguments + nodes_to_start)