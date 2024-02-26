import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
 
def generate_launch_description():
 
  # Constants for paths to different files and folders
  gazebo_models_path = 'src/models'
  package_name = 'clyde_description'
  robot_name_in_model = 'clyde'
  urdf_model_path = 'src/urdf/clyde_simple.urdf'
  world_file_path = 'src/worlds/new-env0'
  rviz_config_path = 'rviz/urdf_config.rviz'
     
  # Pose where we want to spawn the rob`ot
  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.0'
 
  ############ You do not need to change anything below this line #############
   
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  world_path = os.path.join(pkg_share, world_file_path)
  gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  urdf_model_path = os.path.join(pkg_share, urdf_model_path)
  rviz_config_path = os.path.join(pkg_share, rviz_config_path)

   
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  namespace = LaunchConfiguration('namespace')
  urdf_model = LaunchConfiguration('urdf_model')
  use_namespace = LaunchConfiguration('use_namespace')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
   
  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace'
  )
 
  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack'
  )
             
  declare_sdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=urdf_model_path, 
    description='Absolute path to robot sdf file'
  )
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient'
  )
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true'
  )
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator'
  )
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load'
  )
  
  declare_jsp_gui_cmd = DeclareLaunchArgument(
    name='gui', 
    default_value='True',
    description='Flag to enable joint_state_publisher_gui'
  )

  declare_rviz_config_cmd = DeclareLaunchArgument(
    name='rvizconfig', 
    default_value=rviz_config_path, 
    description='Absolute path to rviz config file'
  )
  
  robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])}]
    )
  joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
  joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
  rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
   
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 
  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
               '-file', urdf_model_path,
                  '-x', spawn_x_val,
                  '-y', spawn_y_val,
                  '-z', spawn_z_val,
                  '-Y', spawn_yaw_val],
                  output='screen')
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_sdf_model_path_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_rviz_config_cmd)
  ld.add_action(declare_jsp_gui_cmd)
 
  # Add any actions
  ld.add_action(joint_state_publisher_node)
  ld.add_action(joint_state_publisher_gui_node)
  ld.add_action(robot_state_publisher_node)
  ld.add_action(rviz_node)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_cmd)
 
  return ld