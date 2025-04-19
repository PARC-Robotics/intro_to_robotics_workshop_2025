import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# PARC robot spawn pose
spawn_x_val = "-0.01"
spawn_y_val = "-3.14"
spawn_z_val = "0.31"
spawn_yaw_val = "1.57"

def generate_launch_description():

    # Set the path to different files and folders
    pkg_path = FindPackageShare(package="intro_to_robotics_workshop-2025").find("intro_to_robotics_workshop_2025")
    pkg_bringup = FindPackageShare(package="parc_robot_bringup").find(
        "parc_robot_bringup"
    )
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find(
        "ros_gz_sim"
    )

    bridge_params = os.path.join(pkg_bringup, "config/gz_bridge.yaml")
    rviz_config_file = os.path.join(pkg_path, "rviz/workshop.rviz")
    world_filename = "workshop_world.sdf"
    world_path = os.path.join(pkg_path, "worlds", world_filename)
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_bringup, "models")
    )

    # Launch configuration variables
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model to load",
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Launch Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
        )
    
    # Spawn PARC robot in Gazebo
    spawn_parc_robot = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic",
                "robot_description",
                "-name",
                "parc_robot",
                "-x",
                spawn_x_val,
                "-y",
                spawn_y_val,
                "-z",
                spawn_z_val,
                "-Y",
                spawn_yaw_val,
            ],
        )

    # Start Gazebo ROS bridge
    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # Start Gazebo ROS Left Image bridge
    start_gazebo_ros_left_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/left_camera/image_raw"],
        parameters=[
            {
                # 'use_sim_time': LaunchConfiguration('use_sim_time'),
                'left_camera.image_raw.compressed.jpeg_quality': 75},],
        output="screen",
    )

    # Start Gazebo ROS Left Image bridge
    start_gazebo_ros_right_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/right_camera/image_raw"],
        output="screen",
    )
    
    # Start Gazebo ROS ZED2 Center Image bridge
    start_gazebo_ros_zed2_center_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/zed2_camera_center/image_raw"],
        output="screen",
    )
     
    # Start Gazebo ROS ZED2 Left Raw Image bridge
    start_gazebo_ros_zed2_left_raw_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/zed2_left_raw/image_raw"],
        output="screen",
    )
     
    # Start Gazebo ROS ZED2 Right Raw Image bridge
    start_gazebo_ros_zed2_right_raw_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/zed2_right_raw/image_raw"],
        output="screen",
    )

    # Start Gazebo ROS ZED2 Right Rectified Image bridge
    start_gazebo_ros_zed2_right_rect_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/zed2_right/image_rect"],
        output="screen",
    )

    # Start Gazebo ROS ZED2 Depth Image bridge
    start_gazebo_ros_zed2_depth_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/zed2_depth_camera/depth_image"],
        output="screen",
    )
    
    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        # arguments=['camera/camera_info', 'camera/image/camera_info'],
        arguments=['left_camera/camera_info', 'left_camera/image_raw/camera_info'],
        # parameters=[
        #     {'use_sim_time': LaunchConfiguration('use_sim_time')},
        # ]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(set_env_vars_resources)

    # Add any actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_parc_robot)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_left_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_right_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_zed2_center_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_zed2_left_raw_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_zed2_right_raw_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_zed2_right_rect_image_bridge_cmd)
    ld.add_action(start_gazebo_ros_zed2_depth_image_bridge_cmd)
    ld.add_action(relay_camera_info_node)

    return ld