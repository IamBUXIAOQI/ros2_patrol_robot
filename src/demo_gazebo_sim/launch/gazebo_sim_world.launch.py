import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo

def generate_launch_description():

    this_pkg = get_package_share_directory('demo_gazebo_sim')
    mycar_desc_pkg = get_package_share_directory("demo_gazebo_sim")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(this_pkg,"world","house.sdf")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            # "gz_args": "-r " + world_file + " -v4"
            "gz_args": "-r " + world_file
        }.items(),
    )
    mycar_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mycar_desc_pkg,"launch","mycar_desc_sim.launch.py")
        )
    )
    spawn = Node(package="ros_gz_sim", executable="create",
                arguments=[
                "-name", "mycar",
                "-x", "0",
                "-z", "0.01", #设置为0,可能会陷进地里
                "-y", "0",
                "-topic", "/robot_description"],
            output="screen")
    
    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Stamped@gz.msgs.Twist",
                   "/model/mycar/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                   "/model/mycar/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                   "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/world/empty/model/mycar/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
                   "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                   "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                   "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
                   "/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                   "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                   "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
                   ],
        parameters=[{"qos_overrides./model/mycar.subscriber.reliability": "reliable"}],
        remappings=[
                ("/model/mycar/tf", "/tf"),
                ("/world/empty/model/mycar/joint_state","joint_states"),
                ("/model/mycar/odometry","/odom"),
            ],
        output="screen"
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    # action_load_effort_controller = launch.actions.ExecuteProcess(
    #     cmd='ros2 control load_controller fishbot_effort_controller --set-state active'.split(' '),
    #     output='screen'
    # )

    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        mycar_desc,
        spawn,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=action_load_joint_state_controller,
        #         on_exit=[action_load_effort_controller],
        #     )
        # ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller],
            )
        ),
        bridge
    ])
