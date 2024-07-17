# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("taubot_description"), "urdf", "taubot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("taubot_bringup"),
            "config",
            "taubot_controllers.yaml",
        ]
    )
#    rviz_config_file = PathJoinSubstitution(
#        [FindPackageShare("ros2_control_demo_example_2"), "rviz", "diffbot.rviz"]
#    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
 #   rviz_node = Node(
 #       package="rviz2",
 #       executable="rviz2",
 #       name="rviz2",
 #       output="log",
 #       arguments=["-d", rviz_config_file],
 #   )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
  #  delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
  #      event_handler=OnProcessExit(
  #          target_action=joint_state_broadcaster_spawner,
  #          on_exit=[rviz_node],
  #      )
  #  )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # lidar
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        output='screen',
        name='sllidar_node',
        parameters=[{
            'serial_port':'/dev/ttyUSB0',
            'serial_baudrate':460800,
            'frame_id':'laser',
            'angle_compensate':True,
            'inverted':False,
            'scan_mode':'Standard',
        }],
    ) 

    # Topic relay needed for movement
    topic_relay_node = Node(
        package='topic_relay',
        executable='topic_relay_node',
        name='topic_relay_node',
    )

    # IMU data
    imu_node = Node(
        package='mpu6050',
        executable='mpu6050_node',
        name='mpu6050_node',
    )

    # STM32 reset
    stm32_reset_node = Node(
        package='stm32_reset',
        executable='stm32_reset_node',
        name='stm32_reset_node',
    )

    nodes = [
        stm32_reset_node,
        control_node,
        robot_state_pub_node,
#        control_node,
        joint_state_broadcaster_spawner,
#        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        lidar_node,
        topic_relay_node,
        imu_node,
    ]

    
    return LaunchDescription(nodes)
