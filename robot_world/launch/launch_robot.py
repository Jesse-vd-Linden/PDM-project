import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import lxml.etree as etree 
import yaml 

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    sdf_file_name = '$(find robot_world)/model/model.sdf'

    print("sdf_file_name : {}".format(sdf_file_name))
    sdf = os.path.join(sdf_file_name)
    
    xml_object = etree.parse('/home/jesse/Documents/pdm/catkin_ws/src/robot_world/model/model.sdf')
    xml_string = etree.tostring(xml_object).decode()
    robot_description = DeclareLaunchArgument(
        "robot_description", default_value=xml_string)

    yml_string = yaml.load('/home/jesse/Documents/pdm/catkin_ws/src/robot_world/config/config.yml')
    robot_description = DeclareLaunchArgument(
        "our_robot", default_value=yml_string)

    # rosparam_controller = DeclareLaunchArgument()

    return LaunchDescription([
        robot_description,

        DeclareLaunchArgument(
            'robot_world',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            name="urdf_spawner",
            package="gazebo_ros",
            executable="spawn_model",
            respawn="false",
            output="screen", 
            arguments="-urdf -model our_robot -param robot_description -y -6"
        ),
        Node(
            name="controller_spawner",
            package="controller_manager",
            executable="spawner", 
            namespace="our_robot",
            arguments="left_wheel_position_controller right_wheel_position_controller lower_arm_position_controller upper_arm_position_controller joint_state_controller --shutdown-timeout 3"
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[sdf],
            remappings=[
                ('/joint_states', '/our_robot/joint_states'),
            ]
        )
    ])