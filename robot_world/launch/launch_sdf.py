import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import lxml.etree as etree

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = 'robot_world'
    world_file_name = 'world'

    world = os.path.join(get_package_share_directory(robot_name), 'environment', world_file_name)

    urdf = os.path.join(get_package_share_directory(robot_name), 'model', 'model.sdf')

    xml_object = etree.parse(urdf)
    xml_string = etree.tostring(xml_object).decode()
    # xml = open(urdf, 'r').read()
    #
    # xml = xml.replace('"', '\\"')

    swpan_args = '{name: \"my_robot\", xml: \"' + xml_string + '\" }'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'TheArm',
                 '-x','10',
                 '-y','10',
                 '-z','10',
                 '-R','0.1',
                 '-P','0.2',
                 '-Y','0.3',
                 '-database','simple_arm'],
            output='screen'),
    ])