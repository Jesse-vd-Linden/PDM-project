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

    sdf = os.path.join(get_package_share_directory(robot_name), 'model', 'model2.sdf')
    # print(sdf)
    # xml_object = etree.parse(sdf)
    # xml_string = etree.tostring(xml_object).decode()
    # print(xml_string)
    # # xml = open(urdf, 'r').read()
    #
    # xml = xml.replace('"', '\\"')

    # swpan_args = '{name: \"my_robot\", xml: \"' + xml_string + '\" }'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # ExecuteProcess(
        #     cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        #     output='screen'),

        ExecuteProcess(
            cmd=['ros2','run','gazebo_ros','spawn_entity.py','-entity','our_robot','-x','0','-y','0','-z','0','-file',sdf],
            output='screen'),
    ])