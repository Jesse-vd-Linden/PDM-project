import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import lxml.etree as etree 
import yaml 

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_name = 'robot_world'
    world_file_name = 'world.sdf'
    
    world = os.path.join('/home/jesse/Documents/pdm/catkin_ws/src/robot_world/', 'environment',world_file_name)
    sdf = os.path.join('/home/jesse/Documents/pdm/catkin_ws/src/robot_world/', 'model','servicebot','model.sdf')
    urdf = os.path.join('/home/jesse/Documents/pdm/catkin_ws/src/robot_world/', 'model','servicebot','model.urdf')
    print("world_file_name : {}".format(world))
    print("sdf_file_name : {}".format(sdf))

    robot_file = etree.parse(urdf)
    robot_desc = etree.tostring(robot_file).decode()


    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo','--verbose',world,'-s','libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2','run','gazebo_ros','spawn_entity.py','-entity','our_robot','-x','0','-y','0','-z','0','-R','0','-P','0','-Y','0','-file',sdf],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2','control','load_controller','--set-state','start','joint_state_broadcaster'], #ros2 control load_controller --set-state start joint_state_broadcaster
            output='screen'),
        ExecuteProcess(
            cmd=['ros2','control','load_controller','--set-state','start','joint_trajectory_controller'], #ros2 control load_controller --set-state start joint_trajectory_controller
            output='screen'),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{"robot_description": robot_desc}],
        #     output='screen'),
    ])