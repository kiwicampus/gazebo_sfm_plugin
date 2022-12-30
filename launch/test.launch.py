from os import environ
from os import pathsep
from scripts import GazeboRosPaths
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('gazebo_sfm_plugin'),
        'models',
    ])
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_gazebo_sfm_plugin = get_package_share_directory("gazebo_sfm_plugin")
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        )
    )


    # src_gazebo_cmd = [
    #     'source ',
    #     '/usr/share/gazebo/setup.sh'
    # ]
    # ExecuteProcess(
    #     cmd=src_gazebo_cmd,
    #     #output='screen',
    #     shell=True
    # )
    # print('gazebo_source:', src_gazebo_cmd)

    
    model, plugin, media = GazeboRosPaths.get_paths()
    #print('model:', model)

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }
    print('env:', env)

#    
    # world_path = PathJoinSubstitution([
    #     FindPackageShare('gazebo_sfm_plugin'),
    #     'worlds',
    #     '3_LMU.world'
    # ])


    # gzserver_cmd = [
    #     'gzserver ',
    #     #'-u ', #to start paused
    #     # Pass through arguments to gzserver
    #     LaunchConfiguration('world'), world_path, 
    #     _boolean_command('verbose'), '',
    # ]

    # gzclient_cmd = [
    #     'gzclient',
    #     _boolean_command('verbose'), ' ',
    # ]
#
    


    return LaunchDescription([

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH', 
            value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), my_gazebo_models]
        ),
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH', 
            value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), my_gazebo_models]
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH', 
            value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), plugin]
        ),

        DeclareLaunchArgument(
                "world",
                default_value=[
                    os.path.join(pkg_gazebo_sfm_plugin, "worlds", "cafe3.world"),
                    "",
                ],
                description="SDF world file",
        ),
#
        # DeclareLaunchArgument(
        #     'world', default_value='',
        #              #'/home/kenny/ros2_ws/src/human_nav_gazebo_plugin/worlds/cafe2.world',
        #     description='Specify world file name'
        # ),
#

# if true, reports same problems as in kiwi.launch.py

        # DeclareLaunchArgument(
        #     'verbose', default_value='true',
        #     description='Set "true" to increase messages written to terminal.'
        # ),

        gazebo,

#
        # ExecuteProcess(
        #     cmd=gzserver_cmd,
        #     output='screen',
        #     #additional_env=env,
        #     shell=True,
        #     on_exit=Shutdown(),
        #     #condition=IfCondition(LaunchConfiguration('server_required')),
        # ),  

        # ExecuteProcess(
        #     cmd=gzclient_cmd,
        #     output='screen',
        #     #additional_env=env,
        #     shell=True,
        #     on_exit=Shutdown(),
        #     #condition=IfCondition(LaunchConfiguration('server_required')),
        # ),
    ])


# # Add boolean commands if true
# def _boolean_command(arg):
#     cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
#     py_cmd = PythonExpression(cmd)
#     return py_cmd


#