from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_share = get_package_share_directory('gp7_description')
    simulation_share = get_package_share_directory('gp7_simulation')
    gp7_mesh_parent = os.path.dirname(get_package_share_directory('yaskawa_gp7_support'))
    controllers_file = os.path.join(simulation_share, 'config', 'gp7_controllers.yaml')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description_share, 'urdf', 'gp7.urdf.xacro'), ' controllers_file:=', controllers_file, ' enable_attachment:=', LaunchConfiguration('pick_place')]),
        value_type=str,
    )
    # Run physics so controller activation can complete. gz_ros2_control holds
    # unclaimed joints; the simulation torque limits now allow it to do so.
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')), launch_arguments={'gz_args': [PythonExpression(["'-r -v 3 ' if '", LaunchConfiguration('gui'), "' == 'true' else '-s -r -v 3 '"]), os.path.join(simulation_share, 'worlds', 'pick_place.sdf')]}.items())
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/box_b/attach@std_msgs/msg/Empty]gz.msgs.Empty', '/box_b/detach@std_msgs/msg/Empty]gz.msgs.Empty'], output='screen')
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    # Pass XML directly to the create node's parameter, avoiding discovery and
    # transient-local QoS dependencies on robot_state_publisher's topic.
    spawn_robot = Node(package='ros_gz_sim', executable='create', arguments=['-param', 'robot_description', '-name', 'gp7', '-x', '0', '-y', '0', '-z', '0'], parameters=[{'robot_description': robot_description}], output='screen')
    controllers = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30', '--activate-as-group'],
        output='screen',
    )
    def after_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='GP7 spawn failed'))]
        return [controllers]

    def after_controllers(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='GP7 controller activation failed'))]
        return [Node(
            package='gp7_task', executable='pick_place', output='screen',
            condition=IfCondition(LaunchConfiguration('pick_place')),
        ), Node(
            package='gp7_task', executable='joint_motion_demo', output='screen',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('demo'), "' == 'true' and '", LaunchConfiguration('pick_place'), "' != 'true'"])),
        )]

    mesh_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(filter(None, [gp7_mesh_parent, os.path.dirname(description_share), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])),
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'), mesh_resources,
        DeclareLaunchArgument('demo', default_value='false'),
        DeclareLaunchArgument('pick_place', default_value='false'),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=os.pathsep.join(filter(None, [os.path.join(os.path.dirname(os.path.dirname(get_package_share_directory('gp7_grasp'))), 'lib'), os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))),
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_controllers)),
        gazebo, clock_bridge, state_publisher, spawn_robot,
    ])
