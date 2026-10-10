from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from mirobot_master_slave.coordinates import validate_launch_settings


def validate_before_start(context):
    if LaunchConfiguration('mode').perform(context) == 'vision':
        validate_launch_settings(LaunchConfiguration('params').perform(context))
    return []


def generate_launch_description():
    share = Path(get_package_share_directory('mirobot_description'))
    own = Path(get_package_share_directory('mirobot_master_slave'))
    description = ParameterValue(
        (share / 'urdf/mirobot_urdf_2.urdf').read_text(), value_type=str
    )
    actions = [
        DeclareLaunchArgument('port1', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('port2', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('initialize_joints', default_value='false'),
        DeclareLaunchArgument(
            'mode',
            default_value='none',
            choices=['none', 'replicate', 'sequential', 'roundtrip', 'vision'],
        ),
        DeclareLaunchArgument('params', default_value=str(own / 'config/dual_arm.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    actions.append(OpaqueFunction(function=validate_before_start))
    for i in (1, 2):
        ns = f'mirobot{i}'
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{ns}_display_origin',
                arguments=[
                    '--x',
                    str((i - 1) * 0.5),
                    '--frame-id',
                    'world',
                    '--child-frame-id',
                    f'{ns}/base_link',
                ],
            )
        )
        actions.extend(
            [
                Node(
                    package='mirobot_driver',
                    executable='serial_bridge',
                    namespace=ns,
                    output='screen',
                    parameters=[
                        LaunchConfiguration('params'),
                        {
                            'port': LaunchConfiguration(f'port{i}'),
                            'baud': ParameterValue(
                                LaunchConfiguration('baud'), value_type=int
                            ),
                            'dry_run': ParameterValue(
                                LaunchConfiguration('dry_run'), value_type=bool
                            ),
                        },
                    ],
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    namespace=ns,
                    parameters=[
                        {'robot_description': description, 'frame_prefix': ns + '/'}
                    ],
                    remappings=[('joint_states', 'target_joint_states')],
                ),
                Node(
                    package='mirobot_master_slave',
                    executable='init_joint_once',
                    name=f'{ns}_init_joint_once',
                    condition=IfCondition(LaunchConfiguration('initialize_joints')),
                    parameters=[{'topic': f'/{ns}/target_joint_states'}],
                ),
            ]
        )
    for mode, executable in [
        ('replicate', 'slave_replicator'),
        ('sequential', 'sequential_replicator'),
        ('roundtrip', 'roundtrip_sequencer'),
        ('vision', 'order_yolo_sequencer'),
    ]:
        actions.append(
            Node(
                package='mirobot_master_slave',
                executable=executable,
                output='screen',
                parameters=[LaunchConfiguration('params')],
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration('mode'), "' == '", mode, "'"]
                    )
                ),
            )
        )
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(own / 'rviz/dual_mirobots.rviz')],
        )
    )
    return LaunchDescription(actions)
