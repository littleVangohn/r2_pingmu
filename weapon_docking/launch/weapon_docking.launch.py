from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def _display_env():
    env = {}
    display = os.environ.get('DISPLAY') or ':0'
    env['DISPLAY'] = display

    if 'XAUTHORITY' in os.environ:
        env['XAUTHORITY'] = os.environ['XAUTHORITY']
    else:
        gdm_xauthority = '/run/user/{}/gdm/Xauthority'.format(os.getuid())
        user_xauthority = os.path.expanduser('~/.Xauthority')
        if os.path.exists(gdm_xauthority):
            env['XAUTHORITY'] = gdm_xauthority
        elif os.path.exists(user_xauthority):
            env['XAUTHORITY'] = user_xauthority

    if 'QT_QPA_PLATFORM' not in os.environ:
        env['QT_QPA_PLATFORM'] = 'xcb'
    return env

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('weapon_docking'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='weapon_docking',
            executable='weapon_docking_node',
            name='weapon_docking_node',
            output='screen',
            additional_env=_display_env(),
            parameters=[config_file]
        )
    ])
