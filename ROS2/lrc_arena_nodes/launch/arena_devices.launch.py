from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch all arena device controllers for LRC 2025
    """
    return LaunchDescription([
        Node(
            package='lrc_arena_nodes',
            executable='pump_controller',
            name='pump_controller',
            parameters=[{
                'step_pin': 17,
                'dir_pin': 27,
                'steps_per_ml': 80.0,
                'target_volume_ml': 125.0
            }],
            output='screen'
        ),
        Node(
            package='lrc_arena_nodes',
            executable='led_controller',
            name='led_controller',
            parameters=[{
                'num_pixels': 30,
                'brightness': 0.5
            }],
            output='screen'
        ),
        Node(
            package='lrc_arena_nodes',
            executable='loadcell_lcd_controller',
            name='loadcell_lcd_controller',
            parameters=[{
                'expected_weight_g': 125.0,
                'tolerance_g': 5.0,
                'team_name': 'Team LRC',
                'hx711_dout': 5,
                'hx711_sck': 6
            }],
            output='screen'
        )
    ])
