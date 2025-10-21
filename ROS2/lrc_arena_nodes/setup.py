from setuptools import setup
import os
from glob import glob

package_name = 'lrc_arena_nodes'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LRC Team',
    maintainer_email='team@example.com',
    description='ROS 2 nodes for Lam Research Challenge 2025 arena devices',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pump_controller = lrc_arena_nodes.pump_controller:main',
            'led_controller = lrc_arena_nodes.led_controller:main',
            'loadcell_lcd_controller = lrc_arena_nodes.loadcell_lcd_controller:main',
        ],
    },
)
