from glob import glob
import os

from setuptools import setup

package_name = 'rov_thruster_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='cjx',
    maintainer_email='cjx@todo.todo',
    description='Safe ROS 2 motion and telemetry bridge for rov-master',
    license='MIT',
    entry_points={
        'console_scripts': [
            'thruster_bridge_node = rov_thruster_bridge.thruster_bridge_node:main',
            'send_motion_test = rov_thruster_bridge.send_motion_test:main',
            'axis_sweep_test = rov_thruster_bridge.axis_sweep_test:main',
            'communication_stress_test = rov_thruster_bridge.communication_stress_test:main',
            'competition_command_test = rov_thruster_bridge.competition_command_test:main',
        ],
    },
)
