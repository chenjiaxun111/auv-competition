from glob import glob
import os

from setuptools import setup


package_name = 'auv_competition_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cjx',
    maintainer_email='cjx@todo.todo',
    description='Target-relative planner and visual servo for ball and gate tasks',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_web_viewer = '
            'auv_competition_control.perception_web_viewer:main',
            'competition_controller = '
            'auv_competition_control.competition_controller_node:main',
            'dual_model_perception = '
            'auv_competition_control.dual_model_perception_node:main',
            'zed_sdk_source = '
            'auv_competition_control.zed_sdk_source_node:main',
            'mock_objects3d = auv_competition_control.mock_objects3d_node:main',
        ],
    },
)
