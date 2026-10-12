from setuptools import setup
import os
from glob import glob

package_name = 'nokov_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cjx',
    maintainer_email='cjx@todo.todo',
    description='Nokov mocap to ROS2 TF bridge',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'nokov_tf_bridge = nokov_bridge.nokov_tf_bridge:main',
            'diagnose_nokov = nokov_bridge.diagnose_nokov:main',
        ],
    },
)
