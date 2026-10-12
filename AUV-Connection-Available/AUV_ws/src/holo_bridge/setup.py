from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holo_bridge'

setup(
    name=package_name,
    version='0.0.0',
    # 关键点 1：显式指定包名，确保打包器不会漏掉同名子目录
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 关键点 2：有些版本需要显式安装 launch 文件（如果有的话）
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cjx',
    maintainer_email='cjx@todo.todo',
    description='HoloOcean ROS2 Bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这里的格式是：终端指令 = 包名.文件名:函数名
            'holo_node = holo_bridge.bridge_node:main'
            
        ],
    },
)