import os                      # add
from glob import glob          # add
from setuptools import find_packages, setup

package_name = 'zbot_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),   # add
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),        # add
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Zumbrunnen',
    maintainer_email='michael.l.zumbrunnen@gmail.com',
    description='SO-101 to ROS 2 bridge: joint states and commands',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 'node_name = zbot_arm.module:main',
        ],
    },
)
