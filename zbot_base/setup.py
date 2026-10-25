from setuptools import find_packages, setup

package_name = 'zbot_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Zumbrunnen',
    maintainer_email='michael.l.zumbrunnen@gmail.com',
    description='Diff drive: RoboClaw driver, wheel odometry, IMU',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 'node_name = zbot_base.module:main',
        ],
    },
)
