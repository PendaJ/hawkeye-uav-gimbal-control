from setuptools import setup

package_name = 'hawkeye_x3_gimbal'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hawkeye_gimbal.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hawkeye project',
    maintainer_email='noreply@example.com',
    description='Gazebo Harmonic ROS 2 bridge and velocity adapter for the Hawkeye X3 yaw-pitch gimbal',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'gimbal_adapter = hawkeye_x3_gimbal.gimbal_adapter:main',
        ],
    },
)
