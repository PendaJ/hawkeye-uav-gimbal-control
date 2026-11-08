from setuptools import setup
package_name = 'hawkeye_x3_perception'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Msuega Iorpenda',
    maintainer_email='iorpendaj2@gmail.com',
    description='CPU YOLO26n + ByteTrack ROS 2 person detection',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': ['person_tracker = hawkeye_x3_perception.person_tracker:main'],
    },
)
