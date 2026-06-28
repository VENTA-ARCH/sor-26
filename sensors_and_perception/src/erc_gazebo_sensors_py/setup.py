from setuptools import find_packages, setup

package_name = 'erc_gazebo_sensors_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sachin',
    maintainer_email='sachinmandal3580@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'chase_the_ball = erc_gazebo_sensors_py.chase_the_ball:main',
            'yolo_detector = erc_gazebo_sensors_py.yolo_detector_node:main',
        ],
    },
)
