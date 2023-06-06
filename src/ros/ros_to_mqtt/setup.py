from setuptools import setup

package_name = 'ros_to_mqtt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Beechey',
    maintainer_email='s2206112@ed.ac.uk',
    description='Converts ROS messages to MQTT for telemetry system.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_to_mqtt = ros_to_mqtt.ros_to_mqtt:main',
            'ros_publisher = ros_to_mqtt.ros_test_publisher:main',
            'ros_listener = ros_to_mqtt.ros_listener:main'
        ],
    },
)
