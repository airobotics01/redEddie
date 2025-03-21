from setuptools import find_packages, setup

package_name = 'fr3_control'

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
    maintainer='jeonchanwook',
    maintainer_email='jeonchanwook@todo.todo',
    description='The fr3_control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'home_position_server = fr3_control.home_position_server:main',
            'joint_state_subscriber = fr3_control.joint_state_subscriber:main',
            'ros2_publisher = fr3_control.ros2_publisher:main',
        ],
    },
)
