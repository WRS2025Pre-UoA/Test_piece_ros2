from setuptools import setup

package_name = 'Test_piece_ros2'

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
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='Test_piece_ros2 package',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = Test_piece_ros2.publisher_node:main',
            'subscriber_node = Test_piece_ros2.subscriber_node:main',
        ],
    },
)
