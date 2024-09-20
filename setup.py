from setuptools import setup

package_name = 'test_piece_ros2'  # 小文字に変更

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'cv_bridge', 'opencv-python'],
    zip_safe=True,
    maintainer='your_name',  # 自分の名前に変更
    maintainer_email='your_email@example.com',  # 自分のメールアドレスに変更
    description='A ROS2 package for image processing',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = test_piece_ros2.image_publisher:main',  # パッケージ名を小文字に変更
            'image_subscriber = test_piece_ros2.image_subscriber:main',  # パッケージ名を小文字に変更
        ],
    },
)
