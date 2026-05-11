from setuptools import find_packages, setup

package_name = 'wheel_changing_vision_ik'

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
    maintainer='fahad',
    maintainer_email='fahad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_test_current_pose = wheel_changing_vision_ik.ik_test_current_pose:main',
            'wheel_3d_detector = wheel_changing_vision_ik.wheel_3d_detector:main',
            'wheel_point_ik_probe = wheel_changing_vision_ik.wheel_point_ik_probe:main',
            'auto_move_to_wheel_grab = wheel_changing_vision_ik.auto_move_to_wheel_grab:main',
        ],
    },
)
