from glob import glob

from setuptools import find_packages, setup

package_name = 'fossbot_control_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wiktor',
    maintainer_email='wiktor77887@gmail.com',
    description='Control interface for the FOSSBot mobile robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_teleop = fossbot_control_interface.click_teleop:main',
            'dashboard = fossbot_control_interface.dashboard:main',
            'gesture_control = fossbot_control_interface.gesture_control:main',
        ],
    },
)
