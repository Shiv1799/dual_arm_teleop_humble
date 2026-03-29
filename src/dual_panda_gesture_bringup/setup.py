from setuptools import setup
import os
from glob import glob

package_name = 'dual_panda_gesture_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/dual_panda_gesture_bringup']),
    ('share/dual_panda_gesture_bringup', ['package.xml']),
    ('share/dual_panda_gesture_bringup/launch',
        glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),

],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Dual-arm Panda gesture control using MoveIt Servo and MediaPipe',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dual_gesture_tracker = dual_panda_gesture_bringup.dual_gesture_tracker:main',
            'dual_gesture_bridge = dual_panda_gesture_bringup.dual_gesture_bridge:main',
        ],
    },
)
