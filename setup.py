import os
from setuptools import find_packages, setup

package_name = 'object_detection_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/object_detection_bot']),
    ('share/object_detection_bot', ['package.xml']),
    # Add these two lines below
    (os.path.join('share', 'object_detection_bot', 'launch'), ['launch/rsp_spawn.launch.py']),
    (os.path.join('share', 'object_detection_bot', 'urdf'), ['urdf/robot.urdf']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='salman@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
'detector = object_detection_bot.detector:main',
        ],
    },
)
