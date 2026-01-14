from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'my_bot_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madscientist',
    maintainer_email='madscientist@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'sensor_node=my_bot_logic.distance_sensor:main',
           'motor_node=my_bot_logic.motor_node:main',
           'turtle_spy=my_bot_logic.turtle_spy:main',
           'turtle_bouncer=my_bot_logic.turtle_bouncer:main',
        ],
    },
)
