from setuptools import setup
from glob import glob

package_name = 'mobile_base_unit'
source_package_name = 'src'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,source_package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name, glob("launch/*_launch.py")),
        ("share/" + package_name + "/config", glob("config/*.*")),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KraftRobotics',
    maintainer_email='kraft2k@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobile_base = mobile_base_unit.mobile_base:main',
            'detect_reach_marker = src.detect_reach_marker:main',
            'follow_me = src.follow_me:main',
            'follow_wall = src.follow_wall:main',
            'voice_control = src.voice_control:main',
            'up_front_servo_server = src.up_front_servo_server:main',
            'up_front_servo_client = src.up_front_servo_client:main',
            'speed_test = src.speed_test:main',
            
        ],
    },
)
