from glob import glob
from setuptools import find_packages, setup

package_name = 'ffw_teleop'
authors_info = [
    ('Sungho Woo', 'wsh@robotis.com'),
    ('Woojin Wie', 'wwj@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)
setup(
    name=package_name,
    version='2.3.2',
    packages=find_packages(exclude=[]),
    package_data={'ffw_teleop': ['human_demo_web.html', 'mount_editor.html',
        'mount_assets/*.json', 'mount_assets/models/*', 'mount_assets/vendor/*',
        'mount_assets/vendor/jsm/controls/*', 'mount_assets/vendor/jsm/loaders/*',
        'mount_assets/vendor/jsm/utils/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.json')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='FFW teleop ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_demo = ffw_teleop.human_demo:main',
            'foot_pedal = ffw_teleop.foot_pedal:main',
            'lift_keyboard = ffw_teleop.lift_keyboard:main',
            'record_rosbag = ffw_teleop.record_rosbag:main',
            'replay_rosbag = ffw_teleop.replay_rosbag:main',
            'keyborad_control = ffw_teleop.keyboard_control:main',
            'mobile_teleop = ffw_teleop.mobile_teleop:main',
            'contactglove_sharpa = ffw_teleop.contactglove_sharpa:main',
            'sharpa_vr_bridge = ffw_teleop.sharpa_vr_bridge:main',
            'sharpa_vuer_publisher = ffw_teleop.sharpa_vuer_publisher:main',
            'calibrate_sharpa_vr = ffw_teleop.calibrate_vr:main',
        ],
    },
)
