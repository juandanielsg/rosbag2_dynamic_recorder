from setuptools import setup

package_name = 'rosbag2_dynamic_recorder_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', ['web/index.html']),
        ('share/' + package_name + '/launch', ['launch/recorder_with_ui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juandanielsg',
    maintainer_email='jdsglez@gmail.com',
    description='Browser UI for rosbag2_dynamic_recorder, served locally by a ROS node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ui_node = rosbag2_dynamic_recorder_ui.ui_node:main',
        ],
    },
)
