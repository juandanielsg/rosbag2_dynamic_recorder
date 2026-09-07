from setuptools import find_packages, setup

package_name = 'rosbag2_dynamic_recorder_cli'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # colcon picks its Python test runner from this, not from package.xml: without a declared
    # test dependency on pytest it falls back to `python -m unittest`, which collects nothing
    # here and reports success. Tests that cannot run are worse than no tests.
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='juandanielsg',
    maintainer_email='jdsglez@gmail.com',
    description='ros2 dynrec: command line control of a running rosbag2_dynamic_recorder.',
    license='Apache-2.0',
    entry_points={
        'ros2cli.command': [
            'dynrec = rosbag2_dynamic_recorder_cli.command.dynrec:DynrecCommand',
        ],
        'ros2cli.extension_point': [
            'rosbag2_dynamic_recorder_cli.verb = '
            'rosbag2_dynamic_recorder_cli.verb:VerbExtension',
        ],
        'rosbag2_dynamic_recorder_cli.verb': [
            'add = rosbag2_dynamic_recorder_cli.verb.add:AddVerb',
            'info = rosbag2_dynamic_recorder_cli.verb.info:InfoVerb',
            'pause = rosbag2_dynamic_recorder_cli.verb.pause:PauseVerb',
            'profile = rosbag2_dynamic_recorder_cli.verb.profile:ProfileVerb',
            'profiles = rosbag2_dynamic_recorder_cli.verb.profiles:ProfilesVerb',
            'record = rosbag2_dynamic_recorder_cli.verb.record:RecordVerb',
            'remove = rosbag2_dynamic_recorder_cli.verb.remove:RemoveVerb',
            'resume = rosbag2_dynamic_recorder_cli.verb.resume:ResumeVerb',
            'set = rosbag2_dynamic_recorder_cli.verb.set:SetVerb',
            'snapshot = rosbag2_dynamic_recorder_cli.verb.snapshot:SnapshotVerb',
            'split = rosbag2_dynamic_recorder_cli.verb.split:SplitVerb',
            'status = rosbag2_dynamic_recorder_cli.verb.status:StatusVerb',
            'stop = rosbag2_dynamic_recorder_cli.verb.stop:StopVerb',
            'toggle = rosbag2_dynamic_recorder_cli.verb.toggle:ToggleVerb',
            'topics = rosbag2_dynamic_recorder_cli.verb.topics:TopicsVerb',
        ],
    },
)
