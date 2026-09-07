from setuptools import find_packages, setup

package_name = 'dynrec'

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
    # here and reports success. Same trap the CLI and UI packages document.
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='juandanielsg',
    maintainer_email='jdsglez@gmail.com',
    description='Python library for driving a running rosbag2_dynamic_recorder from a script.',
    license='Apache-2.0',
)
