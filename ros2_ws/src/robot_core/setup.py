from setuptools import find_packages, setup

package_name = 'robot_core'

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
    maintainer='quin',
    maintainer_email='earnearn.panuditeekun@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = robot_core.mission_node:main',
            'movement_node = robot_core.movement_node:main',
            'entry_controller = robot_core.entry_controller:main',
            'movement_test_node = robot_core.movement_test_node:main',
        ],
    },
)
