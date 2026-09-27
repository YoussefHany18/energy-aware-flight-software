from setuptools import find_packages, setup

package_name = 'drone_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/egret.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone',
    maintainer_email='drone@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'energy_monitor=drone_system.energy_monitor:main',
        'docking_manager=drone_system.docking_manager:main',
        'precision_land=drone_system.precision_land:main',
        'aruco_tracker=drone_system.aruco_tracker:main',
        'state_machine=drone_system.state_machine:main',
        'flight_manager=drone_system.flight_manager:main',
        ],
    },
)
