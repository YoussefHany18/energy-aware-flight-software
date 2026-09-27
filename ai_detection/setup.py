from setuptools import setup
from glob import glob
import os

package_name = 'ai_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'numpy<2',          # ensures cv_bridge compatibility
        'opencv-python',
        'ncnn',
        'cv_bridge',
        'rclpy',
        'sensor_msgs'
    ],
    zip_safe=True,
    maintainer='Youssef Hany',
    maintainer_email='your_email@example.com',
    description='AI detection using NCNN for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    data_files=[
        # required by ROS2 ament_index
        ('share/ament_index/resource_index/packages', ['resource/ai_detection']),
        ('share/ai_detection', ['package.xml']),
        # install all files in best_ncnn_model
        (os.path.join('share', package_name, 'best_ncnn_model'),
         glob(os.path.join(package_name, 'best_ncnn_model', '*'))),
    ],
    entry_points={
        'console_scripts': [
            'ai_node = ai_detection.ai_node:main',
        ],
    },
)