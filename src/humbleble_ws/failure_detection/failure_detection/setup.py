from setuptools import setup
import os
from glob import glob

package_name = 'failure_detection'
bayesian_network = "failure_detection/bayesian_network"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, bayesian_network],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kawase',
    maintainer_email='kawase.haruyoshi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'bt_generator_service = failure_detection.bt_generator_service:main',
            'bt_monitoring = failure_detection.monitoring:main',
            'failure_detection = failure_detection.failure_detection:main'
        ],
    },
)
