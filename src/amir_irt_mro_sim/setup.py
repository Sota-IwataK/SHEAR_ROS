from setuptools import setup
import os
from glob import glob

package_name = 'amir_irt_mro_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sota',
    maintainer_email='sota@todo.todo',
    description='Amir IRT-MRO simulation launch package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
