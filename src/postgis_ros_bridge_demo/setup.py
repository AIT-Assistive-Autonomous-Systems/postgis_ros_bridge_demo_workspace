from setuptools import setup
from glob import glob
import os

package_name = 'postgis_ros_bridge_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('cfg/*')),
        (os.path.join('share', package_name), glob('meshes/*')),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='matthias.schoerghuber@ait.ac.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navsat_transform_node = '
                'postgis_ros_bridge_demo.navsat_transform_node:main'
        ],
    },
)
