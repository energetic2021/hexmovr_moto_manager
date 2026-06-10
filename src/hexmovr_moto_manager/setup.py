from setuptools import find_packages, setup
from glob import glob

package_name = 'hexmovr_moto_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hexmovr02',
    maintainer_email='energetic2021@gmail.com',
    description='Hexmovr motor manager with SocketCAN protocol support and RViz controls',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'moto_scaner = hexmovr_moto_manager.moto_scaner:main'
        ],
    },
)
