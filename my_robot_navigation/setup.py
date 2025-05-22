from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            [f for f in glob('launch/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config'), 
            [f for f in glob('config/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clarkpanithi',
    maintainer_email='panithithungkao@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'auto_explorer = my_robot_navigation.auto_explorer:main',
        'twopoints_explorer = my_robot_navigation.twopoints_explorer:main',
        ],
    },
)
