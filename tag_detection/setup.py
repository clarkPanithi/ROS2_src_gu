from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tag_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.yaml')),
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
        'tf_listener = tag_detection.tf_listener:main',],
    },
)
