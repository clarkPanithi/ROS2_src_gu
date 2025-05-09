from setuptools import setup
from glob import glob

package_name = 'apriltag_me'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.*')),
        ('share/' + package_name + '/cfg', glob('cfg/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clarkpanithi',
    maintainer_email='panithithungkao@gmail.com',
    description='AprilTag detection package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_me = apriltag_me.apriltag_me:main',
            'static_camera_info_publisher = apriltag_me.static_camera_info_publisher:main',
        ],
    },
)
