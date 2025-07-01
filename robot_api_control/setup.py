from setuptools import find_packages, setup
import os
import glob
package_name = 'robot_api_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/resource', glob.glob('resource/.env')),
        # ('share/ament_index/resource_index/packages',['resource/' + 'voice_processing']),
        # ('share/voice_processing', ['package.xml']),
        # ('share/object_detection', ['package.xml']),
        # ('share/robot_control', ['package.xml']),

        # ('share/' + package_name + '/launch', glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='jjoonmo0212@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller = robot_api_control.main_controller:main',
            'home = robot_api_control.move_home:main',
            
        ],
    },
)
