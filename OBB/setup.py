from setuptools import find_packages, setup
import glob
import os

package_name = 'OBB'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsbae',
    maintainer_email='jsbae@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = OBB.detection:main',
            'is_adult = OBB.adult_check_face:main',
            'pause_test = OBB.pause_test:main',
            'real = OBB.detection_real:main',
            'canel = OBB.cancel_detection:main',
            'test = OBB.test_yolo:main'         
                 
        ],
    },
)
