from setuptools import find_packages, setup
import glob

package_name = 'voice_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', glob.glob('resource/*') + ['resource/.env']),
        ('share/' + package_name, ['voice_processing/MicController.py']),
        ('share/' + package_name, ['voice_processing/wakeup_word.py']),
        ('share/' + package_name, ['package.xml']),
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
            "get_keyword = voice_processing.get_keyword:main",
        ],
    },
)
