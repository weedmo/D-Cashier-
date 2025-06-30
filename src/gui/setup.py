from setuptools import setup
import os
from glob import glob

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # 'gui' 디렉터리 안에 __init__.py 있어야 함
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # JSON 파일을 install/share/gui/ 로 복사하도록 설정
        (os.path.join('share', package_name), glob('gui/product_data.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joonmo',
    maintainer_email='joonmo@example.com',
    description='DOOSAN MARKET GUI 패키지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'product_service = gui.product_service:main',
        ],
    },
)
