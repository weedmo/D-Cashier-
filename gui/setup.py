from setuptools import setup

package_name = 'gui'
resource_files = [
        'resource/product_data.json',
    ]
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/resource', resource_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joonmo',
    maintainer_email='joonmo@example.com',
    description='Test setup',
    license='MIT',
    entry_points={
        'console_scripts': [
            'real_gui = gui.real_gui:main',
        ],
    },
)
