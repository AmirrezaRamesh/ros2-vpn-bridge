#example of a setup.py for the package if you don't know how to modify

from setuptools import find_packages, setup

package_name = 'package_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amir',
    maintainer_email='rameshamirreza3@gmail.com',
    description='ROS over VPN',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clientA = package_name.clientA:main' #same for clientB
        ],
    },
)
