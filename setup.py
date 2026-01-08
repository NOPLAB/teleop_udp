from setuptools import find_packages, setup

package_name = 'teleop_udp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nop',
    maintainer_email='noplab90@gmail.com',
    description='controller data via UDP',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_sender_node = teleop_udp.udp_sender_node:main',
            'udp_receiver_node = teleop_udp.udp_receiver_node:main',
        ],
    },
)
