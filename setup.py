from setuptools import setup, find_packages

package_name = 'ruiyan_hand_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rh2_controller.launch.py', 'launch/rh2_test.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial', 'python-can'],
    zip_safe=True,
    maintainer='LiTang',
    maintainer_email='litang0617@outlook.com',
    description='ROS2 driver for Ruiyan dexterous hand with RS485 communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ruiyan_hand_node = ruiyan_hand_driver.ruiyan_hand_node:main',
        ],
    },
)
