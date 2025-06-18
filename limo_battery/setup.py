from setuptools import find_packages, setup

package_name = 'limo_battery'

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
    maintainer='user',
    maintainer_email='yuma4438@outlook.jp',
    description='Battery publisher node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_publisher_node = limo_battery.battery_publisher_node:main',
        ],
    },
)
