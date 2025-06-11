from setuptools import setup

package_name = 'airport_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=['airport_navigation',],
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dynamic_nav_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yaswanth',
    maintainer_email='yaswanthkumarsingampalli77@gmail.com',
    description='Airport navigation using ROS 2 and GNSS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_source_node = airport_navigation.gnss_source_node:main',
            'destination_node = airport_navigation.destination_node:main',
            'navigation_node = airport_navigation.navigation_node:main',
        ],
    },
)
