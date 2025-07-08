from setuptools import setup

package_name = 'airport_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/dynamic_nav_launch.py']),
        ('share/' + package_name + '/config', ['config/navigation_params.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous robot navigation system using GNSS and dynamic routing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_source_node = airport_navigation.gnss_source_node:main',
            'destination_node = airport_navigation.destination_node:main',
            'navigation_node = airport_navigation.navigation_node:main',
            'route_manager_node = airport_navigation.route_manager_node:main',
            'rviz_debug_node = airport_navigation.rviz_debug_node:main',  
            'diagnostics_node = airport_navigation.diagnostics_node:main' ,
            'dummy_origin_publisher = airport_navigation.dummy_origin_publisher:main' ,
            'marker_viz_node = airport_navigation.marker_viz_node:main' ,
            'pose_publisher_node = airport_navigation.pose_publisher_node:main'
        ], 
    },

)
