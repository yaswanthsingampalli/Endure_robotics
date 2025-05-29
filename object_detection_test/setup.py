from setuptools import find_packages, setup

package_name = 'object_detection_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yaswanth Kumar Singampalli',
    maintainer_email='yaswanthkumarsingampalli77@gmail.com',
    description='Fake camera and YOLO object detector',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_camera_publisher = object_detection_test.fake_camera_publisher:main',
            'object_detector = object_detection_test.object_detector:main',
        ],
    },
)
