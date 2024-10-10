from setuptools import setup

package_name = 'realsense_camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='RealSense camera publisher node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'realsense_camera_node = realsense_camera_publisher.realsense_camera_node:main',
        ],
    },
)
