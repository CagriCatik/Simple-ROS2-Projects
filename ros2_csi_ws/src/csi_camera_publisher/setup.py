from setuptools import setup

package_name = 'csi_camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'csi_camera_publisher.csi_camera_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for CSI camera video streaming or OpenCV algorithm',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'csi_camera_node = csi_camera_publisher.csi_camera_node:main',  # publisher node
        ],
    },
)
