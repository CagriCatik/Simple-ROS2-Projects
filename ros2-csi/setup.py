from setuptools import setup

package_name = 'csi_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='CSI Camera Node for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csi_camera_node = csi_camera.csi_camera_node:main',
        ],
    },
)
