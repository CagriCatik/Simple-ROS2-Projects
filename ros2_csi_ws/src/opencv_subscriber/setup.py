from setuptools import setup

package_name = 'opencv_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'opencv_subscriber.opencv_algorithm_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='OpenCV subscriber node for ROS2 using Python',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'opencv_algorithm_node = opencv_subscriber.opencv_algorithm_node:main',
        ],
    },
)
