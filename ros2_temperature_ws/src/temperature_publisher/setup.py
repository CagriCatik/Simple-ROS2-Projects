from setuptools import setup

package_name = 'temperature_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Temperature Publisher for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = temperature_publisher.talker:main',
            'listener = temperature_publisher.listener:main',
        ],
    },
)
