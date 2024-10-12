from setuptools import setup

package_name = 'arduino_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Arduino command node for controlling RGB LED strip',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_commander = arduino_commander.arduino_commander:main',
        ],
    },
)
