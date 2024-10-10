from setuptools import setup

package_name = 'custom_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email',
    description='ROS 2 custom messages with Python serialization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = custom_nodes.talker:main',
            'listener = custom_nodes.listener:main',
        ],
    },
)
