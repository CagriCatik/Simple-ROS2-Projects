from setuptools import setup

package_name = 'stereo_depth_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'stereo_depth_subscriber.stereo_depth_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Stereo and Depth subscriber node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'stereo_depth_node = stereo_depth_subscriber.stereo_depth_node:main',
        ],
    },
)
