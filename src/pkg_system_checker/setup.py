from setuptools import setup

package_name = 'pkg_system_checker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A small package to check required ROS2 nodes/topics at launch',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_system = pkg_system_checker.check_system:main',
        ],
    },
)
