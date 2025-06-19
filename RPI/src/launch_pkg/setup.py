from setuptools import find_packages, setup

package_name = 'launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/launch_pkg']),
    ('share/launch_pkg', ['package.xml']),
    ('share/launch_pkg/launch', ['launch/start_all.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berko-vm',
    maintainer_email='berko-vm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
