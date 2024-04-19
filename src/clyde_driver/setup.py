from setuptools import find_packages, setup

package_name = 'clyde_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GroupC',
    maintainer_email='peter.lacaze.2019@uni.strath.ac.uk',
    description='Package holding all resource required for communicating with and on the bittle',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'basic_driver = clyde_driver.bittle_driver_basic:main',
                'joint_reader = clyde_driver.joint_state_publisher:main'
        ],
    },
)
