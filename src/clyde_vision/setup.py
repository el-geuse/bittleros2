from setuptools import find_packages, setup

package_name = 'clyde_vision'

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
    maintainer='ros',
    maintainer_email='peter.lacaze.2019@uni.strath.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_locator = clyde_vision.person_locator_node:main',
            'fall_detector = clyde_vision.fall_detector_node:main',
        ],
    },
)
