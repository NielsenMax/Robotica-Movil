from setuptools import find_packages, setup

package_name = 'cylinder_detector_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Laureano Hess',
    maintainer_email='laureanohess0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cylinder_detector_node = cylinder_detector_pkg.cylinder_detector_node:main',
        ],
    },
)
