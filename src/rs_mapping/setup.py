from setuptools import setup

package_name = 'rs_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bekai Vision',
    maintainer_email='bigalex95@users.noreply.github.com',
    description='Shared occupancy grid built from all robots lidars',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['shared_map = rs_mapping.shared_map:main',
                                      'other_robots = rs_mapping.other_robots:main']},
)
