from setuptools import setup

package_name = 'rs_perception'

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
    description='Thermal people detection and mark fusion',
    license='AGPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'thermal_detector = rs_perception.thermal_detector:main',
        'people_fusion = rs_perception.people_fusion:main',
        'rgb_detector = rs_perception.rgb_detector:main',
        'air_thermal_detector = rs_perception.air_thermal_detector:main',
    ]},
)
