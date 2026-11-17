from setuptools import setup

package_name = 'rs_exploration'

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
    description='Frontier exploration with a decentralized auction',
    license='AGPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'explorer = rs_exploration.explorer:main',
        'air_explorer = rs_exploration.air_explorer:main',
    ]},
)
