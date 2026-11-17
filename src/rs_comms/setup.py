from setuptools import setup

package_name = 'rs_comms'

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
    description='Swarm communication model: comms_sim radio node and per-node comms agent',
    license='AGPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'comms_sim = rs_comms.comms_sim:main',
        'comms_agent = rs_comms.agent:main',
        'onboard = rs_comms.onboard:main',
    ]},
)
