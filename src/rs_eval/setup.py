from setuptools import setup

package_name = 'rs_eval'

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
    description='Exploration coverage metrics and plots',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'coverage = rs_eval.coverage:main',
        'plot_coverage = rs_eval.plot_coverage:main',
        'render_map = rs_eval.render_map:main',
    ]},
)
