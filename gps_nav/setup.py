from setuptools import setup

package_name = 'gps_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamie',
    maintainer_email='jz73@illinois.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid = gps_nav.pid:main',
            'pp_pid = gps_nav.pp_pid:main',
            'track_log = gps_nav.track_log:main',
            'path_visual = gps_nav.path_visual:main'
        ],
    },
)
