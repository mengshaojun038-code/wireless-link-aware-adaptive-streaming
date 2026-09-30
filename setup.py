from setuptools import find_packages, setup

package_name = 'adaptive_stream'

setup(
    name=package_name,
    version='1.0.0',

    packages=find_packages(),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            [
                'package.xml',
                'README.md'
            ]
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='Competition Submission',
    maintainer_email='mengshaojun038@gmail.com',

    description=(
        'ROS 2 network-aware adaptive visual streaming '
        'for mobile robot remote monitoring.'
    ),

    license='Proprietary',

    entry_points={
        'console_scripts': [
            (
                'adaptive_stream_node = '
                'adaptive_stream.adaptive_stream_node:main'
            ),
            (
                'network_monitor_node = '
                'adaptive_stream.network_monitor_node:main'
            ),
        ],
    },
)
