from setuptools import setup

package_name = 'pbcmd'

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
    maintainer='anderson',
    maintainer_email='anderson@mbari.org',
    description='replicates pbcmd for physical MBARI Power Buoy',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pbcmd = pbcmd.pbcmd:main'
        ],
    },
    scripts=[
        'scripts/install_aliases.sh'
    ]
)
