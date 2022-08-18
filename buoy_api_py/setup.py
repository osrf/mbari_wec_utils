from setuptools import setup

package_name = 'buoy_api_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=['buoy_api'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anderson',
    maintainer_email='anderson@mbari.org',
    description='MBARI Power Buoy API (python)',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
