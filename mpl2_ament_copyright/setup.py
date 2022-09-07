from setuptools import setup

package_name = 'mpl2_ament_copyright'

setup(
    name=package_name,
    version='0.0.0',
    packages=['mpl2_ament_copyright'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    package_data={'': [
        'template/*',
    ]},
    zip_safe=True,
    maintainer='anderson',
    maintainer_email='anderson@mbari.org',
    description='Add support for MPL-2.0 License to ament_copyright',
    license='MPL-2.0',
    tests_require=['pytest'],
    entry_points={
        'ament_copyright.license': [
            'mpl2 = mpl2_ament_copyright:mpl2'
        ],
    },
)
