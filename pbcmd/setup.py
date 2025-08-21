import atexit
from functools import partial
import os
from subprocess import check_call

from setuptools import setup
from setuptools.command.install import install


package_name = 'pbcmd'


def _post_install(inst=None):
    print('~~~ post-install ~~~')
    if inst is not None:
        install_aliases = os.path.join(inst.install_scripts, 'install_aliases.sh')
        print('post-install scripts: ', install_aliases)
        check_call(install_aliases)


class PostInstall(install):

    def __init__(self, *args, **kwargs):
        super(PostInstall, self).__init__(*args, **kwargs)
        atexit.register(partial(_post_install, self))


setup(
    name=package_name,
    version='2.0.0',
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
        'scripts/install_aliases.sh',
        'scripts/uninstall_aliases.sh'
    ],
    cmdclass={'install': PostInstall}
)
