import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'led_emitter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
#    packages=find_packages('include', exclude=['test']),
    include_package_data=True,
#    package_dir={'': 'include'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config/led_emitter_node'), glob('config/led_emitter_node/*.yaml'))
#        (os.path.join('share', package_name, 'include/rgb_led'), glob('include/rgb_led/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='stuettgen@fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_emitter_node = led_emitter.led_emitter_node:main',
            'minimal_param_node = led_emitter.minimal_param_node:main'
        ],
    },
)
