from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learn2race_competition'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Ruben Pereira',
    maintainer_email='1181915@isep.ipp.pt',
    description='Competition state machine, LED signalling and rule enforcement.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'competition_fsm = learn2race_competition.competition_fsm_node:main',
            'led_signalling = learn2race_competition.led_signalling_node:main',
            'lap_counter = learn2race_competition.lap_counter_node:main',
            'dragster_referee = learn2race_competition.dragster_referee_node:main',
            'gp_referee = learn2race_competition.gp_referee_node:main',
        ],
    },
)
