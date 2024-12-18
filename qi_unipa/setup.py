from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qi_unipa'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='rossbong99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "qi_unipa_movement = qi_unipa.qi_unipa_movement:main",
            "qi_unipa_sensor = qi_unipa.qi_unipa_sensor:main",
            "qi_unipa_speech = qi_unipa.qi_unipa_speech:main",
            "qi_unipa_tracking = qi_unipa.qi_unipa_tracking:main",
            "qi_unipa_vision = qi_unipa.qi_unipa_vision:main",
        ],
    },
)
