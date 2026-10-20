from glob import glob
from setuptools import setup

setup(
    name='simulation_hmi', version='0.1.0', packages=['simulation_hmi'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/simulation_hmi']),
        ('share/simulation_hmi', ['package.xml', 'README.md']),
        ('share/simulation_hmi/config', glob('config/*.yaml')),
        ('share/simulation_hmi/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Norin', maintainer_email='norin@example.com', license='Apache-2.0',
    description='Qt simulation session and robot task control panel',
    entry_points={'console_scripts': ['panel = simulation_hmi.app:main']},
)
