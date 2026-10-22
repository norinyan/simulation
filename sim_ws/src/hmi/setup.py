from setuptools import setup

setup(
    name='hmi', version='0.1.0', packages=['hmi'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hmi']),
        ('share/hmi', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Norin', maintainer_email='norin@example.com', license='Apache-2.0',
    description='Qt simulation session and robot task control panel',
    entry_points={'console_scripts': ['panel = hmi.app:main']},
)
