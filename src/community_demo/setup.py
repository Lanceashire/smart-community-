from setuptools import find_packages, setup

package_name = 'community_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/community.sdf']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
        ('share/' + package_name + '/gui', ['gui/overview.config']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lanceashire',
    maintainer_email='noreply@example.com',
    description='Single-lap smart community simulation and patrol',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'patrol = community_demo.patrol:main',
        'vision = community_demo.vision:main',
        'cmd_bridge = community_demo.cmd_bridge:main',
    ]},
)
