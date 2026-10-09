from setuptools import find_packages, setup
from pathlib import Path

package_name = 'community_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[(str(Path('share') / package_name / directory), [str(p) for p in Path(directory).iterdir() if p.is_file()])
          for base in ('worlds','launch','gui','models','config','maps')
          for directory in [str(Path(base)), *[str(d) for d in Path(base).rglob('*') if d.is_dir()]]],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lanceashire',
    maintainer_email='noreply@example.com',
    description='Single-lap smart community simulation and patrol',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'localization = community_demo.localization:main',
        'patrol = community_demo.patrol:main',
        'vision = community_demo.vision:main',
        'cmd_bridge = community_demo.cmd_bridge:main',
        'traffic_signals = community_demo.traffic_signals:main',
        'sensor_stress = community_demo.sensor_stress:main',
        'slam_scan = community_demo.slam_scan:main',
    ]},
)
