from glob import glob
from setuptools import setup

setup(
    name='homer_slam', version='0.1.0', packages=['homer_slam'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/homer_slam']),
                ('share/homer_slam', ['package.xml', 'dependencies.repos']),
                *[('share/homer_slam/' + folder, glob(folder + '/*'))
                  for folder in ('launch', 'config', 'rviz', 'patches')],
                ('share/homer_slam/urdf', ['../../simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf'])],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='Homer maintainers', maintainer_email='ao2-yekeen@users.noreply.github.com',
    description='Encoder-free LiDAR SLAM and lightweight visualization', license='MIT',
    entry_points={'console_scripts': [
        'robot_markers = homer_slam.nodes:markers_main',
        'health_monitor = homer_slam.nodes:health_main',
        'viewer_status = homer_slam.nodes:viewer_main',
        'save_map = homer_slam.operations:save_main',
        'record = homer_slam.operations:record_main',
        'baseline = homer_slam.baseline:main',
        'network = homer_slam.network:main',
    ]},
)
