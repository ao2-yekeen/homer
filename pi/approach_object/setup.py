from glob import glob
from setuptools import setup
setup(name='approach_object', version='0.1.0', packages=['approach_object'],
 data_files=[('share/ament_index/resource_index/packages', ['resource/approach_object']), ('share/approach_object', ['package.xml']), ('share/approach_object/config', glob('config/*.yaml') + glob('config/*.json')), ('share/approach_object/launch', glob('launch/*.py'))],
 install_requires=['setuptools', 'PyYAML', 'numpy', 'scipy'], zip_safe=True,
 maintainer='Homer maintainers', maintainer_email='ao2-yekeen@users.noreply.github.com', description='Supervised named floor-object approach', license='MIT',
 entry_points={'console_scripts': ['supervisor = approach_object.supervisor:main', 'detector = approach_object.detector:main']})
