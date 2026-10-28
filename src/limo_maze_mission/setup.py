from glob import glob
from setuptools import find_packages, setup

setup(name='limo_maze_mission', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/limo_maze_mission']),
                  ('share/limo_maze_mission', ['package.xml']),
                  ('share/limo_maze_mission/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='LIMO project', maintainer_email='maintainer@example.invalid',
      description='Unknown-map exploration, observed target annotation, and return.',
      license='Apache-2.0',
      entry_points={'console_scripts': ['mission = limo_maze_mission.node:main']})
