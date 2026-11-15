from glob import glob
from setuptools import setup

p = 'micromouse_practice'
setup(name=p, version='1.0.0', packages=[p],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),
                  ('share/'+p,['package.xml'])]+[('share/'+p+'/'+d,glob(d+'/*')) for d in ['launch','worlds','config','rviz','urdf','assets']],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='student', maintainer_email='student@example.com',
      description='Integrated sensor-based micromouse course experiments', license='Apache-2.0',
      entry_points={'console_scripts':['wall_follow = micromouse_practice.wall_follow:main',
                                      'map_recorder = micromouse_practice.map_recorder:main',
                                      'flood_navigation = micromouse_practice.flood_navigation:main',
                                      'astar_navigation = micromouse_practice.astar_navigation:main']})
