
#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['tbd_multi_robot_relay']
d['package_dir'] = {'': 'python_src'}

setup(**d)
