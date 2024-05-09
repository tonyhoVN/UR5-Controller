from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os,sys

current_dir = os.path.dirname(os.path.abspath(__file__))

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['UR5_API'],
    package_dir={'': ''},
)

setup(**setup_args)