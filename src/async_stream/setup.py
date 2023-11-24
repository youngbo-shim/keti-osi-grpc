import os
from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension

home_path = os.environ.get("HOME")
cmake_prefix_path = os.environ.get("CMAKE_PREFIX_PATH", os.path.join(home_path,'.local/share/cmake/pybind11'))
print(cmake_prefix_path)

client_lib = [
  Extension("client_lib",
            sources=["src/libwrapper.cpp", "src/keti_ros_bridge.cc"],
            include_dirs=[cmake_prefix_path])
]

setup(
  version='0.0.0',
  scripts=['src'],
  packages=['async_stream'],
  package_dir={'': 'src'},
  ext_modules=client_lib
)
