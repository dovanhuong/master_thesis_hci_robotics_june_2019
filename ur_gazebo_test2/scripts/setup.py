from distutils.core import setup, Extension
from catkin_pkg.python_setup import generate_distutils_setup

ur_kin_py = Extension('ur_kin_py',
                      include_dirs=['/home/modmanvision/catkin_ws/src/universal_robot/ur_kinematics/src/',
                                    '/home/modmanvision/catkin_ws/src/universal_robot/ur_kinematics/include/',
                                    '/home/modmanvision/library/cpp/boost_1_68_0'],
                      extra_compile_args=["-fPIC"],
                      extra_link_args=["-lboost_numpy", "-lboost_python"],
                      sources=['/home/modmanvision/catkin_ws/src/universal_robot/ur_kinematics/src/ur_kin_py.cpp'])

setup(name='ur_kin_py',
      version='1.0',
      ext_modules=[ur_kin_py])