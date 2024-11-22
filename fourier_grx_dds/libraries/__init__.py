import os
import ctypes
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))

lib_path_1 = os.path.join(current_dir, 'libpycdr.so.2')
lib_path_2 = os.path.join(current_dir, 'libpydds.so.3.1')

try:
    libpydds = ctypes.CDLL(lib_path_1)
except OSError as e:
    raise OSError(f"Could not load shared library {lib_path_1}: {e}")

try:
    libpycdr = ctypes.CDLL(lib_path_2)
except OSError as e:
    raise OSError(f"Could not load shared library {lib_path_2}: {e}")
