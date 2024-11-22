# robot_control/bin/fftai_dds_bridge_wrapper.py

import os
import sys
import subprocess

def main():
    # Get the directory where this script resides
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Path to the binary executable
    executable = os.path.join(script_dir, 'fftai_dds_bridge')
    # Ensure the executable has execute permissions
    os.chmod(executable, 0o755)
    # Call the executable with any arguments passed to the script
    subprocess.call([executable] + sys.argv[1:])
