#!/bin/bash

# Check if fourier_grx_dds is installed
site_packages_dir=$(python3 -c "import site; print(site.getsitepackages()[0])")

robot_control_libraries="$site_packages_dir/fourier_grx_dds/libraries"

if [ ! -d "$robot_control_libraries" ]; then
    echo "Error: $robot_control_libraries does not exist. "
    echo "Try: pip install fourier-grx-dds==0.1.0b0"
    return
fi

echo "export LD_LIBRARY_PATH=$robot_control_libraries:$LD_LIBRARY_PATH" >> ~/.bashrc

echo "LD_LIBRARY_PATH set to: $LD_LIBRARY_PATH"



