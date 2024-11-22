#!/bin/bash

site_packages_dir=$(python3 -c "import site; print(site.getsitepackages()[0])")

fourier_grx_dds_libraries="$site_packages_dir/fourier_grx_dds/libraries"

# Check if fourier_grx_dds is installed
if [ ! -d "$fourier_grx_dds_libraries" ]; then
    echo "Error: $fourier_grx_dds_libraries does not exist. "
    echo "Try: pip install fourier-grx-dds"
    return
fi

# Set LD_LIBRARY_PATH
echo "export LD_LIBRARY_PATH=$fourier_grx_dds_libraries:\$LD_LIBRARY_PATH" >> ~/.bashrc