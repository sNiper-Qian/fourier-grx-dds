#!/bin/bash

site_packages_dir=$(python -c "import site; print(site.getsitepackages()[0])")

fourier_grx_dds_libraries="$site_packages_dir/fourier_grx_dds/libraries"
rm $fourier_grx_dds_libraries/libfastcdr.so.2
rm $fourier_grx_dds_libraries/libfastdds.so.3.1
ln -s $fourier_grx_dds_libraries/libfastcdr.so.2.2.5 $fourier_grx_dds_libraries/libfastcdr.so.2
ln -s $fourier_grx_dds_libraries/libfastdds.so.3.1.0 $fourier_grx_dds_libraries/libfastdds.so.3.1

# Check if fourier_grx_dds is installed
if [ ! -d "$fourier_grx_dds_libraries" ]; then
    echo "Error: $fourier_grx_dds_libraries does not exist. "
    echo "Try: pip install fourier-grx-dds"
    return
fi

echo "# Add fourier-grx-dds lib " >> ~/.bashrc
# 系统默认路径优先
echo "export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib:/lib64:\$LD_LIBRARY_PATH" >> ~/.bashrc
# Set LD_LIBRARY_PATH
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$fourier_grx_dds_libraries" >> ~/.bashrc

echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$(python -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")" >> ~/.bashrc
