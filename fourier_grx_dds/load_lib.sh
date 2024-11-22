#!/bin/bash
# 设置动态库路径
load_lib_dir=$(dirname "$(realpath "$0")")
export LD_LIBRARY_PATH=$load_lib_dir/libraries:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(python3 -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))"):$LD_LIBRARY_PATH
# 输出验证
echo "LD_LIBRARY_PATH set to: $LD_LIBRARY_PATH"

