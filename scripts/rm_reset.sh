#!/bin/bash
# 重置工作空间脚本

cd /home/thom/RM_version_ws
echo "清理编译文件..."
rm -rf build/ install/
echo "重新编译..."
colcon build --symlink-install
echo "重置完成！"
