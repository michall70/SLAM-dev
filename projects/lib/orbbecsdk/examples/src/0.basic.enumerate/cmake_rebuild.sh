#!/bin/bash

# 获取脚本所在目录（即源文件目录，CMakeLists.txt 所在处）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "当前目录: $(pwd)"

# 如果存在 build 目录则删除
if [ -d "build" ]; then
    echo "删除旧的 build 目录..."
    rm -rf build
fi

# 新建 build 目录
echo "创建 build 目录..."
mkdir build

# 进入 build 目录
cd build

# 运行 cmake 和 make
echo "运行 cmake .."
cmake ..

if [ $? -eq 0 ]; then
    echo "cmake 配置成功，开始编译..."
    make -j$(nproc)
    if [ $? -eq 0 ]; then
        echo "编译完成！"
    else
        echo "make 编译失败，请检查错误信息。"
        exit 2
    fi
else
    echo "cmake 配置失败，请检查 CMakeLists.txt 或依赖项。"
    exit 1
fi
