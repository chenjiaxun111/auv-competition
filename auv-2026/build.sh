#!/bin/sh

# 首次部署时安装系统级依赖，然后编译主程序。
# libev源码由主项目CMake自动编译，不需要单独configure或install。

set -eu

PROJECT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

for command_name in cmake make gcc; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "缺少命令：$command_name" >&2
        exit 1
    fi
done

echo "========== 编译并安装 WiringNP =========="
cd "$PROJECT_DIR/lib/WiringNP"
chmod 755 build
./build

echo "========== 编译并安装 EasyLogger =========="
cd "$PROJECT_DIR/lib/easylogger"
make clean
make
sudo install -m 755 libeasylogger.so /usr/local/lib/libeasylogger.so

sudo ldconfig

echo "========== 编译 AUV Master =========="
cd "$PROJECT_DIR"
make

echo "构建完成：$PROJECT_DIR/build/auvmaster"
