#!/bin/sh

# 单独重新编译第三方库。
# 用法：./compile.sh wiringnp | easylogger
# libev由主项目CMake自动编译，不需要单独处理。

set -eu

PROJECT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
LIBRARY=${1:-}

if [ -z "$LIBRARY" ]; then
    echo "请选择：wiringnp 或 easylogger" >&2
    echo "例如：./compile.sh easylogger" >&2
    exit 1
fi

case "$LIBRARY" in
    wiringnp)
        cd "$PROJECT_DIR/lib/WiringNP"
        chmod 755 build
        ./build
        ;;
    easylogger)
        cd "$PROJECT_DIR/lib/easylogger"
        make clean
        make
        sudo install -m 755 libeasylogger.so /usr/local/lib/libeasylogger.so
        sudo ldconfig
        ;;
    *)
        echo "未知选项：$LIBRARY" >&2
        exit 1
        ;;
esac
