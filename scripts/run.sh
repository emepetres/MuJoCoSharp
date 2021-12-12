#!/bin/bash
set -e

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
pushd $SCRIPT_DIR/..

if [[ ! -d "./lib/mujoco210" ]]
then
    mkdir -p lib
    pushd lib
    wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz
    tar -xf mujoco210-linux-*.tar.gz
    rm mujoco210-linux-*.tar.gz
    popd
fi

swig -csharp src/mujoco.i
gcc -c -fpic src/mujoco_wrap.c

popd
