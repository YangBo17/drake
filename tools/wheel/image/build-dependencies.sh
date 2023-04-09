#!/usr/bin/env bash

# Internal script to build dependencies for a Drake wheel.

set -eu -o pipefail

mkdir -p /opt/drake-wheel-build/dependencies/build
cd /opt/drake-wheel-build/dependencies/build

cmake -G Ninja \
    -DCMAKE_INSTALL_PREFIX=/opt/drake-dependencies \
    /opt/drake-wheel-build/dependencies/src

ninja

<<<<<<< HEAD
if [[ "$(uname)" == "Linux" ]]; then
=======
if [ "$(uname)" == "Linux" ]; then
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    ln -s /opt/drake-dependencies/bin/patchelf /usr/local/bin/patchelf

    # Libraries we get from the distro that get bundled into the wheel need to
    # have their licenses bundled also.
    mkdir -p /opt/drake-dependencies/licenses/mumps
    cp -t /opt/drake-dependencies/licenses/mumps \
        /usr/share/doc/libmumps-seq-dev/copyright
<<<<<<< HEAD
    mkdir -p /opt/drake-dependencies/licenses/gcc
    cp -t /opt/drake-dependencies/licenses/gcc \
        /usr/share/doc/libgomp1/copyright
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
fi
