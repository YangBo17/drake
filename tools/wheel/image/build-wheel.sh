#!/usr/bin/env bash

# Internal script to build a wheel from a Drake installation.

set -eu -o pipefail

<<<<<<< HEAD
if [[ "$(uname)" == "Darwin" ]]; then
=======
if [ "$(uname)" == "Darwin" ]; then
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    HOMEBREW="$(brew config | \grep -E '^HOMEBREW_PREFIX' | cut -c18-)"

    # Use GNU 'cp' on macOS so we have a consistent CLI.
    cp()
    {
        gcp "$@"
    }
fi

# Helper function to change the RPATH of libraries. The first argument is the
# (origin-relative) new RPATH to be added. Remaining arguments are libraries to
# be modified. Note that existing RPATH(s) are removed (except for homebrew
# RPATHs on macOS).
chrpath()
{
    rpath=$1
    shift 1

    for lib in "$@"; do
<<<<<<< HEAD
        if [[ "$(uname)" == "Linux" ]]; then
=======
        if [ "$(uname)" == "Linux" ]; then
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
            patchelf --remove-rpath "$lib"
            patchelf --set-rpath "\$ORIGIN/$rpath" "$lib"
        else
            strip_rpath \
                --exclude="$HOMEBREW" \
                --exclude=/opt/drake-dependencies/lib \
                "$lib"
            install_name_tool -add_rpath "@loader_path/$rpath" "$lib"
        fi
    done
}

###############################################################################

readonly WHEEL_DIR=/opt/drake-wheel-build/wheel
<<<<<<< HEAD
readonly WHEEL_SHARE_DIR=${WHEEL_DIR}/pydrake/share
=======
readonly WHEEL_DATA_DIR=${WHEEL_DIR}/pydrake/share/drake
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

# TODO(mwoehlke-kitware) Most of this should move to Bazel.
mkdir -p ${WHEEL_DIR}/drake
mkdir -p ${WHEEL_DIR}/pydrake/lib
mkdir -p ${WHEEL_DIR}/pydrake/share/drake
cd ${WHEEL_DIR}

cp -r -t ${WHEEL_DIR}/drake \
    /opt/drake/lib/python*/site-packages/drake/*

cp -r -t ${WHEEL_DIR}/pydrake \
    /opt/drake/share/doc \
    /opt/drake/lib/python*/site-packages/pydrake/*

cp -r -t ${WHEEL_DIR}/pydrake/lib \
    /opt/drake/lib/libdrake*.so
<<<<<<< HEAD

if [[ "$(uname)" == "Linux" ]]; then
  cp -r -t ${WHEEL_DIR}/pydrake \
      /opt/drake-wheel-content/*
fi
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

# NOTE: build-vtk.sh also puts licenses in /opt/drake-dependencies/licenses.
cp -r -t ${WHEEL_DIR}/pydrake/doc \
    /opt/drake-dependencies/licenses/*

# MOSEK is "sort of" third party, but is procured as part of Drake's build and
# ends up in /opt/drake.
<<<<<<< HEAD
if [[ "$(uname)" == "Darwin" ]]; then
=======
if [ "$(uname)" == "Darwin" ]; then
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    # On macOS, it is explicitly referenced by @loader_path, and thus must be
    # copied to the same place as libdrake.so.
    cp -r -t ${WHEEL_DIR}/pydrake/lib \
        /opt/drake/lib/libmosek*.dylib \
<<<<<<< HEAD
        /opt/drake/lib/libtbb*.dylib
=======
        /opt/drake/lib/libcilkrts*.dylib
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
else
    # On Linux, it needs to be copied somewhere where auditwheel can find it.
    cp -r -t /opt/drake-dependencies/lib \
        /opt/drake/lib/libmosek*.so* \
<<<<<<< HEAD
        /opt/drake/lib/libtbb*.so*
fi

cp -r -t ${WHEEL_SHARE_DIR}/drake \
=======
        /opt/drake/lib/libcilkrts*.so*
fi

# TODO(mwoehlke-kitware) We need a different way of shipping non-arch files
# (examples, models).
cp -r -t ${WHEEL_DATA_DIR} \
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    /opt/drake/share/drake/.drake-find_resource-sentinel \
    /opt/drake/share/drake/package.xml \
    /opt/drake/share/drake/examples \
    /opt/drake/share/drake/geometry \
    /opt/drake/share/drake/manipulation \
    /opt/drake/share/drake/multibody \
    /opt/drake/share/drake/tutorials
# TODO(#15774) Eventually we will download these at runtime, instead of shipping
# them in the wheel.
cp -r -t ${WHEEL_SHARE_DIR} \
    /opt/drake/share/drake_models

<<<<<<< HEAD
if [[ "$(uname)" == "Linux" ]]; then
    mkdir -p ${WHEEL_SHARE_DIR}/drake/setup
    cp -r -t ${WHEEL_SHARE_DIR}/drake/setup \
=======
if [ "$(uname)" == "Linux" ]; then
    mkdir -p ${WHEEL_DATA_DIR}/setup
    cp -r -t ${WHEEL_DATA_DIR}/setup \
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        /opt/drake/share/drake/setup/deepnote
fi

# TODO(mwoehlke-kitware) We need to remove these to keep the wheel from being
<<<<<<< HEAD
# too large, but (per above), the whole of share/drake_models shouldn't be in
# the wheel (and atlas's meshes should move into drake_models).
rm -rf \
    ${WHEEL_SHARE_DIR}/drake_models/franka_description \
    ${WHEEL_SHARE_DIR}/drake_models/ur3e \
    ${WHEEL_SHARE_DIR}/drake_models/ycb \
    ${WHEEL_SHARE_DIR}/drake/examples/atlas \
    ${WHEEL_SHARE_DIR}/drake_models/wsg_50_hydro_bubble

if [[ "$(uname)" == "Linux" ]]; then
=======
# too large, but (per above), the whole of share/drake shouldn't be in the
# wheel.
rm -rf \
    ${WHEEL_DATA_DIR}/manipulation/models/franka_description/meshes \
    ${WHEEL_DATA_DIR}/manipulation/models/tri-homecart/*.obj \
    ${WHEEL_DATA_DIR}/manipulation/models/tri-homecart/*.png \
    ${WHEEL_DATA_DIR}/manipulation/models/ur3e/*.obj \
    ${WHEEL_DATA_DIR}/manipulation/models/ur3e/*.png \
    ${WHEEL_DATA_DIR}/manipulation/models/ycb/meshes \
    ${WHEEL_DATA_DIR}/examples/atlas \
    ${WHEEL_DATA_DIR}/examples/hydroelastic/spatula_slip_control

if [ "$(uname)" == "Linux" ]; then
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    export LD_LIBRARY_PATH=${WHEEL_DIR}/pydrake/lib:/opt/drake-dependencies/lib
fi

chrpath lib pydrake/*.so
chrpath ../lib pydrake/*/*.so

<<<<<<< HEAD
if [[ "$(uname)" == "Darwin" ]]; then
=======
if [ "$(uname)" == "Darwin" ]; then
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    change_lpath \
        --old='@loader_path/../../../' \
        --new='@rpath/' \
        pydrake/*.so
    change_lpath \
        --old='@loader_path/../../../../' \
        --new='@rpath/' \
        pydrake/*/*.so
fi

python setup.py bdist_wheel

<<<<<<< HEAD
if [[ "$(uname)" == "Darwin" ]]; then
=======
if [ "$(uname)" == "Darwin" ]; then
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    delocate-wheel -w wheelhouse -v dist/drake*.whl
else
    GLIBC_VERSION=$(ldd --version | sed -n '1{s/.* //;s/[.]/_/p}')

    auditwheel repair --plat manylinux_${GLIBC_VERSION}_x86_64 dist/drake*.whl
fi
