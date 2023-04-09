#!/bin/bash

# Internal script to build a Drake from which a wheel will be created.
# Docker (Linux) only.

set -eu -o pipefail

cd /opt/drake-wheel-build/drake

git apply < /image/pip-drake.patch

export SNOPT_PATH=git

bazel run \
    --disk_cache=/var/cache/bazel/disk_cache \
    --repository_cache=/var/cache/bazel/repository_cache \
    --repo_env=DRAKE_OS=manylinux \
<<<<<<< HEAD
    --config=omp \
    --define=WITH_MOSEK=ON \
    --define=WITH_SNOPT=ON \
=======
    --define WITH_MOSEK=ON \
    --define WITH_SNOPT=ON \
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
    //:install -- /opt/drake
