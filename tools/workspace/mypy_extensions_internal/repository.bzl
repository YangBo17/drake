<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
<<<<<<< HEAD
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "python/mypy_extensions",
        commit = "1.0.0",
        sha256 = "c1f1fc0cc5f5be7d3a70b6dd4b85f9e2b02d788d66f3a168652a65df6571df07",  # noqa
=======
        repository = "python/mypy_extensions",
<<<<<<< HEAD
        commit = "0.4.3",
        sha256 = "21e830f4baf996d0a9bd7048a5b23722dbd3b88354b8bf0e22376f9a8d508c16",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
        commit = "1.0.0",
        sha256 = "c1f1fc0cc5f5be7d3a70b6dd4b85f9e2b02d788d66f3a168652a65df6571df07",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
