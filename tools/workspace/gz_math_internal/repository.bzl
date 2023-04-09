<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("//tools/workspace:github.bzl", "github_archive")

def gz_math_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
<<<<<<< HEAD
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "gazebosim/gz-math",
        commit = "gz-math7_7.1.0",
        sha256 = "3df09a16b84fa27fabf4955b5efb207f417ef9b0b5b801ae28cfda6d8e11765a",  # noqa
=======
        repository = "gazebosim/gz-math",
        commit = "ignition-math6_6.11.0",
        sha256 = "e6b8901c94147e2c2659323083ce1d151495a07f9bef72a957069ce5b9f3d9e8",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
