<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("//tools/workspace:github.bzl", "github_archive")

def gz_utils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
<<<<<<< HEAD
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "gazebosim/gz-utils",
        commit = "gz-utils2_2.0.0",
        sha256 = "af9e5b862e10aa0cedd97d9c5ca3eb9a443b7c9e560a083e8f0399e93e1cfafa",  # noqa
=======
        repository = "gazebosim/gz-utils",
        commit = "ignition-utils1_1.4.0",
        sha256 = "74ba40f8a35f9ae07102402d4acde11e4f9a1d52663a339388fe086527093f86",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
