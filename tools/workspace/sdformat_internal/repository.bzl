<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
<<<<<<< HEAD
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "gazebosim/sdformat",
        commit = "sdformat13_13.3.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "c21c70eb9a7d436d1cd083d9d5c4fce2ff645bdb27fc86332eab87d0ef081e6b",  # noqa
        patches = [
=======
        repository = "gazebosim/sdformat",
        commit = "sdformat12_12.5.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "3896772db68b7ca7b18bbf1945a72206885b03d3f0caf29491be5b53b79a7124",  # noqa
        patches = [
            ":patches/1043.patch",
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
