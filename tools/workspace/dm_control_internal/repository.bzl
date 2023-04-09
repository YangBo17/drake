<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
<<<<<<< HEAD
        commit = "774f46182140106e22725914aad3c6299ed91edd",
        sha256 = "69a825c9b984453e3c8e91f383558b83d3f2b0305c5ab7b21d0772cb1da9c659",  # noqa
=======
        commit = "41d0c7383153f9ca6c12f8e865ef5e73a98759bd",
        sha256 = "712fbc5b81518b640b1f1c084b22bebc558a8acecec4831ef5f6ca63181431d6",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
