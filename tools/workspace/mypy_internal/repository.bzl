<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
<<<<<<< HEAD
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "python/mypy",
<<<<<<< HEAD
        commit = "v1.0.1",
        sha256 = "44e971ab71dc8d06fee3c72f03d80cf0ee39d2dd3a65da1ecdd8007861dc17c4",  # noqa
=======
        commit = "v1.0.0",
        sha256 = "26c144b9a01c0dc244ea4273cdabe7a695e302d5ba6423f2e81c4968b1f0c062",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/reject_double_colon.patch",
            ":patches/timeout.patch",
        ],
=======
        repository = "python/mypy",
        # TODO(mwoehlke-kitware): switch to a tag >= v0.980.
        commit = "7c6faf4c7a7bac6b126a30c5c61f5d209a4312c0",
        sha256 = "f0ab4a26f0f75fc345865b17e4c21f34f13b7d9220eab663e96116dd326b9e48",  # noqa
        build_file = ":package.BUILD.bazel",
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        mirrors = mirrors,
    )
