<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
<<<<<<< HEAD
        commit = "6751ba850fd46fee789a095edba7ad9fc99cd188",
        sha256 = "dc38658f7acb4d07442e6dae4d18818d31ac6cc6d66bf16913ef1b208692627c",  # noqa
=======
        commit = "e6bb254c786ef4d67aea7e88d141b3bb2618e964",
        sha256 = "e246342c6d4e895dbc9d4d7206889114e6a6c99857987708f01066d839978beb",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
