<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
load("@drake//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
<<<<<<< HEAD
        commit = "4.5.0",
        sha256 = "c8fd5561e1bd88b743ef2ee065a5e661b2fd7b56e9cbe9ae2aeb928f41438819",  # noqa
=======
        commit = "4.3.0",
        sha256 = "9dbc928aed2839a23d210726697700a1c4593ab3bbf82b981fcc44585a47ce30",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
