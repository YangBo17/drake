load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
<<<<<<< HEAD
        commit = "2.10.0",
        sha256 = "a7306561f1ddf7bc00419b9f0d698d312a8eaa173b834e7c8e4ff32db5efd27f",  # noqa
=======
        commit = "2.8.0",
        sha256 = "9116bd3686beaa22be34be1e5259fb9eecbf246a3991849d33ff6ab07d52f86e",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
