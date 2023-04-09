load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
<<<<<<< HEAD
        commit = "v1.13.0",
        sha256 = "ad7fdba11ea011c1d925b3289cf4af2c66a352e18d4c7264392fead75e919363",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/add_printers.patch",
        ],
=======
        commit = "release-1.12.1",
        sha256 = "81964fe578e9bd7c94dfdb09c8e4d6e6759e19967e397dbea48d1c10e45d0df2",  # noqa
        build_file = ":package.BUILD.bazel",
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        mirrors = mirrors,
    )
